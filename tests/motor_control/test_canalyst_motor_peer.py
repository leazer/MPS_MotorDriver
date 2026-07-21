import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


CONTROL = load_module(
    "canalyst_controlcan",
    ROOT / "tests" / "canalyst_controlcan.py",
)
MODULE = load_module(
    "canalyst_motor_peer_under_test",
    ROOT / "tests" / "canalyst_motor_peer.py",
)
CanFrame = CONTROL.CanFrame


class FakeDevice:
    def __init__(self, received=(), fail_send_ids=()):
        self.received = [list(batch) for batch in received]
        self.fail_send_ids = set(fail_send_ids)
        self.sent = []
        self.receive_calls = []

    def send(self, frame):
        assert isinstance(frame, CanFrame)
        self.sent.append(frame)
        if frame.can_id in self.fail_send_ids:
            raise RuntimeError(f"send failed for {frame.can_id:#x}")

    def receive(self, max_frames, wait_ms):
        self.receive_calls.append((max_frames, wait_ms))
        return self.received.pop(0) if self.received else []


def expect_error(callback, error_type=(TypeError, ValueError)):
    try:
        callback()
    except error_type as caught:
        return caught
    raise AssertionError("expected rejection")


def test_broadcast_golden_vectors():
    expected = {
        MODULE.OPCODE_ARM: "01 01 34 12 78 56 00 20",
        MODULE.OPCODE_SYNC: "02 01 02 10 22 20 00 08",
        MODULE.OPCODE_STOP: "03 01 03 10 23 20 00 DE",
        MODULE.OPCODE_CLEAR_FAULT: "04 01 04 10 24 20 00 F2",
        MODULE.OPCODE_DISCOVER: "05 01 05 10 25 20 00 24",
    }
    for index, (opcode, vector) in enumerate(expected.items()):
        sequence = 0x1234 if index == 0 else 0x1001 + index
        session = 0x5678 if index == 0 else 0x2021 + index
        frame = MODULE.encode_broadcast(opcode, sequence, session)
        assert frame == CanFrame(0x080, bytes.fromhex(vector))
    assert MODULE.crc8(bytes.fromhex("01 01 34 12 78 56 00")) == 0x20


def test_signed_trajectory_golden_vector_and_exact_velocity_scaling():
    assert MODULE.encode_trajectory(-123456, -3210, 0xCAFE) == CanFrame(
        0x101, bytes.fromhex("C0 1D FE FF BF FE FE CA")
    )
    assert MODULE.encode_trajectory(0x7FFFFFFF, 327670, 0xFFFF).data == (
        bytes.fromhex("FF FF FF 7F FF 7F FF FF")
    )
    assert MODULE.encode_trajectory(-0x80000000, -327680, 0).data == (
        bytes.fromhex("00 00 00 80 00 80 00 00")
    )
    for arguments in (
        (0x80000000, 0, 0),
        (-0x80000001, 0, 0),
        (0, 327680, 0),
        (0, -327690, 0),
        (0, -3211, 0),
        (0, 0, 0x10000),
    ):
        expect_error(lambda arguments=arguments: MODULE.encode_trajectory(*arguments))
    for arguments in ((True, 0, 0), (0, True, 0), (0, 0, True)):
        expect_error(
            lambda arguments=arguments: MODULE.encode_trajectory(*arguments),
            TypeError,
        )


def test_feedback_and_health_decoding_golden_vectors():
    feedback = MODULE.decode_feedback(
        CanFrame(0x181, bytes.fromhex("C0 1D FE FF BF FE FE CA"))
    )
    assert feedback == MODULE.Feedback(-123456, -3210, 0xCAFE)
    health = MODULE.decode_health(
        CanFrame(0x281, bytes.fromhex("01 05 34 12 78 56 BC 9A"))
    )
    assert health == MODULE.Health(1, 5, 0x1234, 0x5678, 0x9ABC)


def test_decoders_reject_wrong_id_dlc_version_and_state():
    valid_feedback = bytes.fromhex("C0 1D FE FF BF FE FE CA")
    valid_health = bytes.fromhex("01 05 34 12 78 56 BC 9A")
    for frame in (
        CanFrame(0x182, valid_feedback),
        CanFrame(0x181, valid_feedback[:-1]),
    ):
        expect_error(lambda frame=frame: MODULE.decode_feedback(frame))
    for frame in (
        CanFrame(0x282, valid_health),
        CanFrame(0x281, valid_health[:-1]),
        CanFrame(0x281, bytes.fromhex("02 05 34 12 78 56 BC 9A")),
        CanFrame(0x281, bytes.fromhex("01 06 34 12 78 56 BC 9A")),
    ):
        expect_error(lambda frame=frame: MODULE.decode_health(frame))


def test_command_methods_send_exact_frames_and_stop_once_per_call():
    device = FakeDevice()
    peer = MODULE.CanalystMotorPeer(device, node_id=1)
    peer.discover()
    peer.arm(0x5678, sequence=0x1234)
    peer.stop(session=0x2023, sequence=0x1003)
    peer.stop()
    assert device.sent == [
        MODULE.encode_broadcast(MODULE.OPCODE_DISCOVER, 0, 0),
        MODULE.encode_broadcast(MODULE.OPCODE_ARM, 0x1234, 0x5678),
        MODULE.encode_broadcast(MODULE.OPCODE_STOP, 0x1003, 0x2023),
        MODULE.encode_broadcast(MODULE.OPCODE_STOP, 0, 0),
    ]


def test_preload_must_succeed_before_matching_sync():
    device = FakeDevice(fail_send_ids={0x101})
    peer = MODULE.CanalystMotorPeer(device, node_id=1)
    try:
        peer.apply_point(0x2222, 7, 33596, 0)
    except RuntimeError:
        pass
    else:
        raise AssertionError("preload failure was swallowed")
    assert [frame.can_id for frame in device.sent] == [0x101]


def test_apply_point_preloads_then_syncs_same_session_and_sequence():
    device = FakeDevice()
    peer = MODULE.CanalystMotorPeer(device, node_id=1)
    peer.apply_point(0x2222, 7, 33596, -120)
    assert device.sent == [
        MODULE.encode_trajectory(33596, -120, 7),
        MODULE.encode_broadcast(MODULE.OPCODE_SYNC, 7, 0x2222),
    ]
    expect_error(lambda: peer.sync(0x2222, 8), RuntimeError)
    assert len(device.sent) == 2


def test_command_sequence_rejection_is_wrap_aware_and_arm_resets_window():
    device = FakeDevice()
    peer = MODULE.CanalystMotorPeer(device, node_id=1)
    peer.submit(0, 0, 0xFFFF)
    peer.sync(1, 0xFFFF)
    peer.submit(1, 10, 0)
    peer.sync(1, 0)
    expect_error(lambda: peer.submit(2, 20, 0xFFFF), ValueError)
    peer.arm(2, sequence=0xFFFF)
    peer.submit(3, 30, 0xFFFF)


def test_readers_filter_unrelated_malformed_and_stale_frames():
    feedback_old = CanFrame(0x181, bytes.fromhex("01 00 00 00 01 00 FF FF"))
    feedback_wrap = CanFrame(0x181, bytes.fromhex("02 00 00 00 02 00 00 00"))
    stale_feedback = CanFrame(0x181, bytes.fromhex("03 00 00 00 03 00 FE FF"))
    stale_health = CanFrame(0x281, bytes.fromhex("01 01 00 00 06 00 10 27"))
    current_health = CanFrame(0x281, bytes.fromhex("01 02 00 00 07 00 11 27"))
    device = FakeDevice(received=[
        [CanFrame(0x123, b"junk"), stale_health, feedback_old, current_health],
        [feedback_wrap],
        [stale_feedback],
    ])
    peer = MODULE.CanalystMotorPeer(device, node_id=1)
    peer.arm(7)
    assert peer.read_health() == MODULE.Health(1, 2, 0, 7, 10001)
    assert peer.read_feedback() == MODULE.Feedback(1, 10, 0xFFFF)
    assert peer.read_feedback() == MODULE.Feedback(2, 20, 0)
    assert peer.read_feedback() is None
    assert device.receive_calls == [(64, 0), (64, 0), (64, 0)]


def test_peer_rejects_wrong_node_and_incomplete_device_without_hardware_access():
    expect_error(lambda: MODULE.CanalystMotorPeer(FakeDevice(), node_id=2))
    expect_error(lambda: MODULE.CanalystMotorPeer(object(), node_id=1), TypeError)


if __name__ == "__main__":
    test_broadcast_golden_vectors()
    test_signed_trajectory_golden_vector_and_exact_velocity_scaling()
    test_feedback_and_health_decoding_golden_vectors()
    test_decoders_reject_wrong_id_dlc_version_and_state()
    test_command_methods_send_exact_frames_and_stop_once_per_call()
    test_preload_must_succeed_before_matching_sync()
    test_apply_point_preloads_then_syncs_same_session_and_sequence()
    test_command_sequence_rejection_is_wrap_aware_and_arm_resets_window()
    test_readers_filter_unrelated_malformed_and_stale_frames()
    test_peer_rejects_wrong_node_and_incomplete_device_without_hardware_access()
    print("CANalyst Motor peer tests passed")
