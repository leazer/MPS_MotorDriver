from contextlib import redirect_stderr
import importlib.util
from io import StringIO
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[2]
TESTS = ROOT / "tests"
LAUNCHER_PATH = TESTS / "stage8_canalyst_node1.py"
SPEC = importlib.util.spec_from_file_location(
    "stage8_canalyst_node1_under_test", LAUNCHER_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def expect_error(callback, error_type=Exception, match=None):
    try:
        callback()
    except error_type as error:
        if match is not None:
            assert match in str(error), str(error)
        return error
    raise AssertionError(f"expected {error_type.__name__}")


def position_line(**changes):
    values = {
        "active": 1,
        "target": 1000,
        "velocity": 0,
        "reference": 1000,
        "measured": 1000,
        "error": 0,
        "speed_target": 0,
        "speed_measured": 0,
        "iqref": 100,
        "age": 1,
        "timeout": 0,
        "sequence": 2,
        "fault": 0,
    }
    values.update(changes)
    check = 0x504F5331
    for value in values.values():
        check ^= value & 0xFFFFFFFF
    return (
        "ps a={active} t={target} v={velocity} r={reference} m={measured} "
        "e={error} w={speed_target} x={speed_measured} q={iqref} g={age} "
        "o={timeout} n={sequence} f={fault:08X} k={check:08X}\r\nmsh />"
    ).format(**values, check=check)


def speed_line(**changes):
    values = {
        "active": 0,
        "target": 0,
        "cmd": 0,
        "meas": 0,
        "iqref": 0,
        "id": 0,
        "iq": 0,
        "invalid": 4,
        "streak": 0,
        "freeze": 2,
        "fault": 0,
        "enc": 12345,
    }
    values.update(changes)
    check = 0x53504436
    for value in values.values():
        check ^= value & 0xFFFFFFFF
    return (
        "ss a={active} t={target} c={cmd} m={meas} u={iqref} d={id} q={iq} "
        "n={invalid} s={streak} z={freeze} f={fault:08X} e={enc} "
        "k={check:08X}\r\nmsh />"
    ).format(**values, check=check)


def can_line(**changes):
    values = {
        "id": 1,
        "s": 1,
        "se": 0,
        "p": 0,
        "a": 0,
        "pa": 0,
        "sa": 0,
        "rx": 10,
        "tx": 20,
        "pe": 0,
        "ro": 0,
        "bo": 0,
        "te": 0,
        "f": 0,
    }
    values.update(changes)
    check = 0x43414E31
    for key in (
        "id", "s", "se", "p", "a", "pa", "sa", "rx", "tx", "pe",
        "ro", "bo", "te", "f",
    ):
        check ^= values[key] & 0xFFFFFFFF
    return (
        "cs id={id} s={s} se={se} p={p} a={a} pa={pa} sa={sa} rx={rx} "
        "tx={tx} pe={pe} ro={ro} bo={bo} te={te} f={f:08X} "
        "k={check:08X}\r\nmsh />"
    ).format(**values, check=check)


def joint_config_text(**changes):
    values = {
        "record_present": 1,
        "version": 1,
        "generation": 7,
        "node": 1,
        "zero_raw": 12345,
        "known_mdeg": 0,
        "min_mdeg": -90000,
        "max_mdeg": 90000,
        "direction": 1,
        "crc_valid": 1,
        "encoder_ready": 1,
        "restored_joint_mdeg": 0,
        "restored_joint_valid": 1,
        "service_ready": 1,
        "mutation_busy": 0,
        "runtime_locked": 0,
        "reboot_required": 0,
    }
    values.update(changes)
    body = "\r\n".join(
        f"{key:<21}: {value}" for key, value in values.items()
    )
    return f"=== joint_config ===\r\n{body}\r\nmsh />"


SAFE_PWM_REPLY = (
    "=== TMR1 PWM ===\r\n"
    "CCR1/2/3 : 2812 / 2812 / 2812\r\n"
    "CCR4     : 5264 (ADC trigger)\r\n"
    "EN(PB10) : 0\r\n"
    "msh />"
)


class FakeSerial:
    def __init__(self, events=None, replies=None):
        self.events = [] if events is None else events
        self.replies = {
            "joint_cfg_show": [joint_config_text()],
            "mc_speed_status": [speed_line(), speed_line()],
            "can_status": [can_line(), can_line(rx=12, tx=22)],
            "mc_pos_status": [
                position_line(active=1, measured=1000, sequence=2),
                position_line(active=0, measured=1000, sequence=2, iqref=0),
            ],
            "mc_stop": ["all modes stopped\r\nmsh />"],
            "pwm_info": [SAFE_PWM_REPLY],
        }
        if replies:
            self.replies.update({key: list(value) for key, value in replies.items()})
        self.pending = b""
        self.closed = False

    def reset_input_buffer(self):
        self.pending = b""

    def reset_output_buffer(self):
        pass

    def write(self, data):
        command = data.decode("ascii").rstrip("\r")
        if command == "mc_stop":
            self.events.append("mc_stop")
        elif command == "pwm_info":
            self.events.append("pwm_safe")
        queue = self.replies.get(command)
        if not queue:
            raise AssertionError(f"unexpected serial command {command!r}")
        reply = queue.pop(0) if len(queue) > 1 else queue[0]
        self.pending = reply.encode("ascii")

    def flush(self):
        pass

    @property
    def in_waiting(self):
        return len(self.pending)

    def read(self, count):
        result = self.pending[:count]
        self.pending = self.pending[count:]
        return result

    def close(self):
        self.events.append("serial_close")
        self.closed = True


class FakePeer:
    def __init__(self, events=None, *, fail_after_arm=False, final_position=1000,
                 health=None):
        self.events = [] if events is None else events
        self.fail_after_arm = fail_after_arm
        self.health = list(health or [
            SimpleNamespace(
                protocol_version=1, node_state=1, fault_bits=0,
                session=0, vbus_10mv=1200,
            )
        ])
        self.feedback = [
            SimpleNamespace(
                actual_position_mdeg=0, actual_velocity_mdeg_s=0,
                applied_sequence=0,
            ),
            SimpleNamespace(
                actual_position_mdeg=final_position, actual_velocity_mdeg_s=0,
                applied_sequence=2,
            ),
        ]

    def stop(self, *args, **kwargs):
        del args, kwargs
        self.events.append("stop")

    def discover(self):
        self.events.append("discover")

    def read_health(self):
        return self.health.pop(0) if len(self.health) > 1 else self.health[0]

    def read_feedback(self):
        return self.feedback.pop(0) if self.feedback else None

    def arm(self, session, sequence=0):
        self.events.append("arm")
        assert session != 0
        if self.fail_after_arm:
            raise RuntimeError("injected after ARM")

    def apply_point(self, session, sequence, position_mdeg, velocity_mdeg_s):
        del session, position_mdeg, velocity_mdeg_s
        self.events.extend(("submit", "sync"))
        assert sequence in (1, 2)


def config_mapping(**changes):
    result = {
        "record_present": 1,
        "version": 1,
        "generation": 7,
        "node": 1,
        "zero_raw": 12345,
        "known_mdeg": 0,
        "min_mdeg": -90000,
        "max_mdeg": 90000,
        "direction": 1,
        "crc_valid": 1,
        "encoder_ready": 1,
        "restored_joint_mdeg": 0,
        "restored_joint_valid": 1,
        "service_ready": 1,
        "mutation_busy": 0,
        "runtime_locked": 0,
        "reboot_required": 0,
    }
    result.update(changes)
    return result


def test_cli_exposes_only_explicit_phases_and_required_dll():
    parser = MODULE.build_parser()
    for phase in ("probe", "discover", "step1", "step5"):
        args = parser.parse_args(["--phase", phase, "--dll", "x.dll"])
        assert args.phase == phase
        assert args.port == "COM9"
        assert args.evidence_dir == Path("artifacts/stage8-canalyst")
    with redirect_stderr(StringIO()):
        expect_error(
            lambda: parser.parse_args(["--phase", "sine", "--dll", "x.dll"]),
            SystemExit,
        )
        expect_error(lambda: parser.parse_args(["--phase", "probe"]), SystemExit)


def test_probe_resolves_exports_without_opening_device():
    events = []

    class ProbeDevice:
        def __init__(self, path):
            events.append(("load", Path(path)))
            self.library = SimpleNamespace(**{
                name: object() for name in MODULE.REQUIRED_CONTROLCAN_EXPORTS
            })

        def open(self):
            events.append("open")
            raise AssertionError("probe opened CAN")

    result = MODULE.probe_dll("vendor.dll", device_factory=ProbeDevice)
    assert result["phase"] == "probe"
    assert result["passed"] is True
    assert events == [("load", Path("vendor.dll"))]


def test_discover_phase_cannot_arm_or_send_trajectory():
    events = []
    result = MODULE.run_phase(
        "discover", peer=FakePeer(events), serial=FakeSerial(events),
        firmware_id="fw-a",
    )
    assert result["node_state"] == "READY"
    assert events[:4] == ["stop", "stop", "stop", "discover"]
    assert all(event not in ("arm", "submit", "sync") for event in events)


def test_discover_rejects_can_error_counter_growth():
    serial = FakeSerial(replies={
        "can_status": [can_line(), can_line(rx=11, tx=21, ro=1)],
    })
    expect_error(
        lambda: MODULE.run_phase(
            "discover", peer=FakePeer(), serial=serial, firmware_id="fw-a"
        ),
        AssertionError,
        "counter grew",
    )


def test_step_target_is_relative_and_workspace_checked():
    assert MODULE.relative_target(32596, 1000, -90000, 90000) == 33596
    for args in (
        (89500, 1000, -90000, 90000),
        (0, 5000, -4000, 4000),
    ):
        expect_error(lambda args=args: MODULE.relative_target(*args), ValueError)


def test_failure_still_runs_all_cleanup_layers():
    events = []
    peer = FakePeer(events, fail_after_arm=True)
    serial = FakeSerial(events)
    expect_error(
        lambda: MODULE.run_phase(
            "step1", peer=peer, serial=serial, firmware_id="fw-a"
        ),
        RuntimeError,
        "injected after ARM",
    )
    assert events[-5:] == ["stop", "stop", "stop", "mc_stop", "pwm_safe"]


def test_health_requires_ready_session_zero_fault_free_and_6_to_30_vbus():
    good = SimpleNamespace(
        protocol_version=1, node_state=1, fault_bits=0,
        session=0, vbus_10mv=600,
    )
    assert MODULE.validate_health(good)["node_state"] == "READY"
    assert MODULE.validate_health(SimpleNamespace(**{
        **vars(good), "vbus_10mv": 3000,
    }))["vbus_mv"] == 30000
    for changes in (
        {"protocol_version": 2}, {"node_state": 2}, {"fault_bits": 1},
        {"session": 1}, {"vbus_10mv": 599}, {"vbus_10mv": 3001},
    ):
        bad = SimpleNamespace(**{**vars(good), **changes})
        expect_error(lambda bad=bad: MODULE.validate_health(bad), AssertionError)


def test_com9_preflight_requires_complete_valid_config_and_ten_degree_margin():
    good = config_mapping()
    checked = MODULE.validate_preflight(good, current_mdeg=0)
    assert checked["node"] == 1
    for changes in (
        {"record_present": 0}, {"crc_valid": 0}, {"encoder_ready": 0},
        {"restored_joint_valid": 0}, {"service_ready": 0},
        {"mutation_busy": 1}, {"reboot_required": 1}, {"node": 2},
        {"direction": 0}, {"min_mdeg": 100}, {"max_mdeg": -100},
    ):
        bad = config_mapping(**changes)
        expect_error(
            lambda bad=bad: MODULE.validate_preflight(bad, current_mdeg=0),
            AssertionError,
        )
    expect_error(
        lambda: MODULE.validate_preflight(good, current_mdeg=80500),
        AssertionError,
        "10-degree",
    )


def test_step5_requires_passing_step1_for_same_firmware_and_config():
    with TemporaryDirectory(dir=ROOT) as directory:
        tmp_path = Path(directory)
        config = config_mapping()
        expect_error(
            lambda: MODULE.require_step1_evidence(tmp_path, "fw-a", config),
            RuntimeError,
        )
        candidates = [
            {"phase": "step1", "passed": False, "cleanup_complete": True,
             "firmware_id": "fw-a", "config": config},
            {"phase": "step1", "passed": True, "cleanup_complete": True,
             "firmware_id": "fw-b", "config": config},
            {"phase": "step1", "passed": True, "cleanup_complete": True,
             "firmware_id": "fw-a", "config": config_mapping(generation=8)},
            {"phase": "step1", "passed": True, "cleanup_complete": False,
             "firmware_id": "fw-a", "config": config},
        ]
        for index, candidate in enumerate(candidates):
            (tmp_path / f"bad-{index}.json").write_text(json.dumps(candidate))
        expect_error(
            lambda: MODULE.require_step1_evidence(tmp_path, "fw-a", config),
            RuntimeError,
        )
        passing = dict(candidates[0], passed=True, cleanup_complete=True)
        path = tmp_path / "passing.json"
        path.write_text(json.dumps(passing))
        assert MODULE.require_step1_evidence(tmp_path, "fw-a", config) == path


def test_encoder_and_can_error_counters_may_not_grow():
    speed_before = {"invalid": 4, "freeze": 2, "streak": 0, "fault": 0}
    speed_after = dict(speed_before)
    can_before = {"pe": 0, "ro": 0, "bo": 0, "te": 0, "rx": 10, "tx": 20}
    can_after = dict(can_before, rx=100, tx=200)
    assert MODULE.validate_counter_growth(
        speed_before, speed_after, can_before, can_after
    )["can_error_deltas"] == {"pe": 0, "ro": 0, "bo": 0, "te": 0}
    for group, key in (("speed", "invalid"), ("speed", "freeze"),
                       ("can", "pe"), ("can", "ro"),
                       ("can", "bo"), ("can", "te")):
        changed_speed = dict(speed_after)
        changed_can = dict(can_after)
        if group == "speed":
            changed_speed[key] += 1
        else:
            changed_can[key] += 1
        expect_error(
            lambda changed_speed=changed_speed, changed_can=changed_can:
            MODULE.validate_counter_growth(
                speed_before, changed_speed, can_before, changed_can
            ),
            AssertionError,
        )


def test_motion_sample_rejects_excess_iq_fault_and_wrong_sign():
    good = {"iqref": -500, "fault": 0, "measured": 1000, "active": 1}
    assert MODULE.validate_motion_sample(good, start_mdeg=0, delta_mdeg=1000) == good
    for sample in (
        dict(good, iqref=501),
        dict(good, fault=1),
        dict(good, measured=-1),
        dict(good, measured=0),
    ):
        expect_error(
            lambda sample=sample: MODULE.validate_motion_sample(
                sample, start_mdeg=0, delta_mdeg=1000
            ),
            AssertionError,
        )


def test_step_uses_fresh_relative_feedback_nonzero_session_and_two_points():
    events = []
    peer = FakePeer(events, final_position=1000)
    result = MODULE.run_phase(
        "step1", peer=peer, serial=FakeSerial(events), firmware_id="fw-a",
        session_factory=lambda: 0x2222,
    )
    assert result["start_mdeg"] == 0
    assert result["target_mdeg"] == 1000
    assert result["session"] == 0x2222
    assert events.index("arm") < events.index("submit") < events.index("sync")
    assert events.count("submit") == 2
    assert events.count("sync") == 2


def test_wrong_feedback_direction_aborts_and_cleans_up():
    events = []
    peer = FakePeer(events, final_position=-100)
    expect_error(
        lambda: MODULE.run_phase(
            "step1", peer=peer, serial=FakeSerial(events),
            firmware_id="fw-a", session_factory=lambda: 7,
        ),
        AssertionError,
        "direction",
    )
    assert events[-5:] == ["stop", "stop", "stop", "mc_stop", "pwm_safe"]


def test_evidence_is_atomic_and_only_written_after_every_cleanup_layer():
    with TemporaryDirectory(dir=ROOT) as directory:
        _test_evidence_order(Path(directory))


def _test_evidence_order(tmp_path):
    events = []

    class FakeDevice:
        def __init__(self, _path):
            events.append("device_load")

        def __enter__(self):
            events.append("device_open")
            return self

        def __exit__(self, *_args):
            events.append("device_close")

        def send(self, _frame):
            pass

        def receive(self, **_kwargs):
            return []

    peer = FakePeer(events)
    serial = FakeSerial(events)

    def replace(source, destination):
        source = Path(source)
        destination = Path(destination)
        assert source.exists()
        assert not destination.exists()
        assert events[-1] == "device_close"
        events.append("atomic_replace")
        os.replace(source, destination)

    evidence, path = MODULE.execute_phase(
        "discover",
        dll_path=tmp_path / "vendor.dll",
        port="COM9",
        evidence_dir=tmp_path,
        firmware_id="fw-a",
        device_factory=FakeDevice,
        peer_factory=lambda _device: peer,
        serial_factory=lambda _port: serial,
        replace=replace,
    )
    assert evidence["cleanup_complete"] is True
    assert path.exists()
    assert json.loads(path.read_text(encoding="utf-8"))["passed"] is True
    assert events[-8:] == [
        "stop", "stop", "stop", "mc_stop", "pwm_safe", "serial_close",
        "device_close", "atomic_replace",
    ]
    assert not list(tmp_path.glob("*.tmp"))


def test_failure_evidence_is_written_after_cleanup_without_hiding_error():
    with TemporaryDirectory(dir=ROOT) as directory:
        tmp_path = Path(directory)
        events = []

        class FakeDevice:
            def __init__(self, _path):
                pass

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                events.append("device_close")

            def send(self, _frame):
                pass

            def receive(self, **_kwargs):
                return []

        peer = FakePeer(events, fail_after_arm=True)
        serial = FakeSerial(events)

        error = expect_error(
            lambda: MODULE.execute_phase(
                "step1",
                dll_path=tmp_path / "vendor.dll",
                evidence_dir=tmp_path,
                firmware_id="fw-a",
                device_factory=FakeDevice,
                peer_factory=lambda _device: peer,
                serial_factory=lambda _port: serial,
            ),
            RuntimeError,
            "injected after ARM",
        )
        evidence_files = list(tmp_path.glob("*.json"))
        assert len(evidence_files) == 1
        evidence = json.loads(evidence_files[0].read_text(encoding="utf-8"))
        assert evidence["passed"] is False
        assert evidence["cleanup_complete"] is True
        assert evidence["error"]["type"] == "RuntimeError"
        assert events[-1] == "device_close"
        assert "failure evidence" in " ".join(getattr(error, "__notes__", ()))


if __name__ == "__main__":
    test_cli_exposes_only_explicit_phases_and_required_dll()
    test_probe_resolves_exports_without_opening_device()
    test_discover_phase_cannot_arm_or_send_trajectory()
    test_discover_rejects_can_error_counter_growth()
    test_step_target_is_relative_and_workspace_checked()
    test_failure_still_runs_all_cleanup_layers()
    test_health_requires_ready_session_zero_fault_free_and_6_to_30_vbus()
    test_com9_preflight_requires_complete_valid_config_and_ten_degree_margin()
    test_step5_requires_passing_step1_for_same_firmware_and_config()
    test_encoder_and_can_error_counters_may_not_grow()
    test_motion_sample_rejects_excess_iq_fault_and_wrong_sign()
    test_step_uses_fresh_relative_feedback_nonzero_session_and_two_points()
    test_wrong_feedback_direction_aborts_and_cleans_up()
    test_evidence_is_atomic_and_only_written_after_every_cleanup_layer()
    test_failure_evidence_is_written_after_cleanup_without_hiding_error()
    print("stage8 CANalyst Node 1 tests passed")
