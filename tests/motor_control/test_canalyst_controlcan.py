import ctypes
import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "tests" / "canalyst_controlcan.py"
SPEC = importlib.util.spec_from_file_location(
    "canalyst_controlcan_under_test", MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class FakeFunction:
    def __init__(self, expected_argtypes, restype, callback):
        self.expected_argtypes = expected_argtypes
        self.expected_restype = restype
        self.callback = callback
        self.argtypes = None
        self.restype = None
        self.calls = []

    def __call__(self, *args):
        assert self.argtypes == self.expected_argtypes
        assert self.restype is self.expected_restype
        self.calls.append(args)
        return self.callback(*args)


class FakeLibrary:
    def __init__(
        self,
        *,
        open_result=1,
        init_result=1,
        clear_result=1,
        start_result=1,
        transmit_result=1,
        receive_frames=(),
        receive_result=None,
        reset_result=1,
        close_result=1,
    ):
        self.calls = []
        self.init_config = None
        self.transmitted = []
        self.open_result = open_result
        self.init_result = init_result
        self.clear_result = clear_result
        self.start_result = start_result
        self.transmit_result = transmit_result
        self.receive_frames = list(receive_frames)
        self.receive_result = receive_result
        self.reset_result = reset_result
        self.close_result = close_result

        triple = [MODULE.DWORD, MODULE.DWORD, MODULE.DWORD]
        self.VCI_OpenDevice = FakeFunction(
            triple, MODULE.DWORD, self._open
        )
        self.VCI_CloseDevice = FakeFunction(
            [MODULE.DWORD, MODULE.DWORD], MODULE.DWORD, self._close
        )
        self.VCI_InitCAN = FakeFunction(
            triple + [ctypes.POINTER(MODULE.VCI_INIT_CONFIG)],
            MODULE.DWORD,
            self._init,
        )
        self.VCI_ReadBoardInfo = FakeFunction(
            [
                MODULE.DWORD,
                MODULE.DWORD,
                ctypes.POINTER(MODULE.VCI_BOARD_INFO),
            ],
            MODULE.DWORD,
            lambda *_args: 1,
        )
        self.VCI_ClearBuffer = FakeFunction(
            triple, MODULE.DWORD, self._clear
        )
        self.VCI_StartCAN = FakeFunction(
            triple, MODULE.DWORD, self._start
        )
        self.VCI_ResetCAN = FakeFunction(
            triple, MODULE.DWORD, self._reset
        )
        self.VCI_Transmit = FakeFunction(
            triple
            + [ctypes.POINTER(MODULE.VCI_CAN_OBJ), MODULE.ULONG],
            MODULE.ULONG,
            self._transmit,
        )
        self.VCI_Receive = FakeFunction(
            triple
            + [
                ctypes.POINTER(MODULE.VCI_CAN_OBJ),
                MODULE.ULONG,
                MODULE.INT,
            ],
            MODULE.ULONG,
            self._receive,
        )

    def _open(self, device_type, device_index, reserved):
        self.calls.append(("open", device_type, device_index, reserved))
        return self.open_result

    def _init(self, device_type, device_index, can_index, config_pointer):
        config = ctypes.cast(
            config_pointer, ctypes.POINTER(MODULE.VCI_INIT_CONFIG)
        ).contents
        self.init_config = {
            "AccCode": config.AccCode,
            "AccMask": config.AccMask,
            "Filter": config.Filter,
            "Timing0": config.Timing0,
            "Timing1": config.Timing1,
            "Mode": config.Mode,
        }
        self.calls.append(("init", device_type, device_index, can_index))
        return self.init_result

    def _clear(self, device_type, device_index, can_index):
        self.calls.append(("clear", device_type, device_index, can_index))
        return self.clear_result

    def _start(self, device_type, device_index, can_index):
        self.calls.append(("start", device_type, device_index, can_index))
        return self.start_result

    def _transmit(
        self, device_type, device_index, can_index, frame_pointer, count
    ):
        frame = ctypes.cast(
            frame_pointer, ctypes.POINTER(MODULE.VCI_CAN_OBJ)
        ).contents
        self.calls.append(
            ("transmit", device_type, device_index, can_index, count)
        )
        self.transmitted.append(
            {
                "ID": frame.ID,
                "TimeStamp": frame.TimeStamp,
                "TimeFlag": frame.TimeFlag,
                "SendType": frame.SendType,
                "RemoteFlag": frame.RemoteFlag,
                "ExternFlag": frame.ExternFlag,
                "DataLen": frame.DataLen,
                "Data": bytes(frame.Data),
                "Reserved": bytes(frame.Reserved),
            }
        )
        return self.transmit_result

    def _receive(
        self, device_type, device_index, can_index, frames_pointer,
        max_frames, wait_ms
    ):
        self.calls.append(
            (
                "receive",
                device_type,
                device_index,
                can_index,
                max_frames,
                wait_ms,
            )
        )
        result = (
            len(self.receive_frames)
            if self.receive_result is None
            else self.receive_result
        )
        frames = ctypes.cast(
            frames_pointer, ctypes.POINTER(MODULE.VCI_CAN_OBJ)
        )
        for index, values in enumerate(self.receive_frames[:max_frames]):
            frame = frames[index]
            frame.ID = values.get("ID", 0)
            frame.TimeStamp = values.get("TimeStamp", 0)
            frame.TimeFlag = values.get("TimeFlag", 0)
            frame.SendType = values.get("SendType", 0)
            frame.RemoteFlag = values.get("RemoteFlag", 0)
            frame.ExternFlag = values.get("ExternFlag", 0)
            data = values.get("Data", b"")
            frame.DataLen = values.get("DataLen", len(data))
            for data_index, byte in enumerate(data[:8]):
                frame.Data[data_index] = byte
        return result

    def _reset(self, device_type, device_index, can_index):
        self.calls.append(("reset", device_type, device_index, can_index))
        return self.reset_result

    def _close(self, device_type, device_index):
        self.calls.append(("close", device_type, device_index))
        return self.close_result


def expect_error(callback, error_type=(TypeError, ValueError)):
    try:
        callback()
    except error_type:
        pass
    else:
        raise AssertionError("expected rejection")


def test_confirmed_channel_configuration_and_structure_layout():
    assert ctypes.sizeof(MODULE.VCI_CAN_OBJ) == 24
    assert ctypes.sizeof(MODULE.VCI_INIT_CONFIG) == 16
    device = MODULE.ControlCanDevice("unused", library=FakeLibrary())
    device.open()
    assert device.library.init_config == {
        "AccCode": 0,
        "AccMask": 0xFFFFFFFF,
        "Filter": 1,
        "Timing0": 0x00,
        "Timing1": 0x14,
        "Mode": 0,
    }
    assert device.library.calls[:4] == [
        ("open", 4, 0, 0),
        ("init", 4, 0, 0),
        ("clear", 4, 0, 0),
        ("start", 4, 0, 0),
    ]


def test_every_vendor_function_has_the_exact_windows_signature():
    fake = FakeLibrary(receive_frames=[{"ID": 0x181, "Data": b"x"}])
    device = MODULE.ControlCanDevice("unused", library=fake).open()
    device.send(MODULE.CanFrame(0x080, b"12345678"))
    assert device.receive(max_frames=1, wait_ms=7) == [
        MODULE.CanFrame(0x181, b"x")
    ]
    board = MODULE.VCI_BOARD_INFO()
    assert fake.VCI_ReadBoardInfo(4, 0, ctypes.byref(board)) == 1
    assert len(fake.VCI_ReadBoardInfo.calls) == 1
    device.close()


def test_invalid_standard_identifier_and_dlc_are_rejected():
    for frame in (
        lambda: MODULE.CanFrame(-1, b""),
        lambda: MODULE.CanFrame(0x800, b""),
        lambda: MODULE.CanFrame(0x080, b"123456789"),
        lambda: MODULE.CanFrame(0x080, bytearray(b"1")),
    ):
        expect_error(frame)


def test_send_populates_a_standard_data_frame_and_rejects_short_write():
    fake = FakeLibrary()
    device = MODULE.ControlCanDevice("unused", library=fake).open()
    device.send(MODULE.CanFrame(0x101, b"\x01\x02\x03", timestamp=99))
    assert fake.transmitted[-1] == {
        "ID": 0x101,
        "TimeStamp": 99,
        "TimeFlag": 0,
        "SendType": 0,
        "RemoteFlag": 0,
        "ExternFlag": 0,
        "DataLen": 3,
        "Data": b"\x01\x02\x03\x00\x00\x00\x00\x00",
        "Reserved": b"\x00\x00\x00",
    }
    fake.transmit_result = 0
    expect_error(
        lambda: device.send(MODULE.CanFrame(0x080, b"stop")),
        MODULE.ControlCanError,
    )


def test_receive_filters_nonstandard_frames_and_checks_vendor_bounds():
    fake = FakeLibrary(
        receive_frames=[
            {"ID": 0x181, "TimeStamp": 123, "Data": b"abc"},
            {"ID": 0x1ABCDE, "ExternFlag": 1, "Data": b"ext"},
            {"ID": 0x281, "RemoteFlag": 1, "Data": b"remote"},
        ]
    )
    device = MODULE.ControlCanDevice("unused", library=fake).open()
    assert device.receive(max_frames=3, wait_ms=25) == [
        MODULE.CanFrame(0x181, b"abc", timestamp=123)
    ]
    assert fake.calls[-1] == ("receive", 4, 0, 0, 3, 25)
    for arguments in (
        {"max_frames": 0},
        {"max_frames": MODULE.MAX_RECEIVE_FRAMES + 1},
        {"wait_ms": -1},
        {"wait_ms": 0x80000000},
    ):
        expect_error(lambda arguments=arguments: device.receive(**arguments))
    fake.receive_result = 4
    expect_error(
        lambda: device.receive(max_frames=3), MODULE.ControlCanError
    )


def test_receive_rejects_malformed_vendor_frames():
    fake = FakeLibrary(
        receive_frames=[{"ID": 0x181, "DataLen": 9, "Data": b"12345678"}]
    )
    device = MODULE.ControlCanDevice("unused", library=fake).open()
    expect_error(
        lambda: device.receive(max_frames=1), MODULE.ControlCanError
    )


def test_open_stage_failures_cleanup_only_after_device_opened():
    cases = (
        (dict(open_result=0), [("open", 4, 0, 0)]),
        (
            dict(init_result=0),
            [
                ("open", 4, 0, 0),
                ("init", 4, 0, 0),
                ("reset", 4, 0, 0),
                ("close", 4, 0),
            ],
        ),
        (
            dict(clear_result=0),
            [
                ("open", 4, 0, 0),
                ("init", 4, 0, 0),
                ("clear", 4, 0, 0),
                ("reset", 4, 0, 0),
                ("close", 4, 0),
            ],
        ),
        (
            dict(start_result=0),
            [
                ("open", 4, 0, 0),
                ("init", 4, 0, 0),
                ("clear", 4, 0, 0),
                ("start", 4, 0, 0),
                ("reset", 4, 0, 0),
                ("close", 4, 0),
            ],
        ),
    )
    for options, expected_calls in cases:
        fake = FakeLibrary(**options)
        expect_error(
            lambda fake=fake: MODULE.ControlCanDevice(
                "unused", library=fake
            ).open(),
            MODULE.ControlCanError,
        )
        assert fake.calls == expected_calls


def test_close_is_idempotent_and_aggregates_reset_and_close_failures():
    fake = FakeLibrary(reset_result=0, close_result=0)
    device = MODULE.ControlCanDevice("unused", library=fake).open()
    try:
        device.close()
    except MODULE.ControlCanError as caught:
        message = str(caught).lower()
        assert "reset" in message
        assert "close" in message
    else:
        raise AssertionError("both cleanup failures were swallowed")
    device.close()
    assert fake.calls[-2:] == [("reset", 4, 0, 0), ("close", 4, 0)]


def test_context_cleanup_runs_after_transmit_failure():
    fake = FakeLibrary(transmit_result=0)
    try:
        with MODULE.ControlCanDevice("unused", library=fake) as device:
            device.send(MODULE.CanFrame(0x080, b"12345678"))
    except MODULE.ControlCanError:
        pass
    assert fake.calls[-2:] == [("reset", 4, 0, 0), ("close", 4, 0)]


if __name__ == "__main__":
    test_confirmed_channel_configuration_and_structure_layout()
    test_every_vendor_function_has_the_exact_windows_signature()
    test_invalid_standard_identifier_and_dlc_are_rejected()
    test_send_populates_a_standard_data_frame_and_rejects_short_write()
    test_receive_filters_nonstandard_frames_and_checks_vendor_bounds()
    test_receive_rejects_malformed_vendor_frames()
    test_open_stage_failures_cleanup_only_after_device_opened()
    test_close_is_idempotent_and_aggregates_reset_and_close_failures()
    test_context_cleanup_runs_after_transmit_failure()
    print("CANalyst ControlCAN tests passed")
