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


DWORD = ctypes.c_uint32
UINT = ctypes.c_uint32
ULONG = ctypes.c_uint32
INT = ctypes.c_int32
USHORT = ctypes.c_uint16
BYTE = ctypes.c_uint8


def assert_structure(structure, fields, offsets, size):
    assert structure._fields_ == fields
    assert {
        name: getattr(structure, name).offset for name, _field_type in fields
    } == offsets
    assert ctypes.sizeof(structure) == size


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
        board_info_result=1,
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
        self.board_info_result = board_info_result

        triple = [DWORD, DWORD, DWORD]
        self.VCI_OpenDevice = FakeFunction(
            triple, DWORD, self._open
        )
        self.VCI_CloseDevice = FakeFunction(
            [DWORD, DWORD], DWORD, self._close
        )
        self.VCI_InitCAN = FakeFunction(
            triple + [ctypes.POINTER(MODULE.VCI_INIT_CONFIG)],
            DWORD,
            self._init,
        )
        self.VCI_ReadBoardInfo = FakeFunction(
            [DWORD, DWORD, ctypes.POINTER(MODULE.VCI_BOARD_INFO)],
            DWORD,
            self._board_info,
        )
        self.VCI_ClearBuffer = FakeFunction(
            triple, DWORD, self._clear
        )
        self.VCI_StartCAN = FakeFunction(
            triple, DWORD, self._start
        )
        self.VCI_ResetCAN = FakeFunction(
            triple, DWORD, self._reset
        )
        self.VCI_Transmit = FakeFunction(
            triple + [ctypes.POINTER(MODULE.VCI_CAN_OBJ), ULONG],
            ULONG,
            self._transmit,
        )
        self.VCI_Receive = FakeFunction(
            triple
            + [
                ctypes.POINTER(MODULE.VCI_CAN_OBJ),
                ULONG,
                INT,
            ],
            ULONG,
            self._receive,
        )

    @staticmethod
    def _result_or_raise(result):
        if isinstance(result, BaseException):
            raise result
        return result

    def _open(self, device_type, device_index, reserved):
        self.calls.append(("open", device_type, device_index, reserved))
        return self._result_or_raise(self.open_result)

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
        return self._result_or_raise(self.init_result)

    def _board_info(self, device_type, device_index, _board_pointer):
        self.calls.append(("board_info", device_type, device_index))
        return self._result_or_raise(self.board_info_result)

    def _clear(self, device_type, device_index, can_index):
        self.calls.append(("clear", device_type, device_index, can_index))
        return self._result_or_raise(self.clear_result)

    def _start(self, device_type, device_index, can_index):
        self.calls.append(("start", device_type, device_index, can_index))
        return self._result_or_raise(self.start_result)

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
        return self._result_or_raise(self.transmit_result)

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
        result = self._result_or_raise(
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
        return self._result_or_raise(self.reset_result)

    def _close(self, device_type, device_index):
        self.calls.append(("close", device_type, device_index))
        return self._result_or_raise(self.close_result)


def expect_error(callback, error_type=(TypeError, ValueError)):
    try:
        callback()
    except error_type as caught:
        return caught
    else:
        raise AssertionError("expected rejection")


def test_vendor_structures_match_the_independent_controlcan_abi():
    assert MODULE.DWORD is DWORD
    assert MODULE.UINT is UINT
    assert MODULE.ULONG is ULONG
    assert MODULE.INT is INT
    assert MODULE.USHORT is USHORT
    assert MODULE.BYTE is BYTE
    assert_structure(
        MODULE.VCI_BOARD_INFO,
        [
            ("hw_Version", USHORT),
            ("fw_Version", USHORT),
            ("dr_Version", USHORT),
            ("in_Version", USHORT),
            ("irq_Num", USHORT),
            ("can_Num", BYTE),
            ("str_Serial_Num", ctypes.c_char * 20),
            ("str_hw_Type", ctypes.c_char * 40),
            ("Reserved", USHORT * 4),
        ],
        {
            "hw_Version": 0,
            "fw_Version": 2,
            "dr_Version": 4,
            "in_Version": 6,
            "irq_Num": 8,
            "can_Num": 10,
            "str_Serial_Num": 11,
            "str_hw_Type": 31,
            "Reserved": 72,
        },
        80,
    )
    assert_structure(
        MODULE.VCI_CAN_OBJ,
        [
            ("ID", UINT),
            ("TimeStamp", UINT),
            ("TimeFlag", BYTE),
            ("SendType", BYTE),
            ("RemoteFlag", BYTE),
            ("ExternFlag", BYTE),
            ("DataLen", BYTE),
            ("Data", BYTE * 8),
            ("Reserved", BYTE * 3),
        ],
        {
            "ID": 0,
            "TimeStamp": 4,
            "TimeFlag": 8,
            "SendType": 9,
            "RemoteFlag": 10,
            "ExternFlag": 11,
            "DataLen": 12,
            "Data": 13,
            "Reserved": 21,
        },
        24,
    )
    assert_structure(
        MODULE.VCI_INIT_CONFIG,
        [
            ("AccCode", DWORD),
            ("AccMask", DWORD),
            ("Reserved", DWORD),
            ("Filter", BYTE),
            ("Timing0", BYTE),
            ("Timing1", BYTE),
            ("Mode", BYTE),
        ],
        {
            "AccCode": 0,
            "AccMask": 4,
            "Reserved": 8,
            "Filter": 12,
            "Timing0": 13,
            "Timing1": 14,
            "Mode": 15,
        },
        16,
    )


def test_confirmed_channel_configuration_and_structure_layout():
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
    triple = [DWORD, DWORD, DWORD]
    expected = {
        "VCI_OpenDevice": (triple, DWORD),
        "VCI_CloseDevice": ([DWORD, DWORD], DWORD),
        "VCI_InitCAN": (
            triple + [ctypes.POINTER(MODULE.VCI_INIT_CONFIG)],
            DWORD,
        ),
        "VCI_ReadBoardInfo": (
            [DWORD, DWORD, ctypes.POINTER(MODULE.VCI_BOARD_INFO)],
            DWORD,
        ),
        "VCI_ClearBuffer": (triple, DWORD),
        "VCI_StartCAN": (triple, DWORD),
        "VCI_ResetCAN": (triple, DWORD),
        "VCI_Transmit": (
            triple + [ctypes.POINTER(MODULE.VCI_CAN_OBJ), ULONG],
            ULONG,
        ),
        "VCI_Receive": (
            triple + [ctypes.POINTER(MODULE.VCI_CAN_OBJ), ULONG, INT],
            ULONG,
        ),
    }
    for name, (argtypes, restype) in expected.items():
        function = getattr(fake, name)
        assert function.argtypes == argtypes
        assert function.restype is restype
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


def test_non_ok_status_errors_name_operation_actual_and_expected_values():
    for option, operation in (
        ("open_result", "VCI_OpenDevice"),
        ("init_result", "VCI_InitCAN"),
        ("clear_result", "VCI_ClearBuffer"),
        ("start_result", "VCI_StartCAN"),
    ):
        error = expect_error(
            lambda option=option: MODULE.ControlCanDevice(
                "unused", library=FakeLibrary(**{option: 2})
            ).open(),
            MODULE.ControlCanError,
        )
        assert str(error) == f"{operation} returned 2, expected 1"

    fake = FakeLibrary(transmit_result=2)
    device = MODULE.ControlCanDevice("unused", library=fake).open()
    error = expect_error(
        lambda: device.send(MODULE.CanFrame(0x080, b"stop")),
        MODULE.ControlCanError,
    )
    assert str(error) == "VCI_Transmit returned 2, expected 1 frame"
    device.close()


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
    calls_after_first_close = list(fake.calls)
    device.close()
    device.close()
    assert fake.calls == calls_after_first_close
    assert calls_after_first_close[-2:] == [
        ("reset", 4, 0, 0),
        ("close", 4, 0),
    ]


def test_close_attempts_close_after_reset_exception_and_keeps_both_errors():
    reset_error = RuntimeError("reset sentinel")
    close_error = OSError("close sentinel")
    fake = FakeLibrary(
        reset_result=reset_error,
        close_result=close_error,
    )
    device = MODULE.ControlCanDevice("unused", library=fake).open()
    error = expect_error(device.close, MODULE.ControlCanError)
    assert str(error) == (
        "VCI_ResetCAN raised reset sentinel; "
        "VCI_CloseDevice raised close sentinel"
    )
    assert fake.calls[-2:] == [("reset", 4, 0, 0), ("close", 4, 0)]


def test_context_preserves_body_exception_and_complete_cleanup_note():
    primary = ValueError("body sentinel")
    fake = FakeLibrary(
        reset_result=RuntimeError("reset sentinel"),
        close_result=OSError("close sentinel"),
    )
    try:
        with MODULE.ControlCanDevice("unused", library=fake):
            raise primary
    except ValueError as caught:
        assert caught is primary
        assert getattr(caught, "__notes__", ()) == [
            "ControlCAN cleanup failed: "
            "VCI_ResetCAN raised reset sentinel; "
            "VCI_CloseDevice raised close sentinel"
        ]
    else:
        raise AssertionError("body exception was swallowed")
    assert fake.calls[-2:] == [("reset", 4, 0, 0), ("close", 4, 0)]


def test_context_cleanup_runs_after_transmit_failure():
    fake = FakeLibrary(transmit_result=0)
    try:
        with MODULE.ControlCanDevice("unused", library=fake) as device:
            device.send(MODULE.CanFrame(0x080, b"12345678"))
    except MODULE.ControlCanError as caught:
        assert str(caught) == "VCI_Transmit returned 0, expected 1 frame"
    else:
        raise AssertionError("transmit failure was swallowed")
    assert fake.calls[-2:] == [("reset", 4, 0, 0), ("close", 4, 0)]


if __name__ == "__main__":
    test_vendor_structures_match_the_independent_controlcan_abi()
    test_confirmed_channel_configuration_and_structure_layout()
    test_every_vendor_function_has_the_exact_windows_signature()
    test_invalid_standard_identifier_and_dlc_are_rejected()
    test_send_populates_a_standard_data_frame_and_rejects_short_write()
    test_receive_filters_nonstandard_frames_and_checks_vendor_bounds()
    test_receive_rejects_malformed_vendor_frames()
    test_open_stage_failures_cleanup_only_after_device_opened()
    test_non_ok_status_errors_name_operation_actual_and_expected_values()
    test_close_is_idempotent_and_aggregates_reset_and_close_failures()
    test_close_attempts_close_after_reset_exception_and_keeps_both_errors()
    test_context_preserves_body_exception_and_complete_cleanup_note()
    test_context_cleanup_runs_after_transmit_failure()
    print("CANalyst ControlCAN tests passed")
