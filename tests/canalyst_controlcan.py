import ctypes
from dataclasses import dataclass
from pathlib import Path


DWORD = ctypes.c_uint32
UINT = ctypes.c_uint32
ULONG = ctypes.c_uint32
INT = ctypes.c_int32
USHORT = ctypes.c_uint16
BYTE = ctypes.c_uint8

STATUS_OK = 1
MAX_RECEIVE_FRAMES = 4096


class VCI_BOARD_INFO(ctypes.Structure):
    _fields_ = [
        ("hw_Version", USHORT),
        ("fw_Version", USHORT),
        ("dr_Version", USHORT),
        ("in_Version", USHORT),
        ("irq_Num", USHORT),
        ("can_Num", BYTE),
        ("str_Serial_Num", ctypes.c_char * 20),
        ("str_hw_Type", ctypes.c_char * 40),
        ("Reserved", USHORT * 4),
    ]


class VCI_CAN_OBJ(ctypes.Structure):
    _fields_ = [
        ("ID", UINT),
        ("TimeStamp", UINT),
        ("TimeFlag", BYTE),
        ("SendType", BYTE),
        ("RemoteFlag", BYTE),
        ("ExternFlag", BYTE),
        ("DataLen", BYTE),
        ("Data", BYTE * 8),
        ("Reserved", BYTE * 3),
    ]


class VCI_INIT_CONFIG(ctypes.Structure):
    _fields_ = [
        ("AccCode", DWORD),
        ("AccMask", DWORD),
        ("Reserved", DWORD),
        ("Filter", BYTE),
        ("Timing0", BYTE),
        ("Timing1", BYTE),
        ("Mode", BYTE),
    ]


class ControlCanError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class CanFrame:
    can_id: int
    data: bytes
    timestamp: int = 0

    def __post_init__(self):
        if not isinstance(self.can_id, int) or isinstance(self.can_id, bool):
            raise TypeError("CAN identifier must be an integer")
        if not 0 <= self.can_id <= 0x7FF:
            raise ValueError("CAN identifier must be an 11-bit standard ID")
        if not isinstance(self.data, bytes):
            raise TypeError("CAN data must be bytes")
        if len(self.data) > 8:
            raise ValueError("classic CAN payload cannot exceed 8 bytes")


class ControlCanDevice:
    def __init__(
        self,
        dll_path,
        device_type=4,
        device_index=0,
        can_index=0,
        *,
        library=None,
    ):
        self.dll_path = Path(dll_path)
        self.device_type = device_type
        self.device_index = device_index
        self.can_index = can_index
        self.library = (
            library
            if library is not None
            else ctypes.WinDLL(str(self.dll_path))
        )
        self._is_open = False
        self._declare_signatures()

    def _declare_signatures(self):
        triple = [DWORD, DWORD, DWORD]
        signatures = {
            "VCI_OpenDevice": (triple, DWORD),
            "VCI_CloseDevice": ([DWORD, DWORD], DWORD),
            "VCI_InitCAN": (
                triple + [ctypes.POINTER(VCI_INIT_CONFIG)],
                DWORD,
            ),
            "VCI_ReadBoardInfo": (
                [DWORD, DWORD, ctypes.POINTER(VCI_BOARD_INFO)],
                DWORD,
            ),
            "VCI_ClearBuffer": (triple, DWORD),
            "VCI_StartCAN": (triple, DWORD),
            "VCI_ResetCAN": (triple, DWORD),
            "VCI_Transmit": (
                triple + [ctypes.POINTER(VCI_CAN_OBJ), ULONG],
                ULONG,
            ),
            "VCI_Receive": (
                triple
                + [ctypes.POINTER(VCI_CAN_OBJ), ULONG, INT],
                ULONG,
            ),
        }
        for name, (argtypes, restype) in signatures.items():
            function = getattr(self.library, name)
            function.argtypes = argtypes
            function.restype = restype

    def _channel_args(self):
        return self.device_type, self.device_index, self.can_index

    @staticmethod
    def _require_ok(operation, result):
        if result != STATUS_OK:
            raise ControlCanError(f"{operation} returned {result}, expected 1")

    def _require_open(self):
        if not self._is_open:
            raise ControlCanError("ControlCAN device is not open")

    def open(self):
        if self._is_open:
            return self
        result = self.library.VCI_OpenDevice(
            self.device_type, self.device_index, 0
        )
        self._require_ok("VCI_OpenDevice", result)
        self._is_open = True
        try:
            config = VCI_INIT_CONFIG(
                AccCode=0,
                AccMask=0xFFFFFFFF,
                Reserved=0,
                Filter=1,
                Timing0=0x00,
                Timing1=0x14,
                Mode=0,
            )
            result = self.library.VCI_InitCAN(
                *self._channel_args(), ctypes.byref(config)
            )
            self._require_ok("VCI_InitCAN", result)
            result = self.library.VCI_ClearBuffer(*self._channel_args())
            self._require_ok("VCI_ClearBuffer", result)
            result = self.library.VCI_StartCAN(*self._channel_args())
            self._require_ok("VCI_StartCAN", result)
        except BaseException as primary_error:
            try:
                self.close()
            except ControlCanError as cleanup_error:
                primary_error.add_note(f"ControlCAN cleanup failed: {cleanup_error}")
            raise
        return self

    def send(self, frame):
        self._require_open()
        if not isinstance(frame, CanFrame):
            raise TypeError("frame must be a CanFrame")
        vendor_frame = VCI_CAN_OBJ(
            ID=frame.can_id,
            TimeStamp=frame.timestamp,
            TimeFlag=0,
            SendType=0,
            RemoteFlag=0,
            ExternFlag=0,
            DataLen=len(frame.data),
        )
        for index, byte in enumerate(frame.data):
            vendor_frame.Data[index] = byte
        result = self.library.VCI_Transmit(
            *self._channel_args(), ctypes.byref(vendor_frame), 1
        )
        if result != 1:
            raise ControlCanError(
                f"VCI_Transmit returned {result}, expected 1 frame"
            )

    def receive(self, max_frames=64, wait_ms=0):
        self._require_open()
        if (
            not isinstance(max_frames, int)
            or isinstance(max_frames, bool)
            or not 1 <= max_frames <= MAX_RECEIVE_FRAMES
        ):
            raise ValueError(
                f"max_frames must be between 1 and {MAX_RECEIVE_FRAMES}"
            )
        if (
            not isinstance(wait_ms, int)
            or isinstance(wait_ms, bool)
            or not 0 <= wait_ms <= 0x7FFFFFFF
        ):
            raise ValueError("wait_ms must fit a nonnegative Windows INT")
        vendor_frames = (VCI_CAN_OBJ * max_frames)()
        result = self.library.VCI_Receive(
            *self._channel_args(), vendor_frames, max_frames, wait_ms
        )
        if result > max_frames:
            raise ControlCanError(
                f"VCI_Receive returned invalid frame count {result}"
            )
        frames = []
        for vendor_frame in vendor_frames[:result]:
            if vendor_frame.ExternFlag or vendor_frame.RemoteFlag:
                continue
            if vendor_frame.ID > 0x7FF:
                continue
            if vendor_frame.DataLen > 8:
                raise ControlCanError(
                    f"VCI_Receive returned invalid DLC {vendor_frame.DataLen}"
                )
            frames.append(
                CanFrame(
                    vendor_frame.ID,
                    bytes(vendor_frame.Data[:vendor_frame.DataLen]),
                    timestamp=vendor_frame.TimeStamp,
                )
            )
        return frames

    def reset(self):
        if not self._is_open:
            return
        result = self.library.VCI_ResetCAN(*self._channel_args())
        self._require_ok("VCI_ResetCAN", result)

    def close(self):
        if not self._is_open:
            return
        self._is_open = False
        errors = []
        try:
            result = self.library.VCI_ResetCAN(*self._channel_args())
            if result != STATUS_OK:
                errors.append(f"VCI_ResetCAN returned {result}")
        except Exception as error:
            errors.append(f"VCI_ResetCAN raised {error}")
        try:
            result = self.library.VCI_CloseDevice(
                self.device_type, self.device_index
            )
            if result != STATUS_OK:
                errors.append(f"VCI_CloseDevice returned {result}")
        except Exception as error:
            errors.append(f"VCI_CloseDevice raised {error}")
        if errors:
            raise ControlCanError("; ".join(errors))

    def __enter__(self):
        return self.open()

    def __exit__(self, _error_type, error, _traceback):
        try:
            self.close()
        except ControlCanError as cleanup_error:
            if error is None:
                raise
            error.add_note(f"ControlCAN cleanup failed: {cleanup_error}")
        return False
