"""Motor CAN v1 peer layered over an injected CAN device."""

import struct
from collections import deque
from dataclasses import dataclass

from canalyst_controlcan import CanFrame


PROTOCOL_VERSION = 1
CAN_ID_BROADCAST = 0x080
CAN_ID_TRAJECTORY = 0x101
CAN_ID_FEEDBACK = 0x181
CAN_ID_HEALTH = 0x281

OPCODE_ARM = 0x01
OPCODE_SYNC = 0x02
OPCODE_STOP = 0x03
OPCODE_CLEAR_FAULT = 0x04
OPCODE_DISCOVER = 0x05

NODE_STATE_UNCONFIGURED = 0
NODE_STATE_READY = 1
NODE_STATE_ARMED = 2
NODE_STATE_RUNNING = 3
NODE_STATE_HOLD = 4
NODE_STATE_FAULT = 5

_VALID_OPCODES = frozenset(range(OPCODE_ARM, OPCODE_DISCOVER + 1))
_VALID_STATES = frozenset(range(NODE_STATE_UNCONFIGURED, NODE_STATE_FAULT + 1))
_RECEIVE_BATCH = 64


@dataclass(frozen=True, slots=True)
class Health:
    protocol_version: int
    node_state: int
    fault_bits: int
    session: int
    vbus_10mv: int


@dataclass(frozen=True, slots=True)
class Feedback:
    actual_position_mdeg: int
    actual_velocity_mdeg_s: int
    applied_sequence: int


def _integer(value, name, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an integer")
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} outside {minimum}..{maximum}")
    return value


def _uint16(value, name):
    return _integer(value, name, 0, 0xFFFF)


def _sequence_newer(candidate, previous):
    difference = (candidate - previous) & 0xFFFF
    return difference != 0 and difference < 0x8000


def crc8(data):
    if not isinstance(data, bytes):
        raise TypeError("CRC input must be bytes")
    result = 0
    for value in data:
        result ^= value
        for _bit in range(8):
            result = ((result << 1) ^ 0x07) & 0xFF if result & 0x80 else (result << 1) & 0xFF
    return result


def encode_broadcast(opcode, sequence, session):
    opcode = _integer(opcode, "opcode", OPCODE_ARM, OPCODE_DISCOVER)
    if opcode not in _VALID_OPCODES:
        raise ValueError("unknown Motor CAN opcode")
    sequence = _uint16(sequence, "sequence")
    session = _uint16(session, "session")
    body = struct.pack("<BBHHB", opcode, PROTOCOL_VERSION, sequence, session, 0)
    return CanFrame(CAN_ID_BROADCAST, body + bytes((crc8(body),)))


def encode_trajectory(position_mdeg, velocity_mdeg_s, sequence):
    position_mdeg = _integer(
        position_mdeg, "position_mdeg", -0x80000000, 0x7FFFFFFF
    )
    velocity_mdeg_s = _integer(
        velocity_mdeg_s, "velocity_mdeg_s", -327680, 327670
    )
    if velocity_mdeg_s % 10:
        raise ValueError("velocity_mdeg_s must be exactly representable in 10 mdeg/s")
    sequence = _uint16(sequence, "sequence")
    return CanFrame(
        CAN_ID_TRAJECTORY,
        struct.pack("<i h H", position_mdeg, velocity_mdeg_s // 10, sequence),
    )


def _require_frame(frame, can_id, name):
    if not isinstance(frame, CanFrame):
        raise TypeError(f"{name} must be a CanFrame")
    if frame.can_id != can_id:
        raise ValueError(f"{name} has wrong CAN identifier")
    if len(frame.data) != 8:
        raise ValueError(f"{name} must have DLC 8")


def decode_feedback(frame):
    _require_frame(frame, CAN_ID_FEEDBACK, "feedback")
    position_mdeg, velocity_wire, sequence = struct.unpack("<i h H", frame.data)
    return Feedback(position_mdeg, velocity_wire * 10, sequence)


def decode_health(frame):
    _require_frame(frame, CAN_ID_HEALTH, "health")
    version, state, fault_bits, session, vbus_10mv = struct.unpack(
        "<BBHHH", frame.data
    )
    if version != PROTOCOL_VERSION:
        raise ValueError("unsupported Motor CAN protocol version")
    if state not in _VALID_STATES:
        raise ValueError("unknown Motor CAN node state")
    return Health(version, state, fault_bits, session, vbus_10mv)


class CanalystMotorPeer:
    def __init__(self, device, node_id=1):
        if isinstance(node_id, bool) or not isinstance(node_id, int):
            raise TypeError("node_id must be an integer")
        if node_id != 1:
            raise ValueError("the Stage 8 peer accepts only Motor CAN Node 1")
        if not callable(getattr(device, "send", None)) or not callable(
            getattr(device, "receive", None)
        ):
            raise TypeError("device must provide send() and receive()")
        self.device = device
        self.node_id = node_id
        self._armed = False
        self._session = None
        self._arm_sequence = None
        self._last_submitted_sequence = None
        self._pending_sequence = None
        self._last_feedback_sequence = None
        self._inbox = deque()

    def discover(self):
        self.device.send(encode_broadcast(OPCODE_DISCOVER, 0, 0))

    def arm(self, session, sequence=0):
        session = _uint16(session, "session")
        sequence = _uint16(sequence, "sequence")
        self.device.send(encode_broadcast(OPCODE_ARM, sequence, session))
        self._armed = True
        self._session = session
        self._arm_sequence = sequence
        self._last_submitted_sequence = None
        self._pending_sequence = None
        self._last_feedback_sequence = None

    def submit(self, position_mdeg, velocity_mdeg_s, sequence):
        if not self._armed:
            raise RuntimeError("trajectory preload requires successful ARM")
        if self._pending_sequence is not None:
            raise RuntimeError("matching SYNC is required before another preload")
        frame = encode_trajectory(position_mdeg, velocity_mdeg_s, sequence)
        sequence = _uint16(sequence, "sequence")
        if self._last_submitted_sequence is None:
            if self._arm_sequence is not None and sequence != self._arm_sequence:
                raise ValueError("first trajectory sequence does not match ARM")
        elif not _sequence_newer(sequence, self._last_submitted_sequence):
            raise ValueError("stale trajectory sequence")
        self.device.send(frame)
        self._last_submitted_sequence = sequence
        self._pending_sequence = sequence

    def sync(self, session, sequence):
        if not self._armed:
            raise RuntimeError("SYNC requires successful ARM")
        session = _uint16(session, "session")
        sequence = _uint16(sequence, "sequence")
        if self._session is not None and session != self._session:
            raise ValueError("stale Motor CAN session")
        if self._pending_sequence != sequence:
            raise RuntimeError("SYNC requires a matching successful preload")
        self.device.send(encode_broadcast(OPCODE_SYNC, sequence, session))
        self._pending_sequence = None

    def apply_point(self, session, sequence, position_mdeg, velocity_mdeg_s):
        self.submit(position_mdeg, velocity_mdeg_s, sequence)
        self.sync(session, sequence)

    def stop(self, session=0, sequence=0):
        self.device.send(encode_broadcast(OPCODE_STOP, sequence, session))
        self._armed = False
        self._pending_sequence = None

    def _take_cached(self, can_id):
        kept = deque()
        match = None
        while self._inbox:
            frame = self._inbox.popleft()
            if match is None and frame.can_id == can_id:
                match = frame
            else:
                kept.append(frame)
        self._inbox = kept
        return match

    def _read(self, can_id, decoder, acceptable):
        while True:
            frame = self._take_cached(can_id)
            if frame is None:
                break
            try:
                value = decoder(frame)
            except (TypeError, ValueError):
                continue
            if acceptable(value):
                return value

        result = None
        frames = self.device.receive(max_frames=_RECEIVE_BATCH, wait_ms=0)
        for frame in frames:
            if not isinstance(frame, CanFrame):
                continue
            if frame.can_id != can_id:
                if frame.can_id in (CAN_ID_FEEDBACK, CAN_ID_HEALTH):
                    self._inbox.append(frame)
                continue
            try:
                value = decoder(frame)
            except (TypeError, ValueError):
                continue
            if result is None:
                if acceptable(value):
                    result = value
            else:
                self._inbox.append(frame)
        return result

    def read_health(self):
        return self._read(
            CAN_ID_HEALTH,
            decode_health,
            lambda health: self._session is None or health.session == self._session,
        )

    def read_feedback(self):
        def acceptable(feedback):
            previous = self._last_feedback_sequence
            if previous is not None and feedback.applied_sequence != previous and not _sequence_newer(
                feedback.applied_sequence, previous
            ):
                return False
            self._last_feedback_sequence = feedback.applied_sequence
            return True

        return self._read(CAN_ID_FEEDBACK, decode_feedback, acceptable)
