"""Gated CANalyst-II qualification launcher for Motor CAN Node 1.

All powered operations are injected behind the existing Stage 8 cleanup
runner.  Importing this module never loads a vendor DLL or opens hardware.
"""

import argparse
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import re
import secrets
import subprocess
import sys


TESTS_DIR = Path(__file__).resolve().parent
ROOT = TESTS_DIR.parent
if str(TESTS_DIR) not in sys.path:
    sys.path.insert(0, str(TESTS_DIR))

from canalyst_controlcan import ControlCanDevice
from canalyst_motor_peer import CanalystMotorPeer


def _load_sibling(name):
    path = TESTS_DIR / f"{name}.py"
    spec = importlib.util.spec_from_file_location(
        f"{name}_for_stage8_canalyst", path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_STAGE7 = _load_sibling("stage7_bench")
_STAGE8 = _load_sibling("stage8_can_bench")

read_position_status = _STAGE7.read_position_status
read_speed_status = _STAGE7._STAGE6.read_speed_status
parse_can_status = _STAGE8.parse_can_status
run_can_bench = _STAGE8.run_can_bench
send_cmd = _STAGE8.send_cmd

PORT = "COM9"
DEFAULT_EVIDENCE_DIR = Path("artifacts/stage8-canalyst")
MIN_VBUS_10MV = 600
MAX_VBUS_10MV = 3000
WORKSPACE_MARGIN_MDEG = 10000
POSITION_IQ_LIMIT_MA = 500

PHASE_DELTAS = {"step1": 1000, "step5": 5000}
STATE_NAMES = {
    0: "UNCONFIGURED",
    1: "READY",
    2: "ARMED",
    3: "RUNNING",
    4: "HOLD",
    5: "FAULT",
}

REQUIRED_CONTROLCAN_EXPORTS = (
    "VCI_OpenDevice",
    "VCI_CloseDevice",
    "VCI_InitCAN",
    "VCI_ReadBoardInfo",
    "VCI_ClearBuffer",
    "VCI_StartCAN",
    "VCI_ResetCAN",
    "VCI_Transmit",
    "VCI_Receive",
)

_CONFIG_FIELDS = (
    "record_present",
    "version",
    "generation",
    "node",
    "zero_raw",
    "known_mdeg",
    "min_mdeg",
    "max_mdeg",
    "direction",
    "crc_valid",
    "encoder_ready",
    "restored_joint_mdeg",
    "restored_joint_valid",
    "service_ready",
    "mutation_busy",
    "runtime_locked",
    "reboot_required",
)
_PERSISTED_CONFIG_FIELDS = (
    "version",
    "generation",
    "node",
    "zero_raw",
    "known_mdeg",
    "min_mdeg",
    "max_mdeg",
    "direction",
)
_CONFIG_LINE = re.compile(r"^([a-z_]+)\s*:\s*(-?[0-9]+)\s*$")


def build_parser():
    parser = argparse.ArgumentParser(
        description="Safely qualify Motor CAN Node 1 through CANalyst-II"
    )
    parser.add_argument(
        "--phase", required=True, choices=("probe", "discover", "step1", "step5")
    )
    parser.add_argument("--dll", type=Path, required=True)
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--evidence-dir", type=Path, default=DEFAULT_EVIDENCE_DIR)
    return parser


def _plain_int(value, name):
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an integer")
    return value


def relative_target(current_mdeg, delta_mdeg, minimum_mdeg, maximum_mdeg):
    current = _plain_int(current_mdeg, "current_mdeg")
    delta = _plain_int(delta_mdeg, "delta_mdeg")
    minimum = _plain_int(minimum_mdeg, "minimum_mdeg")
    maximum = _plain_int(maximum_mdeg, "maximum_mdeg")
    if minimum >= maximum:
        raise ValueError("joint workspace minimum must be below maximum")
    target = current + delta
    if not minimum <= current <= maximum:
        raise ValueError("fresh position is outside the persisted workspace")
    if not minimum <= target <= maximum:
        raise ValueError("relative target is outside the persisted workspace")
    return target


def validate_health(health):
    if health is None:
        raise AssertionError("no fresh Node 1 health frame")
    if health.protocol_version != 1:
        raise AssertionError("Node 1 protocol version is not 1")
    if health.node_state != 1:
        raise AssertionError("Node 1 is not READY")
    if health.fault_bits != 0:
        raise AssertionError("Node 1 health reports fault bits")
    if health.session != 0:
        raise AssertionError("Node 1 health session is not zero")
    if not MIN_VBUS_10MV <= health.vbus_10mv <= MAX_VBUS_10MV:
        raise AssertionError("Node 1 VBUS is outside 6-30 V")
    return {
        "protocol_version": health.protocol_version,
        "node_state": STATE_NAMES[health.node_state],
        "fault_bits": health.fault_bits,
        "session": health.session,
        "vbus_mv": health.vbus_10mv * 10,
    }


def parse_joint_config(text):
    if not isinstance(text, str):
        raise AssertionError("joint_cfg_show reply must be text")
    parsed = {}
    for line in text.splitlines():
        match = _CONFIG_LINE.fullmatch(line.strip())
        if match and match.group(1) in _CONFIG_FIELDS:
            key = match.group(1)
            if key in parsed:
                raise AssertionError(f"duplicate joint config field {key}")
            parsed[key] = int(match.group(2))
    missing = [field for field in _CONFIG_FIELDS if field not in parsed]
    if missing:
        raise AssertionError(f"incomplete joint_cfg_show reply: {', '.join(missing)}")
    return parsed


def read_joint_config(serial):
    return parse_joint_config(send_cmd(serial, "joint_cfg_show", timeout=1.0))


def config_fingerprint(config):
    if not isinstance(config, dict):
        raise TypeError("config must be a mapping")
    try:
        return {key: config[key] for key in _PERSISTED_CONFIG_FIELDS}
    except KeyError as error:
        raise ValueError(f"missing persisted config field {error.args[0]}") from error


def validate_preflight(config, current_mdeg):
    if not isinstance(config, dict):
        raise TypeError("config must be a mapping")
    missing = [field for field in _CONFIG_FIELDS if field not in config]
    if missing:
        raise AssertionError(f"incomplete joint config: {', '.join(missing)}")
    current = _plain_int(current_mdeg, "current_mdeg")
    for field in _CONFIG_FIELDS:
        _plain_int(config[field], field)
    required_one = (
        "record_present",
        "crc_valid",
        "encoder_ready",
        "restored_joint_valid",
        "service_ready",
    )
    if any(config[field] != 1 for field in required_one):
        raise AssertionError("joint configuration is not valid and ready")
    if config["mutation_busy"] != 0 or config["reboot_required"] != 0:
        raise AssertionError("joint configuration is mutating or requires reboot")
    if config["version"] != 1 or config["node"] != 1:
        raise AssertionError("joint configuration is not version 1 for Node 1")
    if config["direction"] not in (-1, 1):
        raise AssertionError("joint direction must be +1 or -1")
    minimum = config["min_mdeg"]
    maximum = config["max_mdeg"]
    if minimum >= maximum:
        raise AssertionError("joint workspace is invalid")
    if not minimum <= config["known_mdeg"] <= maximum:
        raise AssertionError("known joint position is outside its workspace")
    if not minimum <= current <= maximum:
        raise AssertionError("fresh joint position is outside its workspace")
    if (
        current - minimum < WORKSPACE_MARGIN_MDEG
        or maximum - current < WORKSPACE_MARGIN_MDEG
    ):
        raise AssertionError("fresh position lacks the required 10-degree margin")
    return dict(config)


def read_can_status(serial):
    reply = send_cmd(serial, "can_status", timeout=1.0)
    checked_lines = [
        line.strip() for line in reply.splitlines() if line.strip().startswith("cs ")
    ]
    parsed = (
        parse_can_status(checked_lines[0]) if len(checked_lines) == 1 else None
    )
    if parsed is None:
        raise AssertionError("no valid checked can_status reply")
    return parsed


def validate_counter_growth(speed_before, speed_after, can_before, can_after):
    encoder_deltas = {}
    for key in ("invalid", "freeze"):
        before = _plain_int(speed_before[key], f"speed_before.{key}")
        after = _plain_int(speed_after[key], f"speed_after.{key}")
        encoder_deltas[key] = after - before
        if encoder_deltas[key] != 0:
            raise AssertionError(f"encoder {key} counter grew")
    if speed_after.get("streak") != 0 or speed_after.get("fault") != 0:
        raise AssertionError("encoder sampling is not healthy")
    can_deltas = {}
    for key in ("pe", "ro", "bo", "te"):
        before = _plain_int(can_before[key], f"can_before.{key}")
        after = _plain_int(can_after[key], f"can_after.{key}")
        can_deltas[key] = after - before
        if can_deltas[key] != 0:
            raise AssertionError(f"CAN {key} error counter grew")
    if can_after.get("f", 0) != 0:
        raise AssertionError("CAN status reports fault bits")
    return {
        "encoder_deltas": encoder_deltas,
        "can_error_deltas": can_deltas,
        "can_rx_delta": can_after.get("rx", 0) - can_before.get("rx", 0),
        "can_tx_delta": can_after.get("tx", 0) - can_before.get("tx", 0),
    }


def validate_motion_sample(sample, start_mdeg, delta_mdeg):
    if sample is None:
        raise AssertionError("no fresh mc_pos_status sample")
    if sample.get("active") != 1:
        raise AssertionError("position loop is not active during motion")
    if sample.get("fault") != 0:
        raise AssertionError("position sample reports fault bits")
    iqref = _plain_int(sample.get("iqref"), "iqref")
    if abs(iqref) > POSITION_IQ_LIMIT_MA:
        raise AssertionError("abs(iqref) exceeds 500 mA")
    movement = _plain_int(sample.get("measured"), "measured") - start_mdeg
    if movement * delta_mdeg <= 0:
        raise AssertionError("observed movement has the wrong direction")
    return sample


def _validate_feedback(feedback, start_mdeg, delta_mdeg, sequence):
    if feedback is None:
        raise AssertionError("no fresh Node 1 feedback after relative point")
    if feedback.applied_sequence != sequence:
        raise AssertionError("Node 1 did not confirm the applied sequence")
    movement = feedback.actual_position_mdeg - start_mdeg
    if movement * delta_mdeg <= 0:
        raise AssertionError("CAN feedback movement has the wrong direction")
    return {
        "actual_position_mdeg": feedback.actual_position_mdeg,
        "actual_velocity_mdeg_s": feedback.actual_velocity_mdeg_s,
        "applied_sequence": feedback.applied_sequence,
    }


def _validate_final_position(sample):
    if sample is None or sample.get("active") != 0:
        raise AssertionError("motor did not return to DISABLED position state")
    if sample.get("fault") != 0:
        raise AssertionError("final position status reports a fault")
    return sample


def require_step1_evidence(evidence_dir, firmware_id, config):
    directory = Path(evidence_dir)
    expected_config = config_fingerprint(config)
    if directory.exists():
        for path in sorted(directory.glob("*.json"), reverse=True):
            try:
                evidence = json.loads(path.read_text(encoding="utf-8"))
                observed_config = config_fingerprint(evidence.get("config", {}))
            except (OSError, UnicodeError, json.JSONDecodeError, TypeError, ValueError):
                continue
            if (
                evidence.get("phase") == "step1"
                and evidence.get("passed") is True
                and evidence.get("cleanup_complete") is True
                and evidence.get("firmware_id") == firmware_id
                and observed_config == expected_config
            ):
                return path
    raise RuntimeError(
        "step5 requires passing step1 evidence for the same firmware/config"
    )


def _motion_qualification(
    phase, peer, serial, firmware_id, evidence_dir, session_factory
):
    delta = PHASE_DELTAS[phase]
    health = validate_health(peer.read_health())
    start_feedback = peer.read_feedback()
    if start_feedback is None:
        raise AssertionError("no fresh Node 1 feedback before motion")
    start = start_feedback.actual_position_mdeg
    config = validate_preflight(read_joint_config(serial), start)
    target = relative_target(start, delta, config["min_mdeg"], config["max_mdeg"])
    if phase == "step5":
        require_step1_evidence(evidence_dir, firmware_id, config)

    speed_before = read_speed_status(serial)
    can_before = read_can_status(serial)
    session = _plain_int(session_factory(), "session")
    if not 1 <= session <= 0xFFFF:
        raise ValueError("fresh session must be a nonzero uint16")
    first_sequence = (start_feedback.applied_sequence + 1) & 0xFFFF
    second_sequence = (first_sequence + 1) & 0xFFFF
    peer.arm(session, sequence=first_sequence)
    peer.apply_point(session, first_sequence, start, 0)
    peer.apply_point(session, second_sequence, target, 0)

    feedback = _validate_feedback(
        peer.read_feedback(), start, delta, second_sequence
    )
    position = validate_motion_sample(
        read_position_status(serial), start_mdeg=start, delta_mdeg=delta
    )
    speed_after = read_speed_status(serial)
    can_after = read_can_status(serial)
    counters = validate_counter_growth(
        speed_before, speed_after, can_before, can_after
    )
    peer.stop(session=session, sequence=second_sequence)
    final_position = _validate_final_position(read_position_status(serial))
    return {
        "phase": phase,
        "passed": True,
        "firmware_id": firmware_id,
        "config": config_fingerprint(config),
        "health": health,
        "node_state": health["node_state"],
        "start_mdeg": start,
        "target_mdeg": target,
        "delta_mdeg": delta,
        "session": session,
        "baseline_sequence": first_sequence,
        "applied_sequence": second_sequence,
        "feedback": feedback,
        "position": position,
        "peak_abs_iqref_ma": abs(position["iqref"]),
        "counters": counters,
        "final_position": final_position,
    }


def _discover_qualification(peer, serial, firmware_id):
    feedback = peer.read_feedback()
    if feedback is None:
        raise AssertionError("no fresh Node 1 feedback for preflight")
    config = validate_preflight(
        read_joint_config(serial), feedback.actual_position_mdeg
    )
    can_before = read_can_status(serial)
    for _attempt in range(3):
        peer.stop()
    peer.discover()
    health = validate_health(peer.read_health())
    can_after = read_can_status(serial)
    encoder_neutral = {"invalid": 0, "freeze": 0, "streak": 0, "fault": 0}
    counters = validate_counter_growth(
        encoder_neutral, encoder_neutral, can_before, can_after
    )
    return {
        "phase": "discover",
        "passed": True,
        "firmware_id": firmware_id,
        "config": config_fingerprint(config),
        "health": health,
        "node_state": health["node_state"],
        "counters": counters,
    }


def run_phase(
    phase,
    *,
    peer,
    serial=None,
    serial_factory=_STAGE8.open_port,
    port=PORT,
    evidence_dir=DEFAULT_EVIDENCE_DIR,
    firmware_id,
    session_factory=lambda: secrets.randbelow(0xFFFF) + 1,
):
    if phase not in ("discover", "step1", "step5"):
        raise ValueError("run_phase accepts discover, step1, or step5")

    def qualification(qualified_peer, qualified_serial):
        if phase == "discover":
            result = _discover_qualification(
                qualified_peer, qualified_serial, firmware_id
            )
        else:
            result = _motion_qualification(
                phase,
                qualified_peer,
                qualified_serial,
                firmware_id,
                evidence_dir,
                session_factory,
            )
        events = getattr(qualified_peer, "events", None)
        if isinstance(events, list):
            result["events"] = list(events)
        return result

    return run_can_bench(
        peer,
        serial_adapter=serial,
        qualification=qualification,
        serial_factory=serial_factory,
        port=port,
    )


def probe_dll(dll_path, device_factory=ControlCanDevice):
    device = device_factory(dll_path)
    missing = [
        name for name in REQUIRED_CONTROLCAN_EXPORTS
        if not hasattr(device.library, name)
    ]
    if missing:
        raise RuntimeError(f"ControlCAN DLL missing exports: {', '.join(missing)}")
    return {
        "phase": "probe",
        "passed": True,
        "dll": str(Path(dll_path)),
        "exports": list(REQUIRED_CONTROLCAN_EXPORTS),
    }


def current_firmware_id():
    completed = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip()


def write_evidence_atomic(evidence_dir, evidence, *, replace=os.replace):
    directory = Path(evidence_dir)
    directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    destination = directory / f"{timestamp}-{evidence['phase']}.json"
    temporary = directory / f".{destination.name}.{os.getpid()}.tmp"
    payload = dict(evidence)
    payload["timestamp_utc"] = timestamp
    try:
        with temporary.open("x", encoding="utf-8", newline="\n") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        replace(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return destination


def execute_phase(
    phase,
    *,
    dll_path,
    port=PORT,
    evidence_dir=DEFAULT_EVIDENCE_DIR,
    firmware_id=None,
    device_factory=ControlCanDevice,
    peer_factory=CanalystMotorPeer,
    serial_factory=_STAGE8.open_port,
    replace=os.replace,
):
    firmware = current_firmware_id() if firmware_id is None else firmware_id
    if phase == "probe":
        evidence = probe_dll(dll_path, device_factory=device_factory)
        evidence["firmware_id"] = firmware
        evidence["cleanup_complete"] = True
        return evidence, write_evidence_atomic(
            evidence_dir, evidence, replace=replace
        )

    primary = None
    primary_traceback = None
    evidence = None
    try:
        with device_factory(dll_path) as device:
            peer = peer_factory(device)
            evidence = run_phase(
                phase,
                peer=peer,
                serial_factory=serial_factory,
                port=port,
                evidence_dir=evidence_dir,
                firmware_id=firmware,
            )
    except BaseException as error:
        primary = error
        primary_traceback = error.__traceback__
    if primary is not None:
        failure = {
            "phase": phase,
            "passed": False,
            "firmware_id": firmware,
            "cleanup_complete": True,
            "error": {
                "type": type(primary).__name__,
                "message": str(primary),
            },
        }
        try:
            path = write_evidence_atomic(evidence_dir, failure, replace=replace)
        except BaseException as evidence_error:
            if hasattr(primary, "add_note"):
                primary.add_note(f"failure evidence write failed: {evidence_error}")
        else:
            if hasattr(primary, "add_note"):
                primary.add_note(f"failure evidence: {path}")
        raise primary.with_traceback(primary_traceback)
    evidence["cleanup_complete"] = True
    return evidence, write_evidence_atomic(evidence_dir, evidence, replace=replace)


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        evidence, path = execute_phase(
            args.phase,
            dll_path=args.dll,
            port=args.port,
            evidence_dir=args.evidence_dir,
        )
    except BaseException as error:
        detail = str(error).encode("ascii", errors="backslashreplace").decode("ascii")
        print(f"FAIL: {type(error).__name__}: {detail}", flush=True)
        return 1
    print(json.dumps(evidence, sort_keys=True), flush=True)
    print(f"evidence: {path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
