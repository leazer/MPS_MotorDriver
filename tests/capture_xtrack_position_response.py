"""Capture checked COM9 telemetry for X-Track realtime position qualification."""

import argparse
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import time


ROOT = Path(__file__).resolve().parent


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


stage7 = _load("stage7_capture", ROOT / "stage7_bench.py")
stage8 = _load("stage8_capture", ROOT / "stage8_can_bench.py")


def _checked_position(serial):
    return stage7.read_position_status(serial, attempts=5)


def _checked_can(serial):
    last = ""
    for _ in range(5):
        last = stage8.send_cmd(serial, "can_status", timeout=1.0)
        line = re.search(r"cs [^\r\n]+(?:\r\n)?", last)
        parsed = stage8.parse_can_status(line.group(0)) if line else None
        if parsed is not None:
            return parsed
    raise RuntimeError("no complete checked can_status: " + repr(last))


def _final_state(serial):
    stage8.send_cmd(serial, "mc_stop", timeout=1.0)
    state_text = stage8.send_cmd(serial, "mc_state", timeout=1.0)
    fault_text = stage8.send_cmd(serial, "fault", timeout=1.0)
    pwm_text = stage8.send_cmd(serial, "pwm_info", timeout=1.0)
    state_match = re.search(r"state\s*:\s*(\d+)", state_text)
    fault_match = re.search(r"fault\s*=\s*0x([0-9A-Fa-f]+)", fault_text)
    pwm = stage8.parse_pwm_info(pwm_text)
    if state_match is None or fault_match is None or pwm is None:
        raise RuntimeError("incomplete final safe-state telemetry")
    return {
        "type": "final",
        "state": "DISABLED" if int(state_match.group(1)) == 0 else "RUNNING",
        "fault": int(fault_match.group(1), 16),
        "enable": pwm["en"],
        "ccr1": pwm["ccr1"],
        "ccr2": pwm["ccr2"],
        "ccr3": pwm["ccr3"],
    }


def _halt_xtrack(jlink_exe, halt_script, jlink_serial):
    subprocess.run(
        [
            str(jlink_exe), "-NoGui", "1", "-SelectEmuBySN",
            str(jlink_serial), "-Device", "AT32F435CGU7", "-If", "SWD",
            "-Speed", "4000", "-AutoConnect", "1", "-CommandFile",
            str(halt_script),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )


def capture(port, duration_s, output, period_s=0.05,
            jlink_exe=None, halt_script=None, jlink_serial=20721850):
    serial = stage8.open_port(port)
    started = time.monotonic()
    next_sample = started
    can = _checked_can(serial)
    records = []
    cycle = 0
    try:
        while time.monotonic() - started < duration_s:
            if cycle % 5 == 0:
                can = _checked_can(serial)
            position = _checked_position(serial)
            records.append({
                "type": "sample",
                "timestamp_ms": round((time.monotonic() - started) * 1000),
                "target_mdeg": position["target"],
                "measured_mdeg": position["measured"],
                "velocity_mdeg_s": position["velocity"],
                "iqref_ma": position["iqref"],
                "protocol_errors": can["pe"],
                "rx_overflows": can["ro"],
                "bus_off_events": can["bo"],
                "tx_errors": can["te"],
                "fault": position["fault"] | can["f"],
                "sync_age_ms": can["sa"],
                "session": can["se"],
                "can_state": can["s"],
                "pending_sequence": can["p"],
                "applied_sequence": can["a"],
                "position_age_ms": position["age"],
                "reference_mdeg": position["reference"],
                "speed_target_mdeg_s": position["speed_target"],
                "speed_measured_mdeg_s": position["speed_measured"],
                "active": position["active"],
            })
            cycle += 1
            next_sample += period_s
            delay = next_sample - time.monotonic()
            if delay > 0:
                time.sleep(delay)
    finally:
        if jlink_exe is not None and halt_script is not None:
            _halt_xtrack(jlink_exe, halt_script, jlink_serial)
        final = _final_state(serial)
        serial.close()
    records.append(final)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="\n") as stream:
        for record in records:
            stream.write(json.dumps(record, sort_keys=True) + "\n")
    return len(records) - 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--port", default="COM9")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--jlink-exe", type=Path)
    parser.add_argument("--halt-script", type=Path)
    parser.add_argument("--jlink-serial", type=int, default=20721850)
    args = parser.parse_args()
    if (args.jlink_exe is None) != (args.halt_script is None):
        parser.error("--jlink-exe and --halt-script must be supplied together")
    count = capture(
        args.port, args.duration, args.output, args.period,
        args.jlink_exe, args.halt_script, args.jlink_serial
    )
    print("captured_samples=" + str(count))
    print("output=" + str(args.output.resolve()))


if __name__ == "__main__":
    main()
