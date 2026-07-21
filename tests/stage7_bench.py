"""Stage 7 single-motor position/feedforward bench for the COM9 shell."""

import importlib.util
import math
from pathlib import Path
import re
import statistics
import sys
import time


_STAGE6_PATH = Path(__file__).with_name("stage6_bench.py")
_STAGE6_SPEC = importlib.util.spec_from_file_location(
    "stage6_bench_for_stage7", _STAGE6_PATH
)
_STAGE6 = importlib.util.module_from_spec(_STAGE6_SPEC)
_STAGE6_SPEC.loader.exec_module(_STAGE6)

PORT = "COM9"
POSITION_STATUS_CHECK_SEED = 0x504F5331
POSITION_IQ_LIMIT_MA = 500
STATIC_SETTLE_S = 2.0
STATUS_PERIOD_S = 0.05
STREAM_PERIOD_S = 0.01

open_port = _STAGE6.open_port
send_cmd = _STAGE6.send_cmd
send_until_matches = _STAGE6.send_until_matches

STATUS_RE = re.compile(
    r"ps\s+a=(\d+)\s+t=(-?\d+)\s+v=(-?\d+)\s+r=(-?\d+)\s+"
    r"m=(-?\d+)\s+e=(-?\d+)\s+w=(-?\d+)\s+x=(-?\d+)\s+"
    r"q=(-?\d+)\s+g=(\d+)\s+o=(\d+)\s+n=(\d+)\s+"
    r"f=([0-9A-Fa-f]+)\s+k=([0-9A-Fa-f]+)"
)


def parse_position_status(text):
    match = STATUS_RE.search(text)
    if match is None:
        return None
    groups = match.groups()
    values = [int(value, 16) if index in (12, 13) else int(value)
              for index, value in enumerate(groups)]
    keys = (
        "active", "target", "velocity", "reference", "measured", "error",
        "speed_target", "speed_measured", "iqref", "age", "timeout",
        "sequence", "fault", "check",
    )
    sample = dict(zip(keys, values))
    expected = POSITION_STATUS_CHECK_SEED
    for key in keys[:-1]:
        expected ^= sample[key] & 0xFFFFFFFF
    if sample["check"] != expected:
        return None
    del sample["check"]
    return sample


def read_position_status(ser, attempts=5):
    last_text = ""
    for _ in range(attempts):
        last_text = send_cmd(ser, "mc_pos_status", timeout=1.0)
        sample = parse_position_status(last_text)
        if sample is not None:
            return sample
    safe = last_text.encode("ascii", errors="backslashreplace").decode("ascii")
    raise AssertionError(f"no valid position status after {attempts} attempts: {safe!r}")


def assert_position_sample_safe(sample, active=True):
    if active:
        assert sample["active"] == 1, sample
    assert sample["fault"] == 0, sample
    assert abs(sample["iqref"]) <= POSITION_IQ_LIMIT_MA, sample


def prepare_position_bench(ser):
    _STAGE6.prepare_bench(ser)
    send_until_matches(ser, "mc_pos_zero 0", (r"position zero:",))
    sample = read_position_status(ser)
    assert sample["active"] == 0, sample
    assert sample["fault"] == 0, sample


def collect_for(ser, duration_s, active=True):
    started = time.monotonic()
    samples = []
    while time.monotonic() - started < duration_s:
        cycle = time.monotonic()
        sample = read_position_status(ser)
        assert_position_sample_safe(sample, active=active)
        samples.append((time.monotonic() - started, sample))
        remaining = STATUS_PERIOD_S - (time.monotonic() - cycle)
        if remaining > 0:
            time.sleep(remaining)
    return samples


def summarize_step(target_mdeg, samples):
    assert samples
    final = [sample for stamp, sample in samples
             if stamp >= samples[-1][0] - 0.5]
    final_error = statistics.median(abs(sample["error"]) for sample in final)
    directional = [sample["measured"] if target_mdeg >= 0
                   else -sample["measured"] for _, sample in samples]
    overshoot = max(0, max(directional) - abs(target_mdeg))
    overshoot_ratio = overshoot / abs(target_mdeg) if target_mdeg else 0.0
    return {
        "target_mdeg": target_mdeg,
        "final_error_mdeg": final_error,
        "overshoot_ratio": overshoot_ratio,
        "peak_iqref_ma": max(abs(sample["iqref"]) for _, sample in samples),
        "fault": samples[-1][1]["fault"],
    }


def run_static_steps(ser, targets=(5000, -5000)):
    results = []
    for target in targets:
        send_until_matches(
            ser, f"mc_pos {target}", (r"position hold:",), attempts=3
        )
        results.append(summarize_step(target, collect_for(ser, STATIC_SETTLE_S)))
    return results


def run_reversal(ser):
    return run_static_steps(ser, targets=(10000, -10000))


def run_sine_stream(ser, amplitude_deg=10.0, peak_velocity_deg_s=30.0,
                    period_ms=10):
    amplitude_mdeg = amplitude_deg * 1000.0
    omega = peak_velocity_deg_s / amplitude_deg
    duration_s = 2.0 * math.pi / omega
    period_s = period_ms * 0.001
    sequence = 1000
    started = time.monotonic()
    next_tick = started
    samples = []
    tick = 0
    while True:
        now = time.monotonic()
        elapsed = now - started
        if elapsed > duration_s:
            break
        position = int(round(amplitude_mdeg * math.sin(omega * elapsed)))
        velocity = int(round(amplitude_mdeg * omega * math.cos(omega * elapsed)))
        send_until_matches(
            ser,
            f"mc_pos_stream {sequence & 0xFFFF} {position} {velocity}",
            (r"position stream:",),
            attempts=2,
            timeout=0.5,
        )
        sequence += 1
        if tick % 5 == 0:
            sample = read_position_status(ser)
            assert_position_sample_safe(sample)
            samples.append((elapsed, sample))
        tick += 1
        next_tick += period_s
        remaining = next_tick - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
    send_until_matches(
        ser, f"mc_pos_stream {sequence & 0xFFFF} 0 0",
        (r"position stream:",), attempts=2, timeout=0.5,
    )
    final = read_position_status(ser)
    assert_position_sample_safe(final)
    samples.append((time.monotonic() - started, final))
    errors = sorted(abs(sample["error"]) for _, sample in samples)
    p95_index = max(0, math.ceil(len(errors) * 0.95) - 1)
    return {
        "samples": len(samples),
        "p95_error_mdeg": errors[p95_index],
        "peak_iqref_ma": max(abs(sample["iqref"]) for _, sample in samples),
        "final_velocity_mdeg_s": final["velocity"],
        "fault": final["fault"],
    }


def summarize_timeout(before, timed_out, later):
    return {
        "timeout_seen": timed_out["timeout"] == 1 and later["timeout"] == 1,
        "feedforward_zero": timed_out["velocity"] == 0 and later["velocity"] == 0,
        "reference_frozen": timed_out["reference"] == later["reference"],
        "reference_delta_mdeg": later["reference"] - timed_out["reference"],
        "age_ms": timed_out["age"],
        "before_reference_mdeg": before["reference"],
    }


def summarize_sampling_quality(before, after):
    metrics = {
        "invalid_delta": after["invalid"] - before["invalid"],
        "freeze_delta": after["freeze"] - before["freeze"],
        "streak": after["streak"],
        "fault": after["fault"],
    }
    assert metrics["invalid_delta"] == 0, metrics
    assert metrics["freeze_delta"] == 0, metrics
    assert metrics["streak"] == 0, metrics
    assert metrics["fault"] == 0, metrics
    return metrics


def verify_stream_timeout_hold(ser):
    send_until_matches(
        ser, "mc_pos_stream 30000 0 10000", (r"position stream:",),
        attempts=2, timeout=0.5,
    )
    before = read_position_status(ser)
    time.sleep(0.13)
    timed_out = read_position_status(ser)
    time.sleep(0.03)
    later = read_position_status(ser)
    for sample in (before, timed_out, later):
        assert_position_sample_safe(sample)
    metrics = summarize_timeout(before, timed_out, later)
    assert metrics["timeout_seen"], metrics
    assert metrics["feedforward_zero"], metrics
    assert metrics["reference_frozen"], metrics
    return metrics


def verify_final_safe_state(ser):
    summary = _STAGE6.verify_final_safe_state(ser)
    sample = read_position_status(ser)
    assert sample["active"] == 0, sample
    assert sample["fault"] == 0, sample
    return summary + " position_active=0"


def main(argv=None):
    args = sys.argv if argv is None else argv
    port = args[1] if len(args) > 1 else PORT
    ser = None
    log = []
    status_code = 1
    try:
        ser = open_port(port)
        prepare_position_bench(ser)
        sampling_before = _STAGE6.read_speed_status(ser)
        static = run_static_steps(ser)
        reversal = run_reversal(ser)
        sine = run_sine_stream(ser)
        timeout = verify_stream_timeout_hold(ser)
        sampling_after = _STAGE6.read_speed_status(ser)
        sampling = summarize_sampling_quality(sampling_before, sampling_after)
        for name, value in (
            ("static", static), ("reversal", reversal),
            ("sine", sine), ("timeout", timeout), ("sampling", sampling),
        ):
            line = f"{name}: {value}"
            print(line, flush=True)
            log.append(line)
        performance_ok = all(
            result["final_error_mdeg"] <= 500 for result in static
        ) and all(
            result["overshoot_ratio"] <= 0.2 for result in reversal
        ) and sine["p95_error_mdeg"] <= 2000 and (
            sine["final_velocity_mdeg_s"] == 0
        )
        final = verify_final_safe_state(ser)
        print(final, flush=True)
        log.append(final)
        status_code = 0 if performance_ok else 2
    except Exception as exc:
        detail = str(exc).encode("ascii", errors="backslashreplace").decode("ascii")
        message = f"FAIL: {type(exc).__name__}: {detail}"
        print(message, flush=True)
        log.append(message)
    finally:
        if ser is not None:
            try:
                send_cmd(ser, "mc_stop")
            except Exception as exc:
                log.append(f"STOP FAIL: {exc}")
            ser.close()
        with open("tests/stage7_bench_log.txt", "w", encoding="utf-8") as handle:
            handle.write("\n".join(log) + "\n")
    return status_code


if __name__ == "__main__":
    raise SystemExit(main())
