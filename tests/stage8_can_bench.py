"""Stage 8 checked CAN-node qualification helpers.

The module deliberately has no dependency on a particular USB-CAN or X-Track
implementation.  A caller injects an object providing the CAN peer operations
and may inject an already-open serial adapter for tests or powered runs.
"""

import importlib.util
import math
from pathlib import Path
import re
import sys


_STAGE7_PATH = Path(__file__).with_name("stage7_bench.py")
_STAGE7_SPEC = importlib.util.spec_from_file_location(
    "stage7_bench_for_stage8", _STAGE7_PATH
)
_STAGE7 = importlib.util.module_from_spec(_STAGE7_SPEC)
_STAGE7_SPEC.loader.exec_module(_STAGE7)

PORT = "COM9"
CAN_STATUS_CHECK_SEED = 0x43414E31
CAN_STATUS_FIELDS = (
    "id", "s", "se", "p", "a", "pa", "sa", "rx", "tx", "pe", "ro",
    "bo", "te", "f",
)
_UINT16_FIELDS = frozenset(("se", "p", "a", "pa", "sa"))
_COUNTER_FIELDS = ("rx", "tx", "pe", "ro", "bo", "te")
_PHASES = ("static", "reversal", "sine")

open_port = _STAGE7.open_port
send_cmd = _STAGE7.send_cmd
SAFE_DUTY_TICKS = _STAGE7._STAGE6.SAFE_DUTY_TICKS
EXPECTED_SAMPLE_TICK = _STAGE7._STAGE6.EXPECTED_SAMPLE_TICK

_CAN_STATUS_RE = re.compile(
    r"\Acs id=([0-9]+) s=([0-9]+) se=([0-9]+) p=([0-9]+) "
    r"a=([0-9]+) pa=([0-9]+) sa=([0-9]+) rx=([0-9]+) "
    r"tx=([0-9]+) pe=([0-9]+) ro=([0-9]+) bo=([0-9]+) "
    r"te=([0-9]+) f=([0-9A-F]{8}) k=([0-9A-F]{8})(?:\r\n)?\Z"
)

_PWM_DUTY_RE = re.compile(
    r"CCR1/2/3\s*:\s*(\d+)\s*/\s*(\d+)\s*/\s*(\d+)"
)
_PWM_TRIGGER_RE = re.compile(r"CCR4\s*:\s*(\d+)\b")
_PWM_ENABLE_RE = re.compile(r"EN\(PB10\)\s*:\s*([01])\b")


class BenchCleanupError(RuntimeError):
    """Raised after all cleanup steps ran but at least one step failed."""

    def __init__(self, errors):
        self.errors = tuple(errors)
        details = "; ".join(
            f"{type(error).__name__}: {error}" for error in self.errors
        )
        super().__init__(f"CAN bench cleanup failed: {details}")


def _plain_int(value, name, minimum=0, maximum=0xFFFFFFFF):
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an integer")
    if value < minimum or value > maximum:
        raise ValueError(f"{name} outside {minimum}..{maximum}")
    return value


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def can_status_checksum(values):
    """Return the firmware checksum for a mapping of checked status fields."""
    checksum = CAN_STATUS_CHECK_SEED
    for key in CAN_STATUS_FIELDS:
        if key not in values:
            raise ValueError(f"missing CAN status field {key}")
        checksum ^= _plain_int(values[key], key) & 0xFFFFFFFF
    return checksum & 0xFFFFFFFF


def parse_can_status(text):
    """Parse exactly one complete, ordered, checksummed ``cs`` line."""
    if not isinstance(text, str):
        return None
    match = _CAN_STATUS_RE.fullmatch(text)
    if match is None:
        return None
    groups = match.groups()
    try:
        values = {
            key: int(groups[index], 16) if key == "f" else int(groups[index])
            for index, key in enumerate(CAN_STATUS_FIELDS)
        }
        check = int(groups[-1], 16)
        if values["id"] not in (1, 2) or not 0 <= values["s"] <= 5:
            return None
        for key in _UINT16_FIELDS:
            _plain_int(values[key], key, maximum=0xFFFF)
        for key in ("rx", "tx", "pe", "ro", "bo", "te", "f"):
            _plain_int(values[key], key)
        if check != can_status_checksum(values):
            return None
    except (TypeError, ValueError):
        return None
    return values


def parse_pwm_info(text):
    """Parse the established Stage 6/7 safe-PWM fields from ``pwm_info``."""
    if not isinstance(text, str):
        return None
    duty = _PWM_DUTY_RE.search(text)
    trigger = _PWM_TRIGGER_RE.search(text)
    enabled = _PWM_ENABLE_RE.search(text)
    if duty is None or trigger is None or enabled is None:
        return None
    return {
        "ccr1": int(duty.group(1)),
        "ccr2": int(duty.group(2)),
        "ccr3": int(duty.group(3)),
        "ccr4": int(trigger.group(1)),
        "en": int(enabled.group(1)),
    }


def assert_pwm_safe_state(serial):
    """Independently query and assert the existing final PWM safety contract."""
    text = send_cmd(serial, "pwm_info", timeout=1.0)
    snapshot = parse_pwm_info(text)
    if snapshot is None:
        raise AssertionError("incomplete pwm_info reply")
    expected_duties = (SAFE_DUTY_TICKS,) * 3
    actual_duties = (snapshot["ccr1"], snapshot["ccr2"], snapshot["ccr3"])
    if actual_duties != expected_duties:
        raise AssertionError(f"unsafe PWM duties: {snapshot}")
    if snapshot["ccr4"] != EXPECTED_SAMPLE_TICK:
        raise AssertionError(f"unsafe ADC trigger: {snapshot}")
    if snapshot["en"] != 0:
        raise AssertionError(f"PWM output remains enabled: {snapshot}")
    return snapshot


def sequence_metrics(sequences, elapsed_s):
    """Compute rate/loss/duplicates for a forward uint16 sequence stream."""
    if not isinstance(sequences, (list, tuple)) or not sequences:
        raise ValueError("sequences must be a non-empty list or tuple")
    elapsed = _finite_number(elapsed_s, "elapsed_s")
    if elapsed <= 0.0:
        raise ValueError("elapsed_s must be positive")
    checked = [_plain_int(value, "sequence", maximum=0xFFFF)
               for value in sequences]
    missing = 0
    duplicates = 0
    previous = checked[0]
    for current in checked[1:]:
        delta = (current - previous) & 0xFFFF
        if delta == 0:
            duplicates += 1
            continue
        if delta >= 0x8000:
            raise ValueError("sequence stream moved backward or by half-range")
        missing += delta - 1
        previous = current
    return {
        "points": len(checked),
        "point_rate_hz": len(checked) / elapsed,
        "missing_sequences": missing,
        "duplicate_sequences": duplicates,
    }


def error_metrics(samples):
    """Summarize static/reversal/sine tracking error and peak absolute Iq."""
    if not isinstance(samples, (list, tuple)) or not samples:
        raise ValueError("samples must be a non-empty list or tuple")
    errors = {phase: [] for phase in _PHASES}
    peak_iq = 0
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            raise TypeError(f"sample {index} must be a mapping")
        phase = sample.get("phase")
        if phase not in _PHASES:
            raise ValueError(f"sample {index} has invalid phase")
        target = _plain_int(sample.get("target_mdeg"), "target_mdeg",
                            minimum=-0x80000000, maximum=0x7FFFFFFF)
        actual = _plain_int(sample.get("actual_mdeg"), "actual_mdeg",
                            minimum=-0x80000000, maximum=0x7FFFFFFF)
        iq_ma = _plain_int(sample.get("iq_ma"), "iq_ma",
                           minimum=-0x80000000, maximum=0x7FFFFFFF)
        errors[phase].append(abs(target - actual))
        peak_iq = max(peak_iq, abs(iq_ma))
    if any(not errors[phase] for phase in _PHASES):
        raise ValueError("samples must cover static, reversal, and sine phases")
    sine = sorted(errors["sine"])
    p95_index = max(0, math.ceil(len(sine) * 0.95) - 1)
    return {
        "static_error_mdeg": max(errors["static"]),
        "reversal_error_mdeg": max(errors["reversal"]),
        "sine_p95_error_mdeg": sine[p95_index],
        "peak_iq_ma": peak_iq,
    }


def timeout_metrics(sync_stopped_s, hold_seen_s, fatal_seen_s):
    """Return HOLD and fatal timeout latency from the last valid SYNC."""
    stopped = _finite_number(sync_stopped_s, "sync_stopped_s")
    hold = _finite_number(hold_seen_s, "hold_seen_s")
    fatal = _finite_number(fatal_seen_s, "fatal_seen_s")
    if stopped < 0.0 or hold < stopped or fatal < hold:
        raise ValueError("timeout timestamps must be ordered and non-negative")
    return {
        "hold_latency_ms": round((hold - stopped) * 1000.0, 3),
        "fatal_latency_ms": round((fatal - stopped) * 1000.0, 3),
    }


def driver_counter_deltas(before, after):
    """Return checked monotonic deltas for the six printed driver counters."""
    if not isinstance(before, dict) or not isinstance(after, dict):
        raise TypeError("counter snapshots must be mappings")
    deltas = {}
    for key in _COUNTER_FIELDS:
        if key not in before or key not in after:
            raise ValueError(f"missing counter {key}")
        old = _plain_int(before[key], f"before.{key}")
        new = _plain_int(after[key], f"after.{key}")
        if new < old:
            raise ValueError(f"counter {key} decreased")
        deltas[key] = new - old
    return deltas


def qualification_metrics(sequences, elapsed_s, samples, sync_stopped_s,
                          hold_seen_s, fatal_seen_s, status_before,
                          status_after):
    """Combine all Stage 8 acceptance metrics from validated raw observations."""
    result = sequence_metrics(sequences, elapsed_s)
    result.update(error_metrics(samples))
    result.update(timeout_metrics(sync_stopped_s, hold_seen_s, fatal_seen_s))
    result["driver_counters"] = driver_counter_deltas(status_before, status_after)
    return result


def _record_cleanup_error(errors, callback):
    try:
        callback()
    except BaseException as error:
        errors.append(error)


def run_can_bench(peer, serial_adapter=None, qualification=None,
                  serial_factory=open_port, port=PORT):
    """Run an injected qualification and always force both transports safe.

    ``qualification(peer, serial)`` owns the powered scenario and returns its
    metrics.  The supplied peer is expected to provide ``arm``, ``submit``,
    ``sync``, ``stop``, and feedback-read operations appropriate to its adapter.
    Only a serial object opened by ``serial_factory`` is closed here.
    """
    if peer is None or not callable(getattr(peer, "stop", None)):
        raise TypeError("peer must provide stop()")
    if qualification is None or not callable(qualification):
        raise TypeError("qualification must be callable")
    owned_serial = serial_adapter is None
    serial = serial_adapter
    primary = None
    primary_traceback = None
    result = None
    cleanup_errors = []
    try:
        if owned_serial:
            serial = serial_factory(port)
        if serial is None:
            raise ValueError("serial adapter is required")
        result = qualification(peer, serial)
    except BaseException as error:
        primary = error
        primary_traceback = error.__traceback__
    finally:
        for _attempt in range(3):
            _record_cleanup_error(cleanup_errors, peer.stop)
        if serial is not None:
            _record_cleanup_error(
                cleanup_errors, lambda: send_cmd(serial, "mc_stop", timeout=1.0)
            )
            _record_cleanup_error(
                cleanup_errors, lambda: assert_pwm_safe_state(serial)
            )
            if owned_serial:
                _record_cleanup_error(cleanup_errors, serial.close)

    if primary is not None:
        if cleanup_errors and hasattr(primary, "add_note"):
            primary.add_note(str(BenchCleanupError(cleanup_errors)))
        raise primary.with_traceback(primary_traceback)
    if cleanup_errors:
        raise BenchCleanupError(cleanup_errors)
    return result


def main(argv=None, peer=None, serial_adapter=None, qualification=None):
    """Injection-only CLI boundary; adapter-specific launchers call this API."""
    del argv
    try:
        metrics = run_can_bench(
            peer, serial_adapter=serial_adapter, qualification=qualification
        )
    except BaseException as error:
        detail = str(error).encode("ascii", errors="backslashreplace").decode("ascii")
        print(f"FAIL: {type(error).__name__}: {detail}", flush=True)
        return 1
    print(metrics, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
