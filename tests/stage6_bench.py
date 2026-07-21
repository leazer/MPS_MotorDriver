"""Stage 6 speed-loop bench for the COM9 RT-Thread shell."""

import re
import statistics
import sys
import time

import serial


PORT = "COM9"
BAUD = 115200
TARGETS_RPM_ELEC = (10, 60, -60, 200)
SAMPLE_PERIOD_S = 0.05
RUN_DURATION_S = 5.0
SAFE_DUTY_TICKS = 2812
EXPECTED_SAMPLE_TICK = 5264
SPEED_IQ_LIMIT_MA = 500
SETTLE_SPEED_MRAD_S = 500
SETTLE_CONSECUTIVE_SAMPLES = 4

STATUS_RE = re.compile(
    r"ss\s+a=(\d+)\s+t=(-?\d+)\s+c=(-?\d+)\s+m=(-?\d+)\s+"
    r"u=(-?\d+)\s+d=(-?\d+)\s+q=(-?\d+)\s+n=(\d+)\s+"
    r"s=(\d+)\s+z=(\d+)\s+f=([0-9A-Fa-f]+)\s+"
    r"e=(\d+)\s+k=([0-9A-Fa-f]+)"
)
SPEED_STATUS_CHECK_SEED = 0x53504436


def open_port(port):
    ser = serial.Serial(port, BAUD, timeout=0.02)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def read_until_prompt(ser, timeout=2.0):
    data = bytearray()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        waiting = ser.in_waiting
        if waiting:
            data.extend(ser.read(waiting))
            if b"msh >" in data or b"msh />" in data:
                break
        else:
            time.sleep(0.002)
    # Motor commutation can inject an occasional corrupt UART byte.  The shell
    # protocol is ASCII, so dropping non-ASCII bytes preserves parseable fields
    # and avoids U+FFFD failing on Windows' default GBK console encoding.
    return data.decode("ascii", errors="ignore")


def send_cmd(ser, cmd, timeout=2.0):
    ser.reset_input_buffer()
    ser.write((cmd + "\r").encode("ascii"))
    ser.flush()
    return read_until_prompt(ser, timeout=timeout)


def send_until_matches(ser, cmd, patterns, attempts=5, timeout=2.0):
    """Repeat an idempotent/safe shell command until its reply is complete."""
    last_text = ""
    for _ in range(attempts):
        last_text = send_cmd(ser, cmd, timeout=timeout)
        if all(re.search(pattern, last_text) for pattern in patterns):
            return last_text
    safe_text = last_text.encode("ascii", errors="backslashreplace").decode("ascii")
    raise AssertionError(f"incomplete {cmd} reply after {attempts} attempts: {safe_text!r}")


def parse_speed_status(text):
    match = STATUS_RE.search(text)
    if match is None:
        return None
    values = [int(value, 16) if index in (10, 12) else int(value)
              for index, value in enumerate(match.groups())]
    keys = (
        "active", "target", "cmd", "meas", "iqref", "id", "iq",
        "invalid", "streak", "freeze", "fault", "enc", "check",
    )
    snapshot = dict(zip(keys, values))
    expected = SPEED_STATUS_CHECK_SEED
    for key in keys[:-1]:
        expected ^= snapshot[key] & 0xFFFFFFFF
    if snapshot["check"] != expected:
        return None
    del snapshot["check"]
    return snapshot


def read_speed_status(ser, attempts=5):
    """Read one complete compact status, tolerating isolated UART corruption."""
    last_text = ""
    for _ in range(attempts):
        last_text = send_cmd(ser, "mc_speed_status", timeout=1.0)
        snapshot = parse_speed_status(last_text)
        if snapshot is not None:
            return snapshot
    safe_text = last_text.encode("ascii", errors="backslashreplace").decode("ascii")
    raise AssertionError(f"no valid speed status after {attempts} attempts: {safe_text!r}")


def start_speed_loop(ser, rpm_elec, attempts=3):
    """Start a speed target and confirm it even if the UART ack is damaged."""
    expected_target = int(rpm_elec * 6.28318530718 * 1000.0 / 60.0)
    last_text = ""
    for _ in range(attempts):
        last_text = send_cmd(ser, f"mc_speed {rpm_elec}")
        if "speed loop:" in last_text:
            return
        try:
            snapshot = read_speed_status(ser)
        except AssertionError:
            snapshot = None
        if (snapshot is not None and snapshot["active"] == 1 and
                abs(snapshot["target"] - expected_target) <= 1):
            return
        send_cmd(ser, "mc_stop")
    safe_text = last_text.encode("ascii", errors="backslashreplace").decode("ascii")
    raise AssertionError(f"speed loop start failed after {attempts} attempts: {safe_text!r}")


def settle_motor_at_zero(ser, timeout_s=5.0):
    """Actively brake the free shaft to rest, then leave the bridge disabled."""
    send_cmd(ser, "mc_stop")
    try:
        start_speed_loop(ser, 0)
        deadline = time.monotonic() + timeout_s
        settled = 0
        time.sleep(0.1)
        while time.monotonic() < deadline:
            snapshot = read_speed_status(ser)
            assert snapshot["active"] == 1, snapshot
            assert snapshot["fault"] == 0, snapshot
            assert snapshot["streak"] == 0, snapshot
            assert abs(snapshot["iqref"]) <= SPEED_IQ_LIMIT_MA, snapshot
            if abs(snapshot["meas"]) <= SETTLE_SPEED_MRAD_S:
                settled += 1
                if settled >= SETTLE_CONSECUTIVE_SAMPLES:
                    return
            else:
                settled = 0
            time.sleep(SAMPLE_PERIOD_S)
        raise AssertionError("motor did not settle at zero speed")
    finally:
        send_cmd(ser, "mc_stop")


def prepare_bench(ser):
    send_cmd(ser, "mc_stop")
    send_cmd(ser, "fault_clear")
    send_until_matches(
        ser, "mc_cal", (r"mc_cal result:\s*PASS\s+offset_valid=1",),
        attempts=3, timeout=4.0,
    )
    send_until_matches(ser, "enc_cal_status", (r"(?m)^valid\s*:\s*1\s*$",))
    send_until_matches(ser, "fault", (r"fault\s*=\s*0x0+\b",))
    settle_motor_at_zero(ser)


def summarize_target(rpm_elec, samples):
    assert samples, "no speed samples"
    end_time = samples[-1][0]
    final = [snapshot for stamp, snapshot in samples if stamp >= end_time - 1.0]
    if not final:
        final = [samples[-1][1]]
    target = samples[-1][1]["target"]
    steady = statistics.median(snapshot["meas"] for snapshot in final)
    signed_meas = [snapshot["meas"] if target >= 0 else -snapshot["meas"]
                   for _, snapshot in samples]
    target_magnitude = abs(target)
    peak = max(signed_meas)
    rise_time = None
    for stamp, snapshot in samples:
        directional = snapshot["meas"] if target >= 0 else -snapshot["meas"]
        if target_magnitude > 0 and directional >= target_magnitude * 0.9:
            rise_time = stamp
            break
    steady_error_pct = (
        abs(steady - target) * 100.0 / target_magnitude
        if target_magnitude > 0 else 0.0
    )
    peak_ratio = peak / target_magnitude if target_magnitude > 0 else 0.0
    return {
        "rpm_elec": rpm_elec,
        "target": target,
        "steady": steady,
        "steady_error_pct": steady_error_pct,
        "peak_ratio": peak_ratio,
        "rise_time_s": rise_time,
        "peak_iqref_ma": max(abs(snapshot["iqref"]) for _, snapshot in samples),
        "peak_id_ma": max(abs(snapshot["id"]) for _, snapshot in samples),
        "invalid_delta": samples[-1][1]["invalid"] - samples[0][1]["invalid"],
        "freeze_delta": samples[-1][1]["freeze"] - samples[0][1]["freeze"],
        "fault": samples[-1][1]["fault"],
        "direction_ok": steady == 0 or (steady > 0) == (target > 0),
    }


def run_target(ser, rpm_elec, duration_s=RUN_DURATION_S):
    start_speed_loop(ser, rpm_elec)
    started = time.monotonic()
    samples = []
    while time.monotonic() - started < duration_s:
        cycle = time.monotonic()
        snapshot = read_speed_status(ser)
        assert snapshot["active"] == 1, snapshot
        assert snapshot["fault"] == 0, snapshot
        assert snapshot["streak"] == 0, snapshot
        assert abs(snapshot["iqref"]) <= SPEED_IQ_LIMIT_MA, snapshot
        samples.append((time.monotonic() - started, snapshot))
        remaining = SAMPLE_PERIOD_S - (time.monotonic() - cycle)
        if remaining > 0:
            time.sleep(remaining)
    metrics = summarize_target(rpm_elec, samples)
    assert metrics["invalid_delta"] == 0, metrics
    assert metrics["freeze_delta"] == 0, metrics
    return metrics


def verify_final_safe_state(ser):
    send_cmd(ser, "mc_stop")
    state = send_until_matches(ser, "mc_state", (r"state\s*:\s*0\b",))
    fault = send_until_matches(ser, "fault", (r"fault\s*=\s*0x0+\b",))
    pwm = send_until_matches(
        ser,
        "pwm_info",
        (
            r"CCR1/2/3\s*:\s*2812\s*/\s*2812\s*/\s*2812",
            rf"CCR4\s*:\s*{EXPECTED_SAMPLE_TICK}\b",
            r"EN\(PB10\)\s*:\s*0\b",
        ),
    )
    status = read_speed_status(ser)
    assert re.search(r"state\s*:\s*0\b", state), state
    assert re.search(r"fault\s*=\s*0x0+\b", fault), fault
    duties = re.search(r"CCR1/2/3\s*:\s*(\d+)\s*/\s*(\d+)\s*/\s*(\d+)", pwm)
    assert duties and tuple(map(int, duties.groups())) == (SAFE_DUTY_TICKS,) * 3, pwm
    assert re.search(rf"CCR4\s*:\s*{EXPECTED_SAMPLE_TICK}\b", pwm), pwm
    assert re.search(r"EN\(PB10\)\s*:\s*0\b", pwm), pwm
    assert status["active"] == 0 and status["fault"] == 0, status
    return (
        "FINAL SAFE: state=DISABLED fault=0x00000000 en=0 "
        "ccr=2812/2812/2812 tick=5264 speed_active=0"
    )


def format_metrics(metrics):
    rise = "none" if metrics["rise_time_s"] is None else f"{metrics['rise_time_s']:.3f}s"
    return (
        f"{metrics['rpm_elec']:+d}rpm_e: target={metrics['target']} "
        f"steady={metrics['steady']:.0f} err={metrics['steady_error_pct']:.2f}% "
        f"peak={metrics['peak_ratio'] * 100.0:.1f}% rise90={rise} "
        f"peak_iqref={metrics['peak_iqref_ma']}mA peak_id={metrics['peak_id_ma']}mA "
        f"invalid_delta={metrics['invalid_delta']} freeze_delta={metrics['freeze_delta']} "
        f"fault=0x{metrics['fault']:08X} direction_ok={int(metrics['direction_ok'])}"
    )


def main(argv=None):
    args = sys.argv if argv is None else argv
    port = args[1] if len(args) > 1 else PORT
    ser = None
    log = []
    status_code = 1
    try:
        ser = open_port(port)
        prepare_bench(ser)
        performance_ok = True
        for rpm_elec in TARGETS_RPM_ELEC:
            metrics = run_target(ser, rpm_elec)
            line = format_metrics(metrics)
            print(line, flush=True)
            log.append(line)
            settle_motor_at_zero(ser)
            if not metrics["direction_ok"]:
                performance_ok = False
            if abs(rpm_elec) >= 60 and (
                metrics["steady_error_pct"] > 5.0 or metrics["peak_ratio"] > 1.2
            ):
                performance_ok = False
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
        with open("tests/stage6_bench_log.txt", "w", encoding="utf-8") as handle:
            handle.write("\n".join(log) + "\n")
    return status_code


if __name__ == "__main__":
    raise SystemExit(main())
