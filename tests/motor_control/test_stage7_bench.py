import builtins
from contextlib import redirect_stdout
import importlib.util
import io
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BENCH_PATH = ROOT / "tests" / "stage7_bench.py"
SPEC = importlib.util.spec_from_file_location("stage7_bench_under_test", BENCH_PATH)
BENCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BENCH)


class FakeSerial:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


def make_status_line(active=1, target=10000, velocity=30000,
                     reference=10150, measured=9820, error=330,
                     speed_target=1260, speed_measured=1200, iqref=180,
                     age=15, timeout=0, sequence=42, fault=0):
    values = (
        active, target, velocity, reference, measured, error,
        speed_target, speed_measured, iqref, age, timeout, sequence, fault,
    )
    checksum = 0x504F5331
    for value in values:
        checksum ^= value & 0xFFFFFFFF
    return (
        f"ps a={active} t={target} v={velocity} r={reference} m={measured} "
        f"e={error} w={speed_target} x={speed_measured} q={iqref} "
        f"g={age} o={timeout} n={sequence} f={fault:08X} k={checksum:08X}"
    )


def test_parse_compact_position_status():
    text = make_status_line()
    assert BENCH.parse_position_status(text) == {
        "active": 1,
        "target": 10000,
        "velocity": 30000,
        "reference": 10150,
        "measured": 9820,
        "error": 330,
        "speed_target": 1260,
        "speed_measured": 1200,
        "iqref": 180,
        "age": 15,
        "timeout": 0,
        "sequence": 42,
        "fault": 0,
    }
    assert BENCH.parse_position_status(text.replace("m=9820", "m=9821")) is None
    assert BENCH.parse_position_status(text[:-10]) is None
    assert BENCH.parse_position_status(text.replace(" e=330", "")) is None
    assert BENCH.parse_position_status(text.replace("n=42", "n=x")) is None


def test_status_reader_retries_corrupt_reply():
    replies = iter(("ps a=1 truncated", make_status_line()))
    original = BENCH.send_cmd
    BENCH.send_cmd = lambda *_args, **_kwargs: next(replies)
    try:
        sample = BENCH.read_position_status(FakeSerial(), attempts=2)
    finally:
        BENCH.send_cmd = original
    assert sample["sequence"] == 42


def test_stream_uses_short_shell_alias():
    source = BENCH_PATH.read_text(encoding="utf-8")
    assert 'f"mp {sequence & 0xFFFF} {position_mdeg} {velocity_mdeg_s}"' in source
    assert "send_stream_point(ser, sequence, 0, 0)" in source
    assert BENCH.STREAM_PERIOD_S == 0.02
    assert "period_ms=20" in source
    assert "trajectory_time += min(point_delta_s, period_s * 2.0)" in source
    assert "point_rate_hz" in source
    assert "retry_count" in source


def test_stream_point_retries_transient_shell_parse_failure():
    replies = iter((
        "usage: mc_pos_stream <seq> <position_mdeg> <velocity_mdeg_s>",
        "position stream: seq=7 target=100 vel=-20",
    ))
    commands = []
    original_send = BENCH.send_cmd
    original_sleep = BENCH.time.sleep
    BENCH.send_cmd = lambda _ser, cmd, **_kwargs: (
        commands.append(cmd) or next(replies)
    )
    BENCH.time.sleep = lambda _delay: None
    try:
        BENCH.send_stream_point(FakeSerial(), 7, 100, -20)
    finally:
        BENCH.send_cmd = original_send
        BENCH.time.sleep = original_sleep
    assert commands == ["mp 7 100 -20", "mp 7 100 -20"]


def test_main_settles_at_zero_before_sine_stream():
    source = BENCH_PATH.read_text(encoding="utf-8")
    main = source.split("def main", 1)[1]
    assert main.index("settle_position_at_zero(ser)") < main.index(
        "run_sine_stream(ser)"
    )


def test_timeout_metrics_require_zero_feedforward_and_frozen_reference():
    before = BENCH.parse_position_status(make_status_line(reference=10200, age=20))
    timed_out = BENCH.parse_position_status(
        make_status_line(velocity=0, reference=10200, age=100, timeout=1)
    )
    later = BENCH.parse_position_status(
        make_status_line(velocity=0, reference=10200, age=120, timeout=1)
    )
    metrics = BENCH.summarize_timeout(before, timed_out, later)
    assert metrics["feedforward_zero"]
    assert metrics["reference_frozen"]
    assert metrics["timeout_seen"]


def test_sampling_quality_guard_rejects_invalid_or_freeze_growth():
    before = {"invalid": 10, "freeze": 4, "streak": 0, "fault": 0}
    after = {"invalid": 10, "freeze": 4, "streak": 0, "fault": 0}
    assert BENCH.summarize_sampling_quality(before, after) == {
        "invalid_delta": 0,
        "freeze_delta": 0,
        "streak": 0,
        "fault": 0,
    }
    bad = dict(after, invalid=11)
    try:
        BENCH.summarize_sampling_quality(before, bad)
    except AssertionError:
        pass
    else:
        raise AssertionError("invalid growth must fail the position bench")


def test_main_always_stops_and_closes_after_failure():
    fake_serial = FakeSerial()
    commands = []
    originals = (
        BENCH.open_port,
        BENCH.prepare_position_bench,
        BENCH.run_static_steps,
        BENCH.send_cmd,
    )
    original_open = builtins.open
    BENCH.open_port = lambda _port: fake_serial
    BENCH.prepare_position_bench = lambda _ser: None
    BENCH.run_static_steps = lambda *_args, **_kwargs: (
        _ for _ in ()
    ).throw(RuntimeError("boom"))
    BENCH.send_cmd = lambda _ser, cmd, **_kwargs: commands.append(cmd) or ""
    builtins.open = lambda path, *args, **kwargs: (
        io.StringIO() if str(path).endswith("stage7_bench_log.txt")
        else original_open(path, *args, **kwargs)
    )
    try:
        with redirect_stdout(io.StringIO()):
            status = BENCH.main(["stage7_bench.py", "TEST"])
    finally:
        builtins.open = original_open
        (
            BENCH.open_port,
            BENCH.prepare_position_bench,
            BENCH.run_static_steps,
            BENCH.send_cmd,
        ) = originals
    assert status != 0
    assert "mc_stop" in commands
    assert fake_serial.closed


if __name__ == "__main__":
    test_parse_compact_position_status()
    test_status_reader_retries_corrupt_reply()
    test_stream_uses_short_shell_alias()
    test_stream_point_retries_transient_shell_parse_failure()
    test_main_settles_at_zero_before_sine_stream()
    test_timeout_metrics_require_zero_feedforward_and_frozen_reference()
    test_sampling_quality_guard_rejects_invalid_or_freeze_growth()
    test_main_always_stops_and_closes_after_failure()
    print("stage7 bench tests passed")
