import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BENCH_PATH = ROOT / "tests" / "stage8_can_bench.py"
SPEC = importlib.util.spec_from_file_location("stage8_can_bench_under_test", BENCH_PATH)
BENCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BENCH)


def make_status_line(**changes):
    values = {
        "id": 1, "s": 3, "se": 4660, "p": 43, "a": 42,
        "pa": 0, "sa": 7, "rx": 120, "tx": 240, "pe": 0,
        "ro": 0, "bo": 0, "te": 0, "f": 0,
    }
    values.update(changes)
    checksum = BENCH.CAN_STATUS_CHECK_SEED
    for key in BENCH.CAN_STATUS_FIELDS:
        checksum ^= values[key] & 0xFFFFFFFF
    return (
        f"cs id={values['id']} s={values['s']} se={values['se']} "
        f"p={values['p']} a={values['a']} pa={values['pa']} "
        f"sa={values['sa']} rx={values['rx']} tx={values['tx']} "
        f"pe={values['pe']} ro={values['ro']} bo={values['bo']} "
        f"te={values['te']} f={values['f']:08X} k={checksum:08X}"
    )


def expect_rejected(line):
    assert BENCH.parse_can_status(line) is None, line


def test_checked_status_parser_accepts_only_the_frozen_ordered_line():
    line = make_status_line()
    assert BENCH.parse_can_status(line) == {
        "id": 1, "s": 3, "se": 4660, "p": 43, "a": 42,
        "pa": 0, "sa": 7, "rx": 120, "tx": 240, "pe": 0,
        "ro": 0, "bo": 0, "te": 0, "f": 0,
    }
    assert BENCH.parse_can_status(line + "\r\n") == BENCH.parse_can_status(line)
    for bad in (
        line.replace(" pa=0", ""),
        line.replace(" pa=0", " pa=0 pa=0"),
        line.replace(" pa=0", " extra=9 pa=0"),
        line.replace("rx=120", "rx=-120"),
        line.replace("se=4660", "se=+4660"),
        line.replace("f=00000000", "f=0000000"),
        line.replace("f=00000000", "f=0000000g"),
        line.replace("f=00000000", "f=abcdef01"),
        line[:-1],
        line + " garbage",
        "prompt " + line,
        line.replace("rx=120", "rx=121"),
        line.replace("k=", "k=G"),
        make_status_line(id=0),
        make_status_line(id=3),
        make_status_line(s=6),
        make_status_line(s=99),
    ):
        expect_rejected(bad)


def test_parser_enforces_source_widths_and_uint32_checksum_masking():
    line = make_status_line(
        se=65535, p=65535, a=65535, pa=65535, sa=65535,
        rx=0xFFFFFFFF, tx=0xFFFFFFFF, pe=0xFFFFFFFF,
        ro=0xFFFFFFFF, bo=0xFFFFFFFF, te=0xFFFFFFFF, f=0xFFFFFFFF,
    )
    assert BENCH.parse_can_status(line)["rx"] == 0xFFFFFFFF
    for bad in (
        make_status_line(se=65536),
        make_status_line(p=65536),
        make_status_line(pa=65536),
        make_status_line(rx=0x100000000),
    ):
        expect_rejected(bad)


def test_sequence_metrics_are_wrap_aware_and_validate_inputs():
    metrics = BENCH.sequence_metrics([65534, 65535, 0, 2, 2, 3], 0.06)
    assert metrics == {
        "points": 6,
        "point_rate_hz": 100.0,
        "missing_sequences": 1,
        "duplicate_sequences": 1,
    }
    for sequences, elapsed in (([], 1.0), ([1], 0.0), ([1, -1], 1.0),
                               ([1, 65536], 1.0), ([1, 0], 1.0)):
        try:
            BENCH.sequence_metrics(sequences, elapsed)
        except (TypeError, ValueError):
            pass
        else:
            raise AssertionError((sequences, elapsed))


def test_error_timeout_and_driver_metrics_are_explicit_and_checked():
    samples = [
        {"phase": "static", "target_mdeg": 1000, "actual_mdeg": 900,
         "iq_ma": -120},
        {"phase": "reversal", "target_mdeg": -1000, "actual_mdeg": -700,
         "iq_ma": 450},
        {"phase": "sine", "target_mdeg": 500, "actual_mdeg": 450,
         "iq_ma": -300},
        {"phase": "sine", "target_mdeg": -500, "actual_mdeg": -400,
         "iq_ma": 200},
    ]
    assert BENCH.error_metrics(samples) == {
        "static_error_mdeg": 100,
        "reversal_error_mdeg": 300,
        "sine_p95_error_mdeg": 100,
        "peak_iq_ma": 450,
    }
    assert BENCH.timeout_metrics(3.0, 3.051, 3.502) == {
        "hold_latency_ms": 51.0,
        "fatal_latency_ms": 502.0,
    }
    before = dict(rx=10, tx=20, pe=1, ro=2, bo=3, te=4)
    after = dict(rx=110, tx=219, pe=1, ro=4, bo=3, te=5)
    assert BENCH.driver_counter_deltas(before, after) == {
        "rx": 100, "tx": 199, "pe": 0, "ro": 2, "bo": 0, "te": 1,
    }
    for callback, args in (
        (BENCH.error_metrics, ([{"phase": "bad", "target_mdeg": 0,
                                "actual_mdeg": 0, "iq_ma": 0}],)),
        (BENCH.timeout_metrics, (3.0, 2.9, 3.5)),
        (BENCH.driver_counter_deltas, (before, dict(after, te=3))),
    ):
        try:
            callback(*args)
        except (TypeError, ValueError):
            pass
        else:
            raise AssertionError(callback.__name__)


class FakePeer:
    def __init__(self, events, fail_stops=()):
        self.events = events
        self.fail_stops = set(fail_stops)
        self.stop_count = 0

    def stop(self):
        self.stop_count += 1
        self.events.append(f"peer_stop_{self.stop_count}")
        if self.stop_count in self.fail_stops:
            raise RuntimeError(f"peer stop {self.stop_count}")


class FakeSerial:
    def __init__(self, events):
        self.events = events
        self.closed = False

    def close(self):
        self.events.append("close")
        self.closed = True


def test_cleanup_order_count_and_owned_resource_close_on_success():
    events = []
    peer = FakePeer(events)
    serial = FakeSerial(events)
    originals = BENCH.send_cmd, BENCH.verify_final_safe_state
    BENCH.send_cmd = lambda _serial, command, **_kwargs: events.append(command) or ""
    BENCH.verify_final_safe_state = lambda _serial: events.append("safe") or "SAFE"
    try:
        result = BENCH.run_can_bench(
            peer,
            qualification=lambda _peer, _serial: events.append("work") or {"ok": 1},
            serial_factory=lambda _port: serial,
        )
    finally:
        BENCH.send_cmd, BENCH.verify_final_safe_state = originals
    assert result == {"ok": 1}
    assert peer.stop_count == 3
    assert events == [
        "work", "peer_stop_1", "peer_stop_2", "peer_stop_3",
        "mc_stop", "safe", "close",
    ]
    assert serial.closed


def test_cleanup_preserves_primary_error_and_continues_all_failure_paths():
    events = []
    peer = FakePeer(events, fail_stops=(1, 2, 3))
    serial = FakeSerial(events)
    originals = BENCH.send_cmd, BENCH.verify_final_safe_state

    def fail_command(_serial, command, **_kwargs):
        events.append(command)
        raise RuntimeError("serial stop")

    def fail_safe(_serial):
        events.append("safe")
        raise RuntimeError("unsafe")

    BENCH.send_cmd = fail_command
    BENCH.verify_final_safe_state = fail_safe
    primary = ValueError("metric failure")
    try:
        try:
            BENCH.run_can_bench(
                peer,
                serial_adapter=serial,
                qualification=lambda *_args: (_ for _ in ()).throw(primary),
            )
        except ValueError as caught:
            assert caught is primary
            assert "cleanup" in " ".join(getattr(caught, "__notes__", ())).lower()
        else:
            raise AssertionError("primary error was swallowed")
    finally:
        BENCH.send_cmd, BENCH.verify_final_safe_state = originals
    assert events == [
        "peer_stop_1", "peer_stop_2", "peer_stop_3", "mc_stop", "safe",
    ]
    assert not serial.closed


def test_cleanup_only_failure_is_reported_after_every_attempt():
    events = []
    peer = FakePeer(events, fail_stops=(2,))
    serial = FakeSerial(events)
    originals = BENCH.send_cmd, BENCH.verify_final_safe_state
    BENCH.send_cmd = lambda _serial, command, **_kwargs: events.append(command) or ""
    BENCH.verify_final_safe_state = lambda _serial: events.append("safe") or "SAFE"
    try:
        try:
            BENCH.run_can_bench(
                peer, serial_adapter=serial,
                qualification=lambda *_args: {"ok": 1},
            )
        except BENCH.BenchCleanupError as caught:
            assert len(caught.errors) == 1
        else:
            raise AssertionError("cleanup failure was swallowed")
    finally:
        BENCH.send_cmd, BENCH.verify_final_safe_state = originals
    assert peer.stop_count == 3
    assert events[-2:] == ["mc_stop", "safe"]


def test_serial_setup_failure_still_attempts_peer_stop_three_times():
    events = []
    peer = FakePeer(events)
    setup_error = OSError("COM9 unavailable")
    try:
        BENCH.run_can_bench(
            peer,
            qualification=lambda *_args: None,
            serial_factory=lambda _port: (_ for _ in ()).throw(setup_error),
        )
    except OSError as caught:
        assert caught is setup_error
    else:
        raise AssertionError("serial setup failure was swallowed")
    assert events == ["peer_stop_1", "peer_stop_2", "peer_stop_3"]


if __name__ == "__main__":
    test_checked_status_parser_accepts_only_the_frozen_ordered_line()
    test_parser_enforces_source_widths_and_uint32_checksum_masking()
    test_sequence_metrics_are_wrap_aware_and_validate_inputs()
    test_error_timeout_and_driver_metrics_are_explicit_and_checked()
    test_cleanup_order_count_and_owned_resource_close_on_success()
    test_cleanup_preserves_primary_error_and_continues_all_failure_paths()
    test_cleanup_only_failure_is_reported_after_every_attempt()
    test_serial_setup_failure_still_attempts_peer_stop_three_times()
    print("stage8 CAN bench tests passed")
