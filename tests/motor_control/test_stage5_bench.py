import builtins
import importlib.util
import io
from pathlib import Path


BENCH_PATH = Path(__file__).resolve().parents[1] / "stage5_bench.py"
SPEC = importlib.util.spec_from_file_location("stage5_bench_under_test", BENCH_PATH)
BENCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BENCH)


class FakeSerial:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


def test_calibration_parser_requires_success_and_valid_offset():
    assert hasattr(BENCH, "parse_current_offset_calibration"), \
        "missing calibration result parser"
    assert BENCH.parse_current_offset_calibration(
        "mc_cal result: PASS offset_valid=1 a=2048 b=2049 c=2047"
    ) == (True, True)
    assert BENCH.parse_current_offset_calibration(
        "mc_cal result: FAIL offset_valid=1 a=2048 b=2049 c=2047"
    ) == (False, True)
    assert BENCH.parse_current_offset_calibration("offset OK") is None


def test_section_a_refuses_current_command_after_calibration_failure():
    commands = []

    def fake_send_cmd(_ser, cmd, **_kwargs):
        commands.append(cmd)
        responses = {
            "mc_state": "state : 0",
            "fault_clear": "fault cleared",
            "fault": "fault = 0x00000000",
            "mc_cal": "mc_cal result: FAIL offset_valid=0 a=2048 b=2048 c=2048",
            "mc_cur": "usage: mc_cur <iq_ma>",
        }
        return responses[cmd]

    original = BENCH.send_cmd
    BENCH.send_cmd = fake_send_cmd
    try:
        try:
            BENCH.section_a(FakeSerial(), [])
            raise AssertionError("section_a accepted failed current calibration")
        except AssertionError as exc:
            assert "current offset calibration failed" in str(exc)
    finally:
        BENCH.send_cmd = original
    assert "mc_cur" not in commands


def test_main_returns_failure_and_always_stops_and_closes():
    fake_serial = FakeSerial()
    commands = []
    originals = (BENCH.open_port, BENCH.section_a, BENCH.send_cmd)
    original_open = builtins.open
    BENCH.open_port = lambda _port: fake_serial
    BENCH.section_a = lambda _ser, _log: (_ for _ in ()).throw(RuntimeError("boom"))
    BENCH.send_cmd = lambda _ser, cmd, **_kwargs: commands.append(cmd) or ""
    builtins.open = lambda path, *args, **kwargs: (
        io.StringIO() if path == "tests/stage5_bench_log.txt"
        else original_open(path, *args, **kwargs)
    )
    try:
        try:
            status = BENCH.main(["stage5_bench.py", "TEST"])
        except TypeError as exc:
            raise AssertionError("main must accept argv for host testing") from exc
    finally:
        builtins.open = original_open
        BENCH.open_port, BENCH.section_a, BENCH.send_cmd = originals
    assert status != 0
    assert "mc_stop" in commands
    assert fake_serial.closed


if __name__ == "__main__":
    test_calibration_parser_requires_success_and_valid_offset()
    test_section_a_refuses_current_command_after_calibration_failure()
    test_main_returns_failure_and_always_stops_and_closes()
    print("stage5 bench tests passed")
