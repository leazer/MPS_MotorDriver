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


def test_final_safe_state_is_queried_asserted_and_logged():
    commands = []
    responses = {
        "mc_stop": "state=DISABLED",
        "mc_state": "state : 0",
        "fault": "fault = 0x00000000",
        "pwm_info": (
            "CCR1/2/3 : 2812 / 2812 / 2812\n"
            "CCR4     : 5264 (ADC trigger)\n"
            "EN(PB10) : 0"
        ),
        "mc_debug": (
            "cur_avg : id=0mA iq=0mA window=256\n"
            "sample : tick=5264 valid_mask=0x07 recon=1\n"
            "sample_count: invalid_total=0 invalid_consecutive=0 pi_freeze=0"
        ),
    }

    def fake_send_cmd(_ser, cmd, **_kwargs):
        commands.append(cmd)
        return responses[cmd]

    original = BENCH.send_cmd
    BENCH.send_cmd = fake_send_cmd
    log = []
    try:
        BENCH.verify_final_safe_state(FakeSerial(), log)
    finally:
        BENCH.send_cmd = original

    assert commands == ["mc_stop", "mc_state", "fault", "pwm_info", "mc_debug"]
    assert log == [
        "FINAL SAFE: state=DISABLED fault=0x00000000 en=0 "
        "ccr=2812/2812/2812 tick=5264 mask=0x07 "
        "invalid_total=0 invalid_consecutive=0 pi_freeze=0"
    ]


def test_full_quadrant_section_rejects_wrong_fixed_sample_tick():
    snapshots = iter([
        {"id_avg": 0, "iq_avg": 0, "invalid_total": 0,
         "invalid_consecutive": 0, "pi_freeze": 0,
         "sample_tick": 5264, "valid_mask": 0x07, "recon": 1},
        {"id_avg": 0, "iq_avg": 50, "invalid_total": 0,
         "invalid_consecutive": 0, "pi_freeze": 0,
         "sample_tick": 2500, "valid_mask": 0x07, "recon": 1},
        {"id_avg": 0, "iq_avg": 50, "invalid_total": 0,
         "invalid_consecutive": 0, "pi_freeze": 0,
         "sample_tick": 2500, "valid_mask": 0x07, "recon": 1},
        {"id_avg": 0, "iq_avg": 50, "invalid_total": 0,
         "invalid_consecutive": 0, "pi_freeze": 0,
         "sample_tick": 2500, "valid_mask": 0x07, "recon": 1},
    ])

    def fake_send_cmd(_ser, cmd, **_kwargs):
        if cmd == "enc_cal_status":
            return "valid : 1"
        if cmd.startswith("mc_cur "):
            return "current loop"
        return ""

    originals = (
        BENCH.CURRENT_TEST_POINTS_MA,
        BENCH.send_cmd,
        BENCH.read_current_snapshot,
        BENCH.read_fault_value,
        BENCH.time.sleep,
    )
    BENCH.CURRENT_TEST_POINTS_MA = (50,)
    BENCH.send_cmd = fake_send_cmd
    BENCH.read_current_snapshot = lambda _ser: next(snapshots)
    BENCH.read_fault_value = lambda _ser: 0
    BENCH.time.sleep = lambda _seconds: None
    try:
        try:
            BENCH.section_full_quadrant_current(FakeSerial(), [])
        except AssertionError as exc:
            assert "sample tick" in str(exc)
        else:
            raise AssertionError("section accepted a non-5264 sample tick")
    finally:
        (BENCH.CURRENT_TEST_POINTS_MA,
         BENCH.send_cmd,
         BENCH.read_current_snapshot,
         BENCH.read_fault_value,
         BENCH.time.sleep) = originals


if __name__ == "__main__":
    test_calibration_parser_requires_success_and_valid_offset()
    test_section_a_refuses_current_command_after_calibration_failure()
    test_main_returns_failure_and_always_stops_and_closes()
    test_final_safe_state_is_queried_asserted_and_logged()
    test_full_quadrant_section_rejects_wrong_fixed_sample_tick()
    print("stage5 bench tests passed")
