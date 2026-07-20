import builtins
from contextlib import redirect_stdout
import importlib.util
import io
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BENCH_PATH = ROOT / "tests" / "stage6_bench.py"
SPEC = importlib.util.spec_from_file_location("stage6_bench_under_test", BENCH_PATH)
BENCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BENCH)


class FakeSerial:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class ByteSerial:
    def __init__(self, payload):
        self.payload = bytearray(payload)

    @property
    def in_waiting(self):
        return len(self.payload)

    def read(self, count):
        data = bytes(self.payload[:count])
        del self.payload[:count]
        return data


def test_parse_compact_speed_status():
    text = (
        "spdstat active=1 target=6283 cmd=6283 meas=6201 iqref=84 "
        "id=2 iq=82 invalid=7 streak=0 freeze=3 fault=0x00000000"
    )
    assert BENCH.parse_speed_status(text) == {
        "active": 1,
        "target": 6283,
        "cmd": 6283,
        "meas": 6201,
        "iqref": 84,
        "id": 2,
        "iq": 82,
        "invalid": 7,
        "streak": 0,
        "freeze": 3,
        "fault": 0,
    }
    assert BENCH.parse_speed_status("spdstat incomplete") is None


def test_serial_decode_drops_corrupt_bytes_without_console_replacement_chars():
    ser = ByteSerial(
        b"\xffspdstat active=1 target=1 cmd=1 meas=1 iqref=0 id=0 iq=0 "
        b"invalid=0 streak=0 freeze=0 fault=0x00000000\r\nmsh >"
    )
    text = BENCH.read_until_prompt(ser, timeout=0.01)
    assert "\ufffd" not in text
    assert BENCH.parse_speed_status(text)["meas"] == 1


def test_speed_status_retries_after_one_corrupt_reply():
    replies = iter(
        (
            "mc_speY5",
            "spdstat active=1 target=1 cmd=1 meas=1 iqref=0 id=0 iq=0 "
            "invalid=0 streak=0 freeze=0 fault=0x00000000",
        )
    )
    original = BENCH.send_cmd
    BENCH.send_cmd = lambda *_args, **_kwargs: next(replies)
    try:
        snapshot = BENCH.read_speed_status(FakeSerial(), attempts=2)
    finally:
        BENCH.send_cmd = original
    assert snapshot["active"] == 1
    assert snapshot["meas"] == 1


def test_shell_exports_compact_speed_status():
    shell = (ROOT / "application" / "motor_shell.c").read_text(encoding="utf-8")
    assert "static void mc_speed_status" in shell
    assert "MSH_CMD_EXPORT(mc_speed_status" in shell
    assert "spdstat active=" in shell


def test_final_safe_state_is_queried_and_asserted():
    commands = []
    responses = {
        "mc_stop": "all modes stopped",
        "mc_state": "state : 0\nmode : 2\nfault : 0x00000000",
        "fault": "fault = 0x00000000",
        "pwm_info": (
            "CCR1/2/3 : 2812 / 2812 / 2812\n"
            "CCR4     : 5264 (ADC trigger)\nEN(PB10) : 0"
        ),
        "mc_speed_status": (
            "spdstat active=0 target=0 cmd=0 meas=0 iqref=0 "
            "id=0 iq=0 invalid=0 streak=0 freeze=0 fault=0x00000000"
        ),
    }

    def fake_send_cmd(_ser, cmd, **_kwargs):
        commands.append(cmd)
        return responses[cmd]

    original = BENCH.send_cmd
    BENCH.send_cmd = fake_send_cmd
    try:
        summary = BENCH.verify_final_safe_state(FakeSerial())
    finally:
        BENCH.send_cmd = original

    assert commands == ["mc_stop", "mc_state", "fault", "pwm_info", "mc_speed_status"]
    assert "FINAL SAFE" in summary


def test_main_stops_and_closes_after_failure():
    fake_serial = FakeSerial()
    commands = []
    originals = (BENCH.open_port, BENCH.prepare_bench, BENCH.run_target, BENCH.send_cmd)
    original_open = builtins.open
    BENCH.open_port = lambda _port: fake_serial
    BENCH.prepare_bench = lambda _ser: None
    BENCH.run_target = lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("boom"))
    BENCH.send_cmd = lambda _ser, cmd, **_kwargs: commands.append(cmd) or ""
    builtins.open = lambda path, *args, **kwargs: (
        io.StringIO() if str(path).endswith("stage6_bench_log.txt")
        else original_open(path, *args, **kwargs)
    )
    try:
        with redirect_stdout(io.StringIO()):
            status = BENCH.main(["stage6_bench.py", "TEST"])
    finally:
        builtins.open = original_open
        BENCH.open_port, BENCH.prepare_bench, BENCH.run_target, BENCH.send_cmd = originals

    assert status != 0
    assert "mc_stop" in commands
    assert fake_serial.closed


if __name__ == "__main__":
    test_parse_compact_speed_status()
    test_serial_decode_drops_corrupt_bytes_without_console_replacement_chars()
    test_speed_status_retries_after_one_corrupt_reply()
    test_shell_exports_compact_speed_status()
    test_final_safe_state_is_queried_and_asserted()
    test_main_stops_and_closes_after_failure()
    print("stage6 bench tests passed")
