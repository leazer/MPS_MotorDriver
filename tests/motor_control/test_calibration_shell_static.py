from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
PARAMS = ROOT / "application" / "motor_control" / "motor_params.h"
SHELL = ROOT / "application" / "motor_shell.c"


def _read(path):
    return path.read_text(encoding="utf-8")


def _define_int(source, name):
    match = re.search(rf"#define\s+{name}\s+([0-9]+)u?", source)
    assert match, f"{name} define not found"
    return int(match.group(1))


def test_fast_calibration_profile_is_2_mechanical_turns_at_200_electrical_rpm():
    source = _read(PARAMS)

    assert _define_int(source, "CAL_MECH_TURNS_PER_DIRECTION") == 2
    assert _define_int(source, "CAL_SPIN_SPEED_RPM") == 200
    assert _define_int(source, "CAL_SPIN_DURATION_MS") == 4200
    assert _define_int(source, "CAL_SPIN_TIMEOUT_MS") == 8000


def test_legacy_mc_calibration_shell_commands_are_not_exported():
    source = _read(SHELL)

    for command in ("mc_calibrate", "mc_cal_status", "mc_cal_dump", "mc_zero"):
        assert f"MSH_CMD_EXPORT({command}," not in source


def test_encoder_shell_exports_include_unified_calibration_commands():
    source = _read(SHELL)

    for command in (
        "enc_zero",
        "enc_cal_start",
        "enc_cal_stop",
        "enc_cal_status",
        "enc_cal_dump",
        "enc_cal_erase",
    ):
        assert f"MSH_CMD_EXPORT({command}," in source
