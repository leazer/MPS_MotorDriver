from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
ENCODER_SERVICE_H = ROOT / "application" / "motor_control" / "encoder_service.h"
ENCODER_TRACKER_H = ROOT / "application" / "motor_control" / "encoder_tracker.h"
ENCODER_TRACKER_C = ROOT / "application" / "motor_control" / "encoder_tracker.c"


def _read(path):
    return path.read_text(encoding="utf-8")


def test_encoder_snapshot_exposes_corrected_position_and_window_speed():
    source = _read(ENCODER_SERVICE_H)

    for field in (
        "corrected_raw16",
        "corrected_unwrapped",
        "speed_mech_mrad_s",
        "speed_elec_mrad_s",
    ):
        assert field in source


def test_encoder_tracker_uses_service_speed_and_has_reset_api():
    header = _read(ENCODER_TRACKER_H)
    impl = _read(ENCODER_TRACKER_C)

    assert "void encoder_tracker_reset(void);" in header
    assert "encoder_service_get_speed_electrical_rad_s()" in impl
    assert "TRACKER_KP" not in impl
    assert "TRACKER_KI" not in impl


def test_raw16_to_mrad_scale_is_milliradians_not_microradians():
    source = (ROOT / "application" / "motor_control" / "encoder_service.c").read_text(encoding="utf-8")

    assert re.search(r"^#define\s+RAW16_TO_MRAD_NUM\s+6283\s*$", source, re.MULTILINE)
