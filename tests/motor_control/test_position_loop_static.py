from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "application" / "motor_control" / "position_loop.h"
SOURCE = ROOT / "application" / "motor_control" / "position_loop.c"
PARAMS = ROOT / "application" / "motor_control" / "motor_params.h"
CMAKE = ROOT / "CMakeLists.txt"
MDK = ROOT / "project" / "MDK_V5" / "MPS_MotorDriver.uvprojx"


def read(path):
    assert path.exists(), f"missing {path}"
    return path.read_text(encoding="utf-8")


def test_position_loop_public_contract():
    header = read(HEADER)
    for token in [
        "position_setpoint_t",
        "position_loop_snapshot_t",
        "position_loop_init",
        "position_loop_reset",
        "position_loop_set_origin",
        "position_loop_origin_valid",
        "position_loop_sensor_to_joint_mdeg",
        "position_loop_submit",
        "position_loop_run",
        "position_loop_get_iq_feedforward_A",
        "position_loop_get_snapshot",
    ]:
        assert token in header


def test_position_loop_has_coherent_publish_and_bounded_extrapolation():
    source = read(SOURCE)
    params = read(PARAMS)
    assert "s_publish_generation" in source
    assert "generation_before" in source
    assert "generation_after" in source
    assert "POSITION_EXTRAPOLATION_LIMIT_MS" in source
    assert "POSITION_MAX_ERROR_MDEG" in source
    assert "POSITION_MAX_VELOCITY_MDEG_S" in source
    assert "POSITION_SPEED_LIMIT_ELEC_RAD_S" in params
    assert "POSITION_LOOP_DIV" in params
    assert "POSITION_IQ_FRICTION_A" in params
    assert "POSITION_IQ_FRICTION_MOVING_A" in params
    assert "POSITION_IQ_FRICTION_ERROR_MDEG" in params


def test_snapshot_reader_retries_odd_generation_before_comparing_samples():
    source = read(SOURCE)
    reader = source.split("bool position_loop_get_snapshot", 1)[1]

    assert "for (;;)" in reader
    assert "if (generation_before == generation_after &&" in reader
    assert "return true;" in reader


def test_position_loop_uses_mechanical_units_and_pole_pairs_once():
    source = read(SOURCE)
    assert "180000.0f" in source
    assert "MOTOR_POLE_PAIRS" in source
    assert source.count("(float)MOTOR_POLE_PAIRS") == 1


def test_position_loop_is_in_both_target_builds():
    assert "application/motor_control/position_loop.c" in read(CMAKE)
    mdk = read(MDK)
    assert "<FileName>position_loop.c</FileName>" in mdk
    assert "..\\..\\application\\motor_control\\position_loop.c" in mdk


if __name__ == "__main__":
    test_position_loop_public_contract()
    test_position_loop_has_coherent_publish_and_bounded_extrapolation()
    test_snapshot_reader_retries_odd_generation_before_comparing_samples()
    test_position_loop_uses_mechanical_units_and_pole_pairs_once()
    test_position_loop_is_in_both_target_builds()
    print("position loop static tests passed")
