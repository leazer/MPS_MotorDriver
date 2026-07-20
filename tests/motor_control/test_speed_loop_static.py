from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEED_H = ROOT / "application" / "motor_control" / "speed_loop.h"
SPEED_C = ROOT / "application" / "motor_control" / "speed_loop.c"
ISR_H = ROOT / "application" / "motor_control" / "motor_control_isr.h"
ISR_C = ROOT / "application" / "motor_control" / "motor_control_isr.c"
SHELL_C = ROOT / "application" / "motor_shell.c"
APP_C = ROOT / "application" / "motor_app.c"
PARAMS_H = ROOT / "application" / "motor_control" / "motor_params.h"


def read(path):
    return path.read_text(encoding="utf-8")


def test_speed_loop_api_and_debug_snapshot_exist():
    header = read(SPEED_H)
    source = read(SPEED_C)

    for token in (
        "speed_loop_set_target_rad_s",
        "speed_loop_run",
        "speed_loop_reset",
        "speed_loop_get_target_rad_s",
        "speed_loop_get_measured_rad_s",
        "speed_loop_get_iq_ref_A",
    ):
        assert token in header
        assert token in source


def test_speed_mode_isr_api_and_branch_reuse_current_loop():
    header = read(ISR_H)
    source = read(ISR_C)

    for token in (
        "motor_control_isr_speed_start",
        "motor_control_isr_speed_stop",
        "motor_control_isr_speed_active",
    ):
        assert token in header
        assert token in source

    assert "case MOTOR_CONTROL_MODE_SPEED" in source
    assert "speed_loop_run(encoder_tracker_get_speed_rad_s())" in source
    assert "current_loop_set_targets(0.0f, iq_ref)" in source
    assert "Stage 6+ 实现, 暂输出 50%" not in source


def test_shell_exposes_mc_speed_and_debug_fields():
    shell = read(SHELL_C)

    assert "static void mc_speed" in shell
    assert "MSH_CMD_EXPORT(mc_speed" in shell
    assert "mc_speed <rpm_elec>" in shell
    assert "motor_control_isr_speed_start" in shell
    assert "motor_control_isr_speed_stop" in shell
    assert "spd       :" in shell


def test_speed_loop_initialized_from_motor_app():
    app = read(APP_C)

    assert '#include "speed_loop.h"' in app
    assert "speed_loop_init();" in app


def test_speed_mode_has_independent_half_amp_limit():
    params = read(PARAMS_H)
    lines = [line.split() for line in params.splitlines() if line.startswith("#define")]
    assert ["#define", "SPEED_IQ_LIMIT_A", "0.5f"] in lines
    assert ["#define", "PID_SPEED_INTEGRAL_LIMIT", "SPEED_IQ_LIMIT_A"] in lines
    assert ["#define", "PID_SPEED_OUT_LIMIT", "SPEED_IQ_LIMIT_A"] in lines


if __name__ == "__main__":
    test_speed_loop_api_and_debug_snapshot_exist()
    test_speed_mode_isr_api_and_branch_reuse_current_loop()
    test_shell_exposes_mc_speed_and_debug_fields()
    test_speed_loop_initialized_from_motor_app()
    test_speed_mode_has_independent_half_amp_limit()
    print("speed loop static tests passed")
