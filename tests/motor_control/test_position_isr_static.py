from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ISR_H = ROOT / "application" / "motor_control" / "motor_control_isr.h"
ISR_C = ROOT / "application" / "motor_control" / "motor_control_isr.c"
APP_C = ROOT / "application" / "motor_app.c"
FAULT_H = ROOT / "application" / "motor_control" / "fault_manager.h"


def read(path):
    assert path.exists(), f"missing {path}"
    return path.read_text(encoding="utf-8")


def function_body(source, signature, next_signature):
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def test_position_isr_public_api_exists():
    header = read(ISR_H)
    source = read(ISR_C)
    assert '#include "position_loop.h"' in header
    assert '#include "encoder_service.h"' in source
    for token in [
        "motor_control_isr_position_start",
        "motor_control_isr_position_submit",
        "motor_control_isr_position_stop",
        "motor_control_isr_position_active",
    ]:
        assert token in header


def test_position_branch_reuses_verified_speed_and_current_chain():
    source = read(ISR_C)
    start = source.index("case MOTOR_CONTROL_MODE_POSITION")
    end = source.index("default:", start)
    branch = source[start:end]
    for token in [
        "encoder_service_get_control_position_mdeg",
        "position_loop_run",
        "speed_loop_set_target_rad_s",
        "speed_loop_run",
        "position_loop_get_iq_feedforward_A",
        "SPEED_IQ_LIMIT_A",
        "current_loop_set_targets",
        "current_loop_run",
        "foc_ipark",
        "foc_svpwm_3phase_high_side",
        "current_fault_latch(mc, FAULT_POSITION_TRACKING)",
    ]:
        assert token in branch
    assert "Stage 7+" not in branch
    assert "暂输出 50%" not in branch


def test_start_submit_and_stop_have_distinct_lifecycle_semantics():
    source = read(ISR_C)
    start_body = function_body(
        source,
        "int motor_control_isr_position_start",
        "int motor_control_isr_position_submit",
    )
    submit_body = function_body(
        source,
        "int motor_control_isr_position_submit",
        "void motor_control_isr_position_stop",
    )
    stop_body = function_body(
        source,
        "void motor_control_isr_position_stop",
        "bool motor_control_isr_position_active",
    )
    for token in [
        "position_loop_origin_valid",
        "encoder_service_get_snapshot",
        "fault_manager_any_fatal",
        "position_loop_reset",
        "speed_loop_reset",
        "current_loop_reset",
        "position_loop_submit",
        "MOTOR_CONTROL_MODE_POSITION",
        "motor_pwm_at32m412_enable_output",
    ]:
        assert token in start_body
    assert "position_loop_submit" in submit_body
    assert "position_loop_reset" not in submit_body
    assert "speed_loop_reset" not in submit_body
    assert "current_loop_reset" not in submit_body
    for token in [
        "position_loop_reset",
        "speed_loop_reset",
        "current_loop_reset",
        "motor_pwm_at32m412_disable_ovf_irq",
        "motor_pwm_at32m412_disable_output",
        "MOTOR_CONTROL_STATE_DISABLED",
    ]:
        assert token in stop_body


def test_position_tracking_fault_is_fatal_and_loop_is_initialized():
    fault = read(FAULT_H)
    app = read(APP_C)
    assert "FAULT_POSITION_TRACKING" in fault
    fatal = fault[fault.index("#define FAULT_FATAL_MASK"):]
    assert "FAULT_POSITION_TRACKING" in fatal
    assert '#include "position_loop.h"' in app
    assert "position_loop_init();" in app


if __name__ == "__main__":
    test_position_isr_public_api_exists()
    test_position_branch_reuses_verified_speed_and_current_chain()
    test_start_submit_and_stop_have_distinct_lifecycle_semantics()
    test_position_tracking_fault_is_fatal_and_loop_is_initialized()
    print("position ISR static tests passed")
