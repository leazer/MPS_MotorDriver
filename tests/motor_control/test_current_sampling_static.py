from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
PWM_C = ROOT / "platform" / "at32m412" / "motor_pwm_at32m412.c"
PWM_H = ROOT / "platform" / "at32m412" / "motor_pwm_at32m412.h"
ISR_C = ROOT / "application" / "motor_control" / "motor_control_isr.c"
ISR_H = ROOT / "application" / "motor_control" / "motor_control_isr.h"
PARAMS = ROOT / "application" / "motor_control" / "motor_params.h"
FAULT_H = ROOT / "application" / "motor_control" / "fault_manager.h"
SHELL = ROOT / "application" / "motor_shell.c"
CMAKE = ROOT / "CMakeLists.txt"


def read(path):
    return path.read_text(encoding="utf-8")


def function_body(source, name):
    match = re.search(rf"\b{re.escape(name)}\s*\([^;]*?\)\s*\{{", source, re.DOTALL)
    assert match, f"missing function {name}"
    start = match.end() - 1
    depth = 0
    for index in range(start, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start + 1:index]
    raise AssertionError(f"unterminated function {name}")


def test_pwm_stages_hardware_and_tracker_from_identical_values():
    c = read(PWM_C)
    init = function_body(c, "motor_pwm_at32m412_init")
    apply_duty = function_body(c, "pwm_apply_duty_ticks")
    apply_trigger = function_body(c, "pwm_apply_adc_trigger_ticks")

    assert re.search(r"#define\s+PWM_ADC_TRIGGER_TICKS\s+\(TMR1_ARR\s*-\s*360u\)", c)
    assert "current_sample_tracker_init" in init
    assert "tmr_channel_value_set(TMR1, PWM_PHASE_U_TMR_CHANNEL, a)" in apply_duty
    assert "tmr_channel_value_set(TMR1, PWM_PHASE_V_TMR_CHANNEL, b)" in apply_duty
    assert "tmr_channel_value_set(TMR1, PWM_PHASE_W_TMR_CHANNEL, c)" in apply_duty
    assert "current_sample_tracker_stage_duty(&s_sample_tracker, a, b, c)" in apply_duty
    assert "tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, ticks)" in apply_trigger
    assert "current_sample_tracker_stage_trigger(&s_sample_tracker, ticks)" in apply_trigger


def test_thread_updates_are_deferred_to_the_update_handler():
    c = read(PWM_C)
    isr = read(ISR_C)
    shell = read(SHELL)
    defer = function_body(c, "pwm_update_must_be_deferred")
    set_duty = function_body(c, "motor_pwm_at32m412_set_duty_ticks")
    set_trigger = function_body(c, "motor_pwm_at32m412_set_adc_trigger_ticks")
    queue_duty = function_body(c, "pwm_queue_duty_request")
    queue_trigger = function_body(c, "pwm_queue_trigger_request")
    apply_pending = function_body(c, "pwm_apply_pending_thread_updates")
    control_tick = function_body(isr, "motor_control_isr_tick")

    assert "s_ovf_irq_enabled" in defer
    assert "!s_in_update_handler" in defer
    assert "pwm_queue_duty_request" in set_duty
    assert "pwm_queue_trigger_request" in set_trigger
    assert queue_duty.count("__DMB()") >= 2
    assert queue_trigger.count("__DMB()") >= 2
    assert "pwm_apply_duty_ticks" in apply_pending
    assert "pwm_apply_adc_trigger_ticks" in apply_pending
    assert "motor_pwm_at32m412_set_duty_ticks" in control_tick
    assert "motor_pwm_at32m412_set_adc_trigger_ticks" in shell
    assert "ADC trigger request=" in shell


def test_update_handler_latches_sample_before_control_and_publishes_requests_after():
    handler = function_body(read(PWM_C), "TMR1_OVF_TMR10_IRQHandler")

    latch = handler.index("current_sample_tracker_latch_update(&s_sample_tracker)")
    tick = handler.index("motor_control_isr_tick();")
    publish = handler.index("pwm_apply_pending_thread_updates();")
    assert handler.index("s_in_update_handler = true") < latch
    assert latch < tick < publish
    assert publish < handler.index("s_in_update_handler = false")


def test_sample_plan_getter_reads_tracker_not_timer_registers():
    c = read(PWM_C)
    h = read(PWM_H)
    getter = function_body(c, "motor_pwm_at32m412_get_sample_plan")

    assert "motor_pwm_at32m412_get_sample_plan" in h
    assert "current_sample_tracker_get_sampled(&s_sample_tracker, out)" in getter
    assert "TMR1" not in getter
    assert "tmr_channel_value" not in getter


def test_limits_and_fatal_sample_fault():
    params = read(PARAMS)
    fault = read(FAULT_H)
    assert re.search(r"#define\s+CURRENT_SAMPLE_BLANKING_TICKS\s+180u", params)
    assert re.search(r"#define\s+CURRENT_SAMPLE_INVALID_LIMIT\s+8u", params)
    assert re.search(r"#define\s+IQ_OVERCURRENT_A\s+2\.0f", params)
    assert re.search(r"#define\s+IQ_MAX_A\s+1\.5f", params)
    assert re.search(r"#define\s+IQ_MAX_MA\s+1500\b", params)
    assert "FAULT_CURRENT_SAMPLE" in fault
    fatal = re.search(r"#define\s+FAULT_FATAL_MASK[\s\S]*?\n\n", fault).group(0)
    assert "FAULT_CURRENT_SAMPLE" in fatal


def test_isr_uses_reconstruction_before_clarke_and_freezes_invalid_frames():
    source = read(ISR_C)
    assert "motor_pwm_at32m412_get_sample_plan" in source
    assert "current_reconstruction_run" in source
    assert "current_sample_guard_step" in source
    assert source.index("current_reconstruction_run") < source.index("foc_clarke(")
    assert "sample.frame_valid" in source
    assert "s_held_vd_ref" in source and "s_held_vq_ref" in source
    assert "s_dbg_pi_freeze_count" in source
    assert "fault_manager_set(FAULT_CURRENT_SAMPLE)" in source
    assert "sample.valid_mask == CURRENT_PHASE_ALL_MASK" in source


def test_raw_currents_cannot_bypass_reconstruction():
    source = read(ISR_C)
    assert "foc_clarke(sample.ia, sample.ib, sample.ic" in source
    assert "foc_clarke(ia, ib, ic" not in source


def test_guard_trips_latch_fault_state_before_fatal_branch():
    source = read(ISR_C)
    tick = function_body(source, "motor_control_isr_tick")
    fatal_branch = tick.index("fault_manager_any_fatal()")
    overcurrent_trip = tick.index("fault_manager_set(FAULT_OVERCURRENT)")
    invalid_trip = tick.index("fault_manager_set(FAULT_CURRENT_SAMPLE)")
    assert "mc->state = MOTOR_CONTROL_STATE_FAULT" in tick[overcurrent_trip:invalid_trip]
    assert "mc->state = MOTOR_CONTROL_STATE_FAULT" in tick[invalid_trip:fatal_branch]


if __name__ == "__main__":
    test_pwm_stages_hardware_and_tracker_from_identical_values()
    test_thread_updates_are_deferred_to_the_update_handler()
    test_update_handler_latches_sample_before_control_and_publishes_requests_after()
    test_sample_plan_getter_reads_tracker_not_timer_registers()
    test_limits_and_fatal_sample_fault()
    test_isr_uses_reconstruction_before_clarke_and_freezes_invalid_frames()
    test_raw_currents_cannot_bypass_reconstruction()
    test_guard_trips_latch_fault_state_before_fatal_branch()
    print("current sampling static tests passed")
