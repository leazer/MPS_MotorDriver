from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
ISR_C = ROOT / "application" / "motor_control" / "motor_control_isr.c"
SHELL_C = ROOT / "application" / "motor_shell.c"
CURRENT_SENSE_C = ROOT / "platform" / "at32m412" / "current_sense_at32m412.c"


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


def test_all_start_apis_reject_live_mode_switch_before_mutation():
    source = read(ISR_C)
    starts = (
        "motor_control_isr_open_loop_start",
        "motor_control_isr_align_start",
        "motor_control_isr_current_start",
        "motor_control_isr_speed_start",
    )
    mutations = (
        "current_loop_reset", "speed_loop_reset", "encoder_tracker_reset",
        "current_sampling_runtime_reset", "motor_pwm_at32m412_enable_output",
        "motor_pwm_at32m412_enable_ovf_irq", "mc->mode", "mc->state = ",
    )
    for name in starts:
        body = function_body(source, name)
        assert "mc->state == MOTOR_CONTROL_STATE_ENABLED" in body, \
            f"{name} does not reject a live mode switch"
        guard = body.index("mc->state == MOTOR_CONTROL_STATE_ENABLED")
        assert "return -3;" in body[guard:]
        for mutation in mutations:
            if mutation in body:
                assert guard < body.index(mutation), f"{name} mutates via {mutation} before live guard"


def test_runtime_configuration_setters_ignore_live_loop_changes():
    source = read(ISR_C)
    setters = {
        "motor_control_isr_open_loop_set_encoder_angle": "s_ol_use_enc = use_enc",
        "motor_control_isr_current_set_encoder_angle": "s_cur_use_enc = use_enc",
        "motor_control_isr_current_set_speed": "s_cur_speed_rad_s = rad_per_s",
    }
    for name, mutation in setters.items():
        body = function_body(source, name)
        assert "MOTOR_CONTROL_STATE_ENABLED" in body, \
            f"{name} has no live-loop guard"
        guard = body.index("MOTOR_CONTROL_STATE_ENABLED")
        early_return = body.index("return;", guard)
        write = body.index(mutation)
        assert guard < early_return < write, \
            f"{name} can mutate active-loop runtime configuration"


def test_shell_prechecks_running_state_before_open_and_current_configuration():
    source = read(SHELL_C)
    cases = {
        "mc_open": "motor_control_isr_open_loop_set_encoder_angle",
        "mc_cur": "motor_control_isr_current_set_encoder_angle",
    }
    for name, setter in cases.items():
        body = function_body(source, name)
        assert "motor_shell_reject_if_running" in body
        assert body.index("motor_shell_reject_if_running") < body.index(setter), \
            f"{name} configures the loop before rejecting a live start"


def test_all_shell_start_commands_report_already_running_result():
    source = read(SHELL_C)
    for name in ("mc_open", "mc_cur", "mc_speed", "mc_align"):
        body = function_body(source, name)
        assert "-3" in body, f"{name} does not handle the already-running result"
        assert "already enabled/running" in body, \
            f"{name} lacks the authoritative running-state message"
        assert "mc_stop" in body, f"{name} does not tell the operator how to stop"


def test_mc_cal_rejects_every_active_mode_before_first_pwm_mutation():
    body = function_body(read(SHELL_C), "mc_cal")
    guard = body.index("motor_shell_reject_calibration_if_active")
    first_pwm_mutation = min(
        body.index("motor_pwm_at32m412_set_duty_ticks"),
        body.index("motor_pwm_at32m412_enable_output"),
        body.index("motor_pwm_at32m412_enable_ovf_irq"),
    )
    assert guard < first_pwm_mutation

    guard_body = function_body(read(SHELL_C), "motor_shell_reject_calibration_if_active")
    assert "MOTOR_CONTROL_STATE_DISABLED" in guard_body
    for active_api in (
        "motor_control_isr_open_loop_active",
        "motor_control_isr_align_active",
        "motor_control_isr_current_active",
        "motor_control_isr_speed_active",
    ):
        assert active_api in guard_body, f"calibration guard misses {active_api}"
    assert "mc_cal result: FAIL offset_valid=0" in guard_body
    assert "mc_stop" in guard_body


def test_offset_calibration_waits_for_each_distinct_preempt_completion():
    body = function_body(read(CURRENT_SENSE_C), "current_sense_at32m412_calibrate_offset")
    loop = body.index("for (i = 0; i < OFFSET_SAMPLE_COUNT; i++)")
    assert "adc_flag_clear(ADC2, ADC_PCCE_FLAG)" in body[loop:]
    clear = body.index("adc_flag_clear(ADC2, ADC_PCCE_FLAG)", loop)
    assert "adc_flag_get(ADC2, ADC_PCCE_FLAG)" in body[clear:]
    wait = body.index("adc_flag_get(ADC2, ADC_PCCE_FLAG)", clear)
    read_raw = body.index("current_sense_at32m412_read_raw", wait)
    assert loop < clear < wait < read_raw
    assert "OFFSET_SEQUENCE_TIMEOUT" in body
    assert "return false;" in body[wait:read_raw]


def test_offset_calibration_only_commits_valid_complete_average():
    body = function_body(read(CURRENT_SENSE_C), "current_sense_at32m412_calibrate_offset")
    assert "candidate_a" in body and "candidate_b" in body and "candidate_c" in body, \
        "calibration must validate candidate offsets before committing them"
    commit = body.index("s_offset_a = candidate_a")
    validity_check = body.index("OFFSET_VALID_WINDOW_LSB")
    assert validity_check < commit
    assert body.index("s_offset_valid = true", commit) > commit


if __name__ == "__main__":
    test_all_start_apis_reject_live_mode_switch_before_mutation()
    test_runtime_configuration_setters_ignore_live_loop_changes()
    test_shell_prechecks_running_state_before_open_and_current_configuration()
    test_all_shell_start_commands_report_already_running_result()
    test_mc_cal_rejects_every_active_mode_before_first_pwm_mutation()
    test_offset_calibration_waits_for_each_distinct_preempt_completion()
    test_offset_calibration_only_commits_valid_complete_average()
    print("final fix static tests passed")
