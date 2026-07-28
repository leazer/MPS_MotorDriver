from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
PARAMS_H = ROOT / "application" / "motor_control" / "motor_params.h"


def test_current_loop_uses_half_bus_voltage_limit():
    source = PARAMS_H.read_text(encoding="utf-8")

    assert re.search(r"#define\s+PID_CURRENT_INTEGRAL_LIMIT\s+6\.0f\b", source)
    assert re.search(r"#define\s+PID_CURRENT_OUT_LIMIT\s+6\.0f\b", source)


def test_current_debug_reports_window_average():
    isr_h = (ROOT / "application" / "motor_control" / "motor_control_isr.h").read_text(encoding="utf-8")
    isr_c = (ROOT / "application" / "motor_control" / "motor_control_isr.c").read_text(encoding="utf-8")
    shell = (ROOT / "application" / "motor_shell.c").read_text(encoding="utf-8")

    assert "CURRENT_AVG_WINDOW_TICKS 256u" in isr_c
    assert "id_avg_ma" in isr_h
    assert "iq_avg_ma" in isr_h
    assert "current_debug_accumulate_average" in isr_c
    assert "current_debug_reset_average()" in isr_c
    assert "cur_avg" in shell


def test_pwm_adc_trigger_can_be_swept_from_shell():
    pwm_h = (ROOT / "platform" / "at32m412" / "motor_pwm_at32m412.h").read_text(encoding="utf-8")
    pwm_c = (ROOT / "platform" / "at32m412" / "motor_pwm_at32m412.c").read_text(encoding="utf-8")
    shell = (ROOT / "application" / "motor_shell.c").read_text(encoding="utf-8")

    assert "motor_pwm_at32m412_set_adc_trigger_ticks" in pwm_h
    assert "TMR_SELECT_CHANNEL_4" in pwm_c
    assert "pwm_adc_trig" in shell


def test_pwm_adc_trigger_default_matches_bench_window():
    pwm_c = (ROOT / "platform" / "at32m412" / "motor_pwm_at32m412.c").read_text(encoding="utf-8")

    assert "#define PWM_ADC_TRIGGER_TICKS (TMR1_ARR - 360u)" in pwm_c
    assert "tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, PWM_ADC_TRIGGER_TICKS)" in pwm_c


if __name__ == "__main__":
    test_current_loop_uses_half_bus_voltage_limit()
    test_current_debug_reports_window_average()
    test_pwm_adc_trigger_can_be_swept_from_shell()
    test_pwm_adc_trigger_default_matches_bench_window()
    print("current loop tuning static tests passed")
