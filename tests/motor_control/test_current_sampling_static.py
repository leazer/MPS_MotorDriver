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


def test_pwm_sample_tick_and_cycle_pairing():
    c = read(PWM_C)
    h = read(PWM_H)
    assert re.search(r"#define\s+PWM_ADC_TRIGGER_TICKS\s+\(TMR1_ARR\s*-\s*360u\)", c)
    assert "current_sample_tracker_stage_duty" in c
    assert "current_sample_tracker_stage_trigger" in c
    assert "current_sample_tracker_rearm_from_next" in c
    assert "current_sample_tracker_latch_update" in c
    assert c.index("current_sample_tracker_latch_update") < c.index("motor_control_isr_tick();")
    assert "motor_pwm_at32m412_get_sample_plan" in h
    assert "motor_pwm_at32m412_get_sample_plan" in c


if __name__ == "__main__":
    test_pwm_sample_tick_and_cycle_pairing()
    print("current sampling static tests passed")
