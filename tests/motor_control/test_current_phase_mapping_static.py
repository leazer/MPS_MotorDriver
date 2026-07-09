from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
ISR_C = ROOT / "application" / "motor_control" / "motor_control_isr.c"
PINS_H = ROOT / "platform" / "at32m412" / "board_motor_pins.h"


def test_v2_pwm_swap_does_not_remap_current_feedback_in_isr():
    pins = PINS_H.read_text(encoding="utf-8")
    source = ISR_C.read_text(encoding="utf-8")

    assert "PWM_PHASE_U_TMR_CHANNEL  TMR_SELECT_CHANNEL_3" in pins
    assert "PWM_PHASE_W_TMR_CHANNEL  TMR_SELECT_CHANNEL_1" in pins
    assert "ia = current_sense_calc(ia_raw" in source
    assert "ib = current_sense_calc(ib_raw" in source
    assert "ic = current_sense_calc(ic_raw" in source
    assert "ia = i_phys_c;" not in source
    assert "ic = i_phys_a;" not in source


if __name__ == "__main__":
    test_v2_pwm_swap_does_not_remap_current_feedback_in_isr()
    print("current phase mapping static tests passed")
