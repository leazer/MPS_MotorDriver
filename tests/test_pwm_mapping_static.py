import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PINS = ROOT / "platform" / "at32m412" / "board_motor_pins.h"
PWM = ROOT / "platform" / "at32m412" / "motor_pwm_at32m412.c"
BOARD_INIT = ROOT / "platform" / "at32m412" / "board_init_at32m412.c"


def read(path):
    assert path.exists(), f"missing {path}"
    return path.read_text(encoding="utf-8")


def test_v2_swaps_pwma_pwmc_phase_outputs():
    pins = read(PINS)
    pwm = read(PWM)

    assert re.search(r"#define\s+PWM_PHASE_U_TMR_CHANNEL\s+TMR_SELECT_CHANNEL_3\b", pins)
    assert re.search(r"#define\s+PWM_PHASE_V_TMR_CHANNEL\s+TMR_SELECT_CHANNEL_2\b", pins)
    assert re.search(r"#define\s+PWM_PHASE_W_TMR_CHANNEL\s+TMR_SELECT_CHANNEL_1\b", pins)
    assert "tmr_channel_value_set(TMR1, PWM_PHASE_U_TMR_CHANNEL, pwm_clamp_duty(phase_u))" in pwm
    assert "tmr_channel_value_set(TMR1, PWM_PHASE_V_TMR_CHANNEL, pwm_clamp_duty(phase_v))" in pwm
    assert "tmr_channel_value_set(TMR1, PWM_PHASE_W_TMR_CHANNEL, pwm_clamp_duty(phase_w))" in pwm


def test_v2_led_moves_to_pb8_without_gpioa_pin_merge():
    pins = read(PINS)
    board_init = read(BOARD_INIT)

    assert re.search(r"#define\s+LED_GPIO_PORT\s+GPIOB\b", pins)
    assert re.search(r"#define\s+LED_PIN\s+GPIO_PINS_8\b", pins)
    assert "LED_PIN | SPI2_CS_PIN" not in board_init
    assert "gpio_init(LED_GPIO_PORT, &gpio_init_struct)" in board_init
    assert "gpio_init(SPI2_CS_GPIO_PORT, &gpio_init_struct)" in board_init


if __name__ == "__main__":
    test_v2_swaps_pwma_pwmc_phase_outputs()
    test_v2_led_moves_to_pb8_without_gpioa_pin_merge()
    print("pwm mapping static tests passed")
