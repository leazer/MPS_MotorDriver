import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PINS = ROOT / "platform" / "at32m412" / "board_motor_pins.h"
PWM = ROOT / "platform" / "at32m412" / "motor_pwm_at32m412.c"
BOARD_INIT = ROOT / "platform" / "at32m412" / "board_init_at32m412.c"


def read(path):
    assert path.exists(), f"missing {path}"
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


def test_v2_swaps_pwma_pwmc_phase_outputs():
    pins = read(PINS)
    pwm = read(PWM)
    setter = function_body(pwm, "motor_pwm_at32m412_set_duty_ticks")
    apply_duty = function_body(pwm, "pwm_apply_duty_ticks")

    assert re.search(r"#define\s+PWM_PHASE_U_TMR_CHANNEL\s+TMR_SELECT_CHANNEL_3\b", pins)
    assert re.search(r"#define\s+PWM_PHASE_V_TMR_CHANNEL\s+TMR_SELECT_CHANNEL_2\b", pins)
    assert re.search(r"#define\s+PWM_PHASE_W_TMR_CHANNEL\s+TMR_SELECT_CHANNEL_1\b", pins)
    assert re.search(r"\ba\s*=\s*pwm_clamp_duty\(phase_u\)", setter)
    assert re.search(r"\bb\s*=\s*pwm_clamp_duty\(phase_v\)", setter)
    assert re.search(r"\bc\s*=\s*pwm_clamp_duty\(phase_w\)", setter)
    assert "pwm_apply_duty_ticks(a, b, c)" in setter
    assert "tmr_channel_value_set(TMR1, PWM_PHASE_U_TMR_CHANNEL, a)" in apply_duty
    assert "tmr_channel_value_set(TMR1, PWM_PHASE_V_TMR_CHANNEL, b)" in apply_duty
    assert "tmr_channel_value_set(TMR1, PWM_PHASE_W_TMR_CHANNEL, c)" in apply_duty


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
