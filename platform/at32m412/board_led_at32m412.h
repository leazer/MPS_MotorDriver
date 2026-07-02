#ifndef BOARD_LED_AT32M412_H
#define BOARD_LED_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "board_motor_pins.h"
#include "at32m412_416.h"

static inline void board_led_at32m412_on(void)
{
    gpio_bits_reset(LED_GPIO_PORT, LED_PIN);
}

static inline void board_led_at32m412_off(void)
{
    gpio_bits_set(LED_GPIO_PORT, LED_PIN);
}

static inline void board_led_at32m412_toggle(void)
{
    gpio_bits_toggle(LED_GPIO_PORT, LED_PIN);
}

static inline void board_led_at32m412_set(bool on)
{
    if (on) {
        board_led_at32m412_on();
    } else {
        board_led_at32m412_off();
    }
}

#ifdef __cplusplus
}
#endif

#endif
