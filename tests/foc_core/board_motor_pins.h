#ifndef TEST_BOARD_MOTOR_PINS_H
#define TEST_BOARD_MOTOR_PINS_H

#include <stdint.h>

#define TMR1_ARR 5624u
#define PWM_DUTY_MAX ((uint16_t)(TMR1_ARR * 0.95f))

#endif
