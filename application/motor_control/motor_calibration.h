#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "motor_params.h"

typedef struct {
    uint32_t magic;
    uint8_t  version;
    uint8_t  reserved[3];
    uint32_t timestamp_ms;
    int16_t  table[CAL_TABLE_POINTS];
    uint16_t mech_zero_raw;
    uint8_t  pole_pairs;
    uint8_t  reserved2;
    uint32_t crc32;
} motor_calibration_t;

typedef enum {
    CAL_STATE_IDLE = 0,
    CAL_STATE_ZERO_ALIGN,
    CAL_STATE_SPIN_FWD,
    CAL_STATE_SPIN_REV,
    CAL_STATE_COMPUTE,
    CAL_STATE_WRITE_FLASH,
    CAL_STATE_DONE,
} cal_state_t;

/* 开机加载, 失败时 g_cal_valid=false 且置 FAULT_CAL_INVALID */
void motor_calibration_load(void);
bool motor_calibration_is_valid(void);
const motor_calibration_t *motor_calibration_get(void);

/* 触发标定 (CAN/finsh 入口) */
void motor_calibration_start(void);
cal_state_t motor_calibration_get_state(void);
uint8_t motor_calibration_get_progress(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CALIBRATION_H */
