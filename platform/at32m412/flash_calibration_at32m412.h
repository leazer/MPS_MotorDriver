#ifndef FLASH_CALIBRATION_AT32M412_H
#define FLASH_CALIBRATION_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "motor_calibration.h"

/* 从 FLASH 标定区读取 (spec §4.7.7) */
bool flash_calibration_read(motor_calibration_t *cal);

/* 擦除末页 + 写入标定数据 (spec §4.7.5 CAL_WRITE_FLASH) */
bool flash_calibration_write(const motor_calibration_t *cal);

/* 擦除标定区 (finsh mc_cal_erase) */
bool flash_calibration_erase(void);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_CALIBRATION_AT32M412_H */
