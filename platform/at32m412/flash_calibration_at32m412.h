#ifndef FLASH_CALIBRATION_AT32M412_H
#define FLASH_CALIBRATION_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "motor_calibration.h"

/* 从 FLASH 标定区读取 (spec §4.7.7)
 * 流程: memcpy 528 字节 -> 校验 magic/version -> 校验 CRC32
 * 全过返回 true 并填充 cal, 任一失败返回 false */
bool flash_calibration_read(motor_calibration_t *cal);

/* 擦除末页 + 写入标定数据 (spec §4.7.5 CAL_WRITE_FLASH)
 * 流程: flash_unlock -> sector_erase -> word_program×132 -> flash_lock -> 回读校验
 * 返回 true=写入并校验成功, false=擦除/写入/校验失败 */
bool flash_calibration_write(const motor_calibration_t *cal);

/* 擦除标定区 (finsh mc_cal_erase)
 * 仅擦除, 不写入. 下次开机 motor_calibration_load 会因 magic 错误置 FAULT_CAL_INVALID */
bool flash_calibration_erase(void);

/* 硬件 CRC32 计算 (AT32M412 CRC 外设, spec §4.7 提到有硬件 CRC)
 * 默认配置: poly 0x04C11DB7, 输入按 word 反转, 输出反转, init 0xFFFFFFFF
 *          (与标准 zlib/IEEE CRC-32 一致)
 *   data: 待校验数据 (无需 word 对齐, 内部按 byte 处理)
 *   len : 字节数
 * 返回 CRC32 值 */
uint32_t flash_calibration_crc32(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_CALIBRATION_AT32M412_H */
