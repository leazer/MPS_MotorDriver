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
} motor_calibration_t;                    /* 总大小 532 字节 (spec §4.7.3 标注 528 有误) */

/* 编译期断言: 结构体必须能放入 1KB FLASH 标定区.
 * 实际布局: magic(4) + version(1) + reserved[3](3) + timestamp(4) +
 *           table[256](512) + mech_zero(2) + pole_pairs(1) + reserved2(1) +
 *           crc32(4) = 532 字节. 产品代码用 sizeof 处理, 不硬编码. */
typedef char motor_cal_fits_flash[1 - 2*!(sizeof(motor_calibration_t) <= CAL_FLASH_SIZE)];

typedef enum {
    CAL_STATE_IDLE = 0,
    CAL_STATE_ZERO_ALIGN,
    CAL_STATE_SPIN_FWD,
    CAL_STATE_SPIN_REV,
    CAL_STATE_COMPUTE,
    CAL_STATE_WRITE_FLASH,
    CAL_STATE_DONE,
    CAL_STATE_ABORTED       /* 中止 (故障/用户停止) */
} cal_state_t;

/* ===== 开机加载 (spec §4.7.7) ===== */
/* 从 FLASH 读取标定, 校验 magic/version/CRC. 成功 -> g_cal_valid=true + 写入零点;
 * 失败 -> g_cal_valid=false + 置 FAULT_CAL_INVALID (告警级, 不阻止使能). */
void motor_calibration_load(void);
bool motor_calibration_is_valid(void);
const motor_calibration_t *motor_calibration_get(void);

/* ===== 标定触发 (CAN/finsh 入口, spec §4.7.8) ===== */
/* 仅置状态机为 CAL_ZERO_ALIGN, 实际推进在 motor_calibration_poll() 内.
 * 前置: 故障已清, 当前非 ENABLED (不能与开环/闭环并发). */
void motor_calibration_start(void);
cal_state_t motor_calibration_get_state(void);
uint8_t motor_calibration_get_progress(void);   /* 0..100 */

/* ===== ISR 内采集 (spec §4.7.5 CAL_SPIN_FWD/REV) =====
 * 在 FOC ISR (16kHz) 内调用, 仅在 CAL_SPIN_FWD/REV 状态下累加直方图.
 * 不调 RT-Thread API, 不阻塞. 采集满 CAL_SAMPLES_PER_DIRECTION 自动切下一状态. */
void motor_calibration_tick(void);

/* ===== 线程上下文状态机推进 (motor_app_run 内调用) =====
 * 处理 CAL_ZERO_ALIGN / CAL_COMPUTE / CAL_WRITE_FLASH / CAL_DONE / CAL_ABORTED.
 * 这些状态含阻塞操作 (rt_thread_mdelay / FLASH 写), 必须在线程上下文执行.
 * 内部按 CAL_POLL_INTERVAL_MS 节流, 避免占用 CPU. */
void motor_calibration_poll(void);

/* ===== 中止 (故障/用户停止, spec §4.7.5 中止规则) =====
 * 停电机, 状态回 IDLE, 旧标定保留 (FLASH 不写).
 * 可在任意状态下调用, 幂等. */
void motor_calibration_abort(void);

/* ===== 调试/验收接口 ===== */
int16_t motor_calibration_get_max_residual(void);          /* 残差最大绝对值, 0.001° 单位 */
const int16_t *motor_calibration_get_table(void);          /* 当前 RAM 表 (可能为空) */
uint16_t motor_calibration_get_zero(void);                 /* mech_zero_raw */
void motor_calibration_set_zero(uint16_t raw);             /* 手动设零点 (mc_zero) */

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CALIBRATION_H */
