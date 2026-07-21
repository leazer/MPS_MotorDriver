#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ===== ADC 通用 ===== */
#define ADC_VREF_VOLTS                  3.3f
#define ADC_BITS                        12u
#define ADC_FULL_SCALE                  4096u
#define ADC_LSB_VOLTS                   (ADC_VREF_VOLTS / (float)ADC_FULL_SCALE)

/* ===== MP6540H 电流采样 (spec §4.3.2) ===== */
/* 硬件: 4.7k 上拉到 3.3V + 4.7k 下拉到 GND */
#define MP6540H_VREF_VOLTS              1.65f
#define MP6540H_RTERM_OHMS              2350.0f
#define MP6540H_MIRROR_RATIO_TYP        (1.0f / 9200.0f)
#define MP6540H_VSO_PER_AMP_TYP         0.2554f             /* V/A, typ */
#define CURRENT_ZERO_OFFSET_LSB         2048u                /* 12-bit half-scale */
#define CURRENT_GAIN_DEFAULT_A_PER_LSB  (ADC_LSB_VOLTS / MP6540H_VSO_PER_AMP_TYP)
                                                          /* ~3.16e-3 A/LSB typ */
#define IQ_OVERCURRENT_A                2.0f
#define OVERCURRENT_DEBOUNCE_TICKS      4u    /* 0.25ms @16kHz, 防单拍 ADC 毛刺误触发, 真实过流仍快速保护 */
#define CURRENT_SAMPLE_BLANKING_TICKS   180u
#define CURRENT_SAMPLE_INVALID_LIMIT    8u
#define IMBALANCE_THRESHOLD_A           1.5f
#define IMBALANCE_DEBOUNCE_TICKS        100u  /* 连续 N tick 不平衡才触发故障 (防 ADC 开关噪声毛刺爆发, Stage 5 调试发现).
                                              * 100 tick = 6.25ms, 真实硬件故障会持续, 毛刺爆发不会持续这么久. */

/* ===== VBUS 母线电压 (spec §4.3.6, 分压比 1/6) ===== */
#define VBUS_DIVIDER_RATIO              6.0f
#define VBUS_VOLTS_PER_LSB              (ADC_LSB_VOLTS * VBUS_DIVIDER_RATIO)
                                                          /* 4.834 mV/LSB, 满量程 19.8V */
#define VBUS_UNDERVOLTAGE_THRESHOLD_V   8.0f
#define VBUS_OVERVOLTAGE_THRESHOLD_V    18.0f

/* ===== 电机参数 (spec §4.5.1) ===== */
#define MOTOR_POLE_PAIRS                7u    /* 2808 BLDC 默认, 实测后修正 */
/* V2 台架实测: 正电角度斜坡时 MA600A raw 递减, 控制坐标需反向归一化. */
#define MOTOR_ENCODER_DIRECTION         (-1)

/* ===== CAN 协议 (spec §5.1) ===== */
#define MOTOR_NODE_ID                   0x01u
#define CAN_BITRATE                     500000u

/* ===== 旁轴标定 (spec §4.7.3 / §4.7.4) ===== */
#define CAL_FLASH_ADDR                  0x0801FC00u
#define CAL_FLASH_SIZE                  1024u
#define CAL_MAGIC                       0x304C4143u   /* 'CAL0' little-endian */
#define CAL_VERSION                     1u
#define CAL_TABLE_POINTS                256u
#define CAL_HIST_BINS                   256u   /* = CAL_TABLE_POINTS, 直方图分箱数 */
/* CRC 覆盖范围: table(512) + mech_zero_raw(2) + pole_pairs(1) + reserved2(1) = 516 字节 */
#define CAL_CRC_PAYLOAD_SIZE            (CAL_TABLE_POINTS * 2u + 2u + 1u + 1u)
/* 标定旋转: 以机械圈为单位, 需覆盖编码器机械全范围 (0..65535) 至少 1 圈,
 * 取 2 圈缩短标定时间, 质量由 covered_bins/max_residual 门限兜底.
 * 开环电角度斜坡驱动, 电角度圈数 = 机械圈 × 极对数.
 * 注: 早期 CAL_TURNS_PER_DIRECTION=5 被当电角度圈用, 5电圈=0.71机械圈, 覆盖不足致标定失败. */
#define CAL_MECH_TURNS_PER_DIRECTION   2u
#define CAL_TURNS_PER_DIRECTION        (CAL_MECH_TURNS_PER_DIRECTION * MOTOR_POLE_PAIRS)  /* 电角度圈数 */
#define CAL_SPIN_SPEED_RPM             200     /* 电角度 rpm (开环斜坡速度), 200rpm=3.33圈/秒 */
/* 状态切换按旋转时长判断 (非样本数): 14电圈@200rpm=4.2s/方向.
 * 早期按 CAL_SAMPLES_PER_DIRECTION 切状态, 20480@16kHz=1.28s采满, 采样远快于旋转,
 * 采满时电机几乎没转. 改为按时间切状态后, 采样持续填直方图, RAM 累加器 int32 不溢出. */
#define CAL_SPIN_DURATION_MS           4200u   /* 每方向旋转时长 (14电圈@200rpm=4.2s) */
#define CAL_SPIN_TIMEOUT_MS            8000u   /* 每方向 8s 超时 (4.2s+余量) */
#define CAL_SAMPLES_PER_DIRECTION      20480u  /* 保留供参考 (进度估算已改用时间) */
#define CAL_MAX_RESIDUAL_MDEG           1000     /* 验收: 残差峰峰 < 1° (0.001° 为单位) */

/* 直方图累加用 int32, 单位为 raw16 LSB (65536 = 360°).
 * 每箱累加 (raw - bin_base), bin_base = idx * 256. 单箱范围 ±256 LSB. */
#define CAL_HIST_VALUE_MAX              32767   /* int16 限幅, 0.001° 单位 (±32.767°) */

/* 标定状态机轮询间隔 (motor_app_run 内, 避免占用 CPU) */
#define CAL_POLL_INTERVAL_MS            5u

/* ===== ALIGN 零点对齐 (spec §4.5.3) ===== */
/* Vd_align 由 ZERO_ALIGN_CURRENT_A × 估算相电阻.
 * 2808 BLDC 相电阻约 1Ω, 1A × 1Ω = 1V. 台架实测后修正. */
#define ALIGN_VD_VOLTS                  1.5f
#define ALIGN_VD_MAX_VOLTS              (VBUS_OVERVOLTAGE_THRESHOLD_V)   /* 18V 上限 */
/* ALIGN 总持续 500ms, 前 400ms 稳定, 后 100ms 采样平均作零点.
 * 采样窗口转为 tick 数: 100ms × 16kHz = 1600 ticks */
#define ALIGN_HOLD_MS                   (ZERO_ALIGN_HOLD_MS)
#define ALIGN_SETTLE_MS                 (ZERO_ALIGN_HOLD_MS - ZERO_ALIGN_SAMPLE_WINDOW_MS)  /* 400ms */
#define ALIGN_SAMPLE_TICKS              (ZERO_ALIGN_SAMPLE_WINDOW_MS * PWM_FREQUENCY_HZ / 1000u)

/* ===== PID 参数初值 (spec §4.4) ===== */
/* 电流环 */
#define PID_ID_KP                       0.5f
#define PID_ID_KI                       100.0f
#define PID_IQ_KP                       0.5f
#define PID_IQ_KI                       100.0f
#define PID_CURRENT_INTEGRAL_LIMIT      2.0f   /* 台架保守限幅: 避免低电流调试时进入占空比饱和区 */
#define PID_CURRENT_OUT_LIMIT           2.0f   /* 12V 母线下先限制到 ±2V, 待采样/零点稳定后再上调 */
#define IQ_MAX_A                        1.5f          /* 电流环目标上限, < 过流 2.0A 留 0.5A 余量 */
#define IQ_MAX_MA                       1500          /* mA, shell 层用 */
#define CURRENT_RAMP_DEFAULT_RPM        300.0f        /* ramp 模式默认角速度 (电角度 rpm), 调试用 */

/* 速度环 */
#define PID_SPEED_KP                    0.01f
#define PID_SPEED_KP_BRAKE              0.04f
#define PID_SPEED_KI                    0.01f
#define SPEED_IQ_FRICTION_A             0.02f
#define SPEED_IQ_LIMIT_A                0.5f
#define PID_SPEED_INTEGRAL_LIMIT        SPEED_IQ_LIMIT_A
#define PID_SPEED_OUT_LIMIT             SPEED_IQ_LIMIT_A
#define RPM_MAX                         3000

/* 位置环 */
#define PID_POSITION_KP                 5.0f
#define POSITION_SPEED_LIMIT_RPM_ELEC   200.0f
#define POSITION_SPEED_LIMIT_ELEC_RAD_S (POSITION_SPEED_LIMIT_RPM_ELEC * 6.28318530718f / 60.0f)
#define POSITION_MAX_VELOCITY_MDEG_S    60000
#define POSITION_MAX_ERROR_MDEG         30000
#define POSITION_EXTRAPOLATION_LIMIT_MS 20u
#define POSITION_LOOP_DIV               16u

/* ===== 零点标定 (spec §4.5.3) ===== */
#define ZERO_ALIGN_CURRENT_A            1.0f
#define ZERO_ALIGN_HOLD_MS              500u
#define ZERO_ALIGN_SAMPLE_WINDOW_MS     100u

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PARAMS_H */
