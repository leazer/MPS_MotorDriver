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
#define IQ_OVERCURRENT_A                5.0f
#define IMBALANCE_THRESHOLD_A           1.5f

/* ===== VBUS 母线电压 (spec §4.3.6, 分压比 1/6) ===== */
#define VBUS_DIVIDER_RATIO              6.0f
#define VBUS_VOLTS_PER_LSB              (ADC_LSB_VOLTS * VBUS_DIVIDER_RATIO)
                                                          /* 4.834 mV/LSB, 满量程 19.8V */
#define VBUS_UNDERVOLTAGE_THRESHOLD_V   8.0f
#define VBUS_OVERVOLTAGE_THRESHOLD_V    18.0f

/* ===== 电机参数 (spec §4.5.1) ===== */
#define MOTOR_POLE_PAIRS                7u    /* 2808 BLDC 默认, 实测后修正 */

/* ===== CAN 协议 (spec §5.1) ===== */
#define MOTOR_NODE_ID                   0x01u
#define CAN_BITRATE                     500000u

/* ===== 旁轴标定 (spec §4.7.3 / §4.7.4) ===== */
#define CAL_FLASH_ADDR                  0x0801FC00u
#define CAL_FLASH_SIZE                  1024u
#define CAL_MAGIC                       0x304C4143u   /* 'CAL0' little-endian */
#define CAL_VERSION                     1u
#define CAL_TABLE_POINTS                256u
#define CAL_TURNS_PER_DIRECTION         5u
#define CAL_SPIN_SPEED_RPM              30
#define CAL_SAMPLES_PER_TURN            4096u

/* ===== PID 参数初值 (spec §4.4) ===== */
/* 电流环 */
#define PID_ID_KP                       0.5f
#define PID_ID_KI                       100.0f
#define PID_IQ_KP                       0.5f
#define PID_IQ_KI                       100.0f
#define PID_CURRENT_INTEGRAL_LIMIT      (VBUS_OVERVOLTAGE_THRESHOLD_V / 2.0f)
#define PID_CURRENT_OUT_LIMIT           (VBUS_OVERVOLTAGE_THRESHOLD_V / 2.0f)
#define IQ_MAX_A                        8.0f

/* 速度环 */
#define PID_SPEED_KP                    0.01f
#define PID_SPEED_KI                    0.5f
#define PID_SPEED_INTEGRAL_LIMIT        IQ_MAX_A
#define PID_SPEED_OUT_LIMIT             IQ_MAX_A
#define RPM_MAX                         3000

/* 位置环 */
#define PID_POSITION_KP                 5.0f
#define PID_POSITION_OUT_LIMIT          RPM_MAX

/* ===== 零点标定 (spec §4.5.3) ===== */
#define ZERO_ALIGN_CURRENT_A            1.0f
#define ZERO_ALIGN_HOLD_MS              500u
#define ZERO_ALIGN_SAMPLE_WINDOW_MS     100u

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PARAMS_H */
