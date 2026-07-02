#ifndef MOTOR_CONTROL_ISR_H
#define MOTOR_CONTROL_ISR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "motor_control.h"

/* FOC ISR 主体, 由 TMR1_OVF 中断调用 (spec §3.1).
 *
 * Stage 2: OPEN_LOOP 模式 (角度斜坡 + 固定 Vd -> IPark -> SVPWM).
 * Stage 4: ALIGN 模式 (固定 theta=0 锁定转子) + OPEN_LOOP 可选编码器电角度.
 *          编码器读取 + 标定采集钩子 (motor_calibration_tick).
 *          CURRENT/SPEED/POSITION 模式留空, Stage 5+ 填充.
 *          DISABLED/FAULT 状态下输出 50% 三相同电位 (不出力).
 *
 * ISR 不调用任何 RT-Thread API (spec §3.5 PRIMASK 临界区约束).
 */
void motor_control_isr_tick(void);

/* ===== 开环控制接口 (供 msh 命令调用, 非 ISR 上下文) =====
 *
 * OPEN_LOOP 模式行为 (spec §3.4 模式 0):
 *   - 跳过 Park / 电流环
 *   - Vd = open_loop_vd (用户设定, V), Vq = 0
 *   - theta_e 按固定角速度递增 (open_loop_speed_rad_per_s × dt)
 *     或使用编码器电角度 (若 set_encoder_angle(true))
 *   - IPark(Vd, 0, theta_e) -> SVPWM -> 写 CCR
 *
 * 安全: motor_control_state 必须为 ENABLED 且 fault 为 0 才真正输出.
 *       DISABLED/FAULT 状态下 ISR 强制输出 50% 三相同电位.
 */

/* 启动开环: 设置开环电压幅值与电角速度, 并切到 OPEN_LOOP + ENABLED.
 *   vd_volts        : d 轴目标电压 (V), 典型 0.5~3.0
 *   speed_rad_per_s : theta_e 递增角速度 (rad/s), 正=正转, 负=反转
 * 返回: 0=成功, -1=故障未清, -2=参数越界
 */
int motor_control_isr_open_loop_start(float vd_volts, float speed_rad_per_s);

/* 停止开环: 切回 DISABLED, 强制输出 50%, 不清故障标志 */
void motor_control_isr_open_loop_stop(void);

/* 是否正在开环运行 */
bool motor_control_isr_open_loop_active(void);

/* Stage 4: 选择 OPEN_LOOP 模式的电角度来源.
 *   use_enc=true  : theta_e = 编码器电角度 (验证编码器方向/零点)
 *   use_enc=false : theta_e = 斜坡递增 (Stage 2 默认行为, 向后兼容) */
void motor_control_isr_open_loop_set_encoder_angle(bool use_enc);

/* ===== ALIGN 零点对齐接口 (spec §4.5.3, Stage 4) =====
 *
 * ALIGN 模式: theta_e=0 (固定 d 轴 0°), Vd=vd_volts, Vq=0.
 * 转子被强制对齐到 d 轴 0° 方向, 持续期间采集 MA600A 角度作零点.
 * 前置: 故障已清. 调用后 MP6540H EN 拉高, TMR1_OVF IRQ 使能.
 */
int motor_control_isr_align_start(float vd_volts);
void motor_control_isr_align_stop(void);
bool motor_control_isr_align_active(void);

/* 读取 ALIGN 期间采集的零点角度 (最后 ALIGN_SAMPLE_WINDOW_TICKS 的平均, 16-bit).
 * 供 motor_calibration_poll 在 ALIGN 结束时读取. */
uint16_t motor_control_isr_get_align_angle(void);

/* ===== CURRENT 模式接口 (Stage 5, spec-stage5 §5.3) =====
 *
 * CURRENT 模式 (spec §3.4 模式 1):
 *   - Id 目标 = 0, Iq 目标 = iq_ref (用户/CAN 设置)
 *   - Clarke -> Park(theta) -> 电流环 PI -> IPark -> SVPWM
 *   - theta 来源: enc (编码器电角度, 需有效标定) 或 ramp (斜坡, 调试)
 *
 * 安全: 故障未清返回 -1. 启动时互斥清 OPEN_LOOP/ALIGN.
 */
int  motor_control_isr_current_start(float iq_ref_A);
void motor_control_isr_current_stop(void);
bool motor_control_isr_current_active(void);
void motor_control_isr_current_set_encoder_angle(bool use_enc);
void motor_control_isr_current_set_speed(float rad_per_s);

/* 获取最近一次 ISR 内部状态 (供 msh 观察, 非强一致, 仅调试用)
 * 注意: 所有浮点字段改用定点 (毫伏/毫弧度/毫度) 表示, 因 RT-Thread
 * Nano rt_kprintf 不支持 %f, shell 打印需用整数.
 */
typedef struct {
    int32_t theta_mrad;     /* 当前电角度 (毫弧度, 0..6283) */
    int32_t v_alpha_mv;     /* IPark 输出 (毫伏) */
    int32_t v_beta_mv;
    uint16_t ta;            /* SVPWM 输出 CCR */
    uint16_t tb;
    uint16_t tc;
    uint32_t tick_count;    /* ISR 累计 tick (16kHz) */
    uint32_t ol_branch_hits;/* OPEN_LOOP 分支命中计数 (诊断用) */
    uint32_t fault_hits;    /* fault 分支命中计数 */
    uint32_t disabled_hits; /* disabled 分支命中计数 */
    /* Stage 3: 电流/电压采样快照 */
    int32_t ia_ma;          /* 三相电流 (毫安) */
    int32_t ib_ma;
    int32_t ic_ma;
    int32_t vbus_mv;        /* 母线电压 (毫伏) */
    uint16_t ia_raw;        /* ADC 原始值 */
    uint16_t ib_raw;
    uint16_t ic_raw;
    uint32_t oc_hits;       /* 过流保护命中计数 */
    uint32_t imbal_hits;    /* 电流不平衡命中计数 */
    /* Stage 4: 编码器 + ALIGN + 标定快照 */
    uint16_t enc_raw;       /* 编码器原始角度 (16-bit) */
    int32_t  enc_theta_mrad;/* 编码器电角度 (毫弧度) */
    uint16_t enc_errors;    /* 编码器读取失败计数 */
    uint32_t enc_spikes;    /* 编码器尖峰拒绝计数 */
    uint32_t enc_bus_errors;/* 编码器总线错误计数 */
    uint8_t  enc_alive;     /* 编码器存活 (0/1) */
    uint32_t align_hits;    /* ALIGN 分支命中计数 */
    uint32_t cal_state;     /* 当前标定状态 (cal_state_t 镜像) */
    uint8_t  cal_progress;  /* 标定进度 0..100 */
    /* Stage 5: 电流环快照 */
    uint32_t cur_hits;      /* CURRENT 分支命中计数 */
    int32_t  id_ma;         /* 实测 d 轴电流 (毫安) */
    int32_t  iq_ma;         /* 实测 q 轴电流 (毫安) */
    int32_t  id_ref_ma;     /* 目标 Id (毫安) */
    int32_t  iq_ref_ma;     /* 目标 Iq (毫安) */
} motor_control_isr_debug_t;

void motor_control_isr_get_debug(motor_control_isr_debug_t *dbg);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_ISR_H */
