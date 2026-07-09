/*
 * motor_control_isr.c - FOC ISR 主体 (TMR1_OVF 16kHz 调用)
 *
 * Stage 2: OPEN_LOOP 模式实现
 *   - 角度斜坡: theta_e += speed_rad_per_s * dt
 *   - Vd = open_loop_vd, Vq = 0
 *   - IPark -> SVPWM -> 写 CCR
 *   - DISABLED/FAULT: 输出 50% 三相同电位
 *
 * ISR 不调用 RT-Thread API (spec §3.5). 与 shell 共享的运行时参数
 * (vd/speed/theta_e) 用 volatile 标记, 单 word 读写原子 (M4 32-bit).
 *
 * 注意: ARMCC V5.06 默认 C90, 变量声明必须在块开头.
 */
#include "motor_control_isr.h"
#include "motor_app.h"
#include "foc_core.h"
#include "motor_pwm_at32m412.h"
#include "current_sense_at32m412.h"
#include "encoder_tracker.h"
#include "motor_calibration.h"
#include "fault_manager.h"
#include "motor_params.h"
#include "board_motor_pins.h"
#include "current_loop.h"
#include "speed_loop.h"
#include <math.h>

#define ISR_DT_S            (1.0f / (float)PWM_FREQUENCY_HZ)   /* 62.5 µs */
#define TWO_PI_F            6.28318530718f
#define OPEN_LOOP_VD_MAX    (VBUS_OVERVOLTAGE_THRESHOLD_V)     /* 18V 上限 */
#define OPEN_LOOP_VD_MIN    0.0f
#define OPEN_LOOP_SPEED_MAX 628.0f   /* 100 Hz 电角度上限 (~600rpm @7pp) */

/* 开环安全 VBUS: ADC 未就绪时的回退值 (避免除零) */
#define VBUS_FALLBACK_V     12.0f

/* 编码器连续失败阈值: 超过则置 FAULT_SENSOR (spec §4.6) */
#define ENC_FAIL_THRESHOLD  32u

/* ALIGN 零点采样: 前 ALIGN_SETTLE_MS 稳定, 后 ALIGN_SAMPLE_TICKS 累加平均.
 * 用累加和 + 计数器, 不需环形缓冲, 省 RAM. */

/* ===== 开环运行时参数 (ISR 读, shell 写) ===== */
static volatile float    s_ol_vd;            /* d 轴目标电压 (V) */
static volatile float    s_ol_speed_rad_s;   /* theta_e 角速度 (rad/s) */
static volatile float    s_ol_theta_e;       /* 当前电角度 (rad, [0,2π)) */
static volatile bool     s_ol_active;        /* 开环是否激活 */
static volatile bool     s_ol_use_enc;       /* OPEN_LOOP 电角度来源: true=编码器, false=斜坡 */

/* ===== ALIGN 模式参数 (Stage 4, spec §4.5.3) ===== */
static volatile float    s_align_vd;         /* ALIGN 模式 Vd (V) */
static volatile bool     s_align_active;     /* ALIGN 是否激活 */
static volatile uint32_t s_align_tick_cnt;   /* ALIGN 启动后累计 tick */
static volatile uint32_t s_align_sum;        /* 采样窗口内角度累加和 */
static volatile uint32_t s_align_sample_cnt; /* 采样窗口内已采样本数 */

/* ===== Stage 5: CURRENT 模式参数 (ISR 读, shell 写) ===== */
static volatile bool     s_cur_active;       /* CURRENT 模式是否激活 */
static volatile bool     s_cur_use_enc;      /* theta 来源: true=编码器电角度, false=斜坡 (独立于 s_ol_use_enc) */
static volatile float    s_cur_theta_e;      /* ramp 模式电角度 (rad, [0,2π)) */
static volatile float    s_cur_speed_rad_s;  /* ramp 模式角速度 (rad/s), 调试用 */

/* ===== Stage 6: SPEED 模式参数 (ISR 读, shell 写) ===== */
static volatile bool     s_spd_active;      /* SPEED 模式是否激活 */

/* ===== 编码器读取状态 (Stage 4) ===== */
static volatile uint16_t s_enc_raw16;        /* 最近一次编码器原始角度 (16-bit) */
static volatile int16_t  s_enc_speed_raw;    /* 编码器速度原始值 */
static volatile float    s_enc_theta_e;      /* 编码器电角度 (rad) */
static volatile uint16_t s_enc_error_cnt;    /* 读取失败累计 */

/* ===== ISR 调试用快照 (ISR 写, shell 读) ===== */
static volatile int32_t  s_dbg_theta_mrad;
static volatile int32_t  s_dbg_v_alpha_mv;
static volatile int32_t  s_dbg_v_beta_mv;
static volatile uint16_t s_dbg_ta;
static volatile uint16_t s_dbg_tb;
static volatile uint16_t s_dbg_tc;
static volatile uint32_t s_dbg_tick_count;
static volatile uint32_t s_dbg_ol_hits;
static volatile uint32_t s_dbg_fault_hits;
static volatile uint32_t s_dbg_disabled_hits;

/* Stage 3: 电流/电压采样快照 */
static volatile int32_t  s_dbg_ia_ma;       /* 三相电流 (毫安) */
static volatile int32_t  s_dbg_ib_ma;
static volatile int32_t  s_dbg_ic_ma;
static volatile int32_t  s_dbg_vbus_mv;     /* 母线电压 (毫伏) */
static volatile uint16_t s_dbg_ia_raw;      /* ADC 原始值 */
static volatile uint16_t s_dbg_ib_raw;
static volatile uint16_t s_dbg_ic_raw;
static volatile uint32_t s_dbg_oc_hits;     /* 过流保护命中计数 */
static volatile uint32_t s_dbg_imbal_hits;  /* 电流不平衡命中计数 */
static uint16_t          s_oc_consec;       /* 连续相过流计数 (防单拍 ADC 毛刺误触发) */
static uint16_t          s_imbal_consec;    /* 连续不平衡计数 (防单拍 ADC 毛刺误触发) */

/* Stage 4: 编码器 + ALIGN + 标定快照 */
static volatile uint16_t s_dbg_enc_raw;
static volatile int32_t  s_dbg_enc_theta_mrad;
static volatile uint16_t s_dbg_enc_errors;
static volatile uint32_t s_dbg_enc_spikes;
static volatile uint32_t s_dbg_enc_bus_errors;
static volatile uint8_t  s_dbg_enc_alive;
static volatile uint32_t s_dbg_align_hits;
static volatile uint32_t s_dbg_cal_state;
static volatile uint8_t  s_dbg_cal_progress;

/* Stage 5: 电流环快照 */
static volatile uint32_t s_dbg_cur_hits;
static volatile int32_t  s_dbg_id_ma;
static volatile int32_t  s_dbg_iq_ma;

/* Stage 6: 速度环快照 */
static volatile uint32_t s_dbg_spd_hits;

/* VBUS 缓存: FOC ISR 内不触发普通 ADC 转换, 避免与注入序列冲突.
 * 后续 VBUS 保护应移到低频线程或独立硬件触发路径. */
static volatile float    s_vbus_cached = VBUS_FALLBACK_V;

#define RAD_TO_MRAD_F       1000.0f
#define VOLTS_TO_MV_F       1000.0f

/* (Stage 5 电流环将引入限幅 helper, 此处预留位置) */

void motor_control_isr_tick(void)
{
    motor_control_t *mc;
    float vbus;
    float theta;
    float vd;
    float speed;
    float v_alpha;
    float v_beta;
    uint16_t ta;
    uint16_t tb;
    uint16_t tc;
    /* Stage 3: 电流采样 */
    uint16_t ia_raw;
    uint16_t ib_raw;
    uint16_t ic_raw;
    uint16_t ofs_a;
    uint16_t ofs_b;
    uint16_t ofs_c;
    float    ia;
    float    ib;
    float    ic;
    float    i_sum;
    float    gain;

    s_dbg_tick_count++;
    mc = motor_app_get_control_rw();

    /* ===== Stage 3: 读 ADC 注入序列 (硬件已由 TMR1_CH4 顶点触发完成) ===== */
    current_sense_at32m412_read_raw(&ia_raw, &ib_raw, &ic_raw);
    s_dbg_ia_raw = ia_raw;
    s_dbg_ib_raw = ib_raw;
    s_dbg_ic_raw = ic_raw;

    vbus = s_vbus_cached;
    s_dbg_vbus_mv = (int32_t)(vbus * VOLTS_TO_MV_F);

    /* 电流换算 (raw -> A, spec §4.3.4). 零偏由标定流程设置. */
    current_sense_at32m412_get_offset(&ofs_a, &ofs_b, &ofs_c);
    gain = CURRENT_GAIN_DEFAULT_A_PER_LSB;   /* ~3.16 mA/LSB typ */
    ia = current_sense_calc(ia_raw, (float)ofs_a, gain);
    ib = current_sense_calc(ib_raw, (float)ofs_b, gain);
    ic = current_sense_calc(ic_raw, (float)ofs_c, gain);
    s_dbg_ia_ma = (int32_t)(ia * 1000.0f);
    s_dbg_ib_ma = (int32_t)(ib * 1000.0f);
    s_dbg_ic_ma = (int32_t)(ic * 1000.0f);

    /* ===== Stage 3: 电流保护 (过流 + 不平衡, spec §4.3.5) ===== */
    /* 过流: 任一相电流绝对值超阈值 */
    if (fabsf(ia) > IQ_OVERCURRENT_A ||
        fabsf(ib) > IQ_OVERCURRENT_A ||
        fabsf(ic) > IQ_OVERCURRENT_A) {
        s_dbg_oc_hits++;
        s_oc_consec++;
        if (s_oc_consec >= OVERCURRENT_DEBOUNCE_TICKS) {
            fault_manager_set(FAULT_OVERCURRENT);
        }
    } else {
        s_oc_consec = 0u;
    }
    /* 不平衡: ia+ib+ic 应接近 0 (基尔霍夫), 超阈值视为故障.
     * 防毛刺: 连续 IMBALANCE_DEBOUNCE_TICKS 才锁存故障 (Stage 5 调试发现
     * 电流环启动瞬态 PWM 切换时单拍 ADC 毛刺可致 i_sum 瞬超阈值). */
    i_sum = ia + ib + ic;
    if (fabsf(i_sum) > IMBALANCE_THRESHOLD_A) {
        s_dbg_imbal_hits++;
        s_imbal_consec++;
        if (s_imbal_consec >= IMBALANCE_DEBOUNCE_TICKS) {
            fault_manager_set(FAULT_OVERCURRENT);
        }
    } else {
        s_imbal_consec = 0u;
    }

    /* 故障态: 强制关 PWM 输出 + 50% 三相同电位, 不出力 (spec §3.3)
     * 仅致命故障触发关断; FAULT_CAL_INVALID 是告警, 不阻止 (spec §4.7.3) */
    if (mc->state == MOTOR_CONTROL_STATE_FAULT || fault_manager_any_fatal()) {
        s_dbg_fault_hits++;
        motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);
        motor_pwm_at32m412_disable_output();
        return;
    }

    /* DISABLED: 50% 三相同电位, 不出力 (但 ISR 仍运行, 供调试观察) */
    if (mc->state != MOTOR_CONTROL_STATE_ENABLED) {
        s_dbg_disabled_hits++;
        motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);
        return;
    }

    /* ===== Stage 4: 编码器跟踪 (FOC tick 只预测角度, 不做 SPI 访问) ===== */
    {
        encoder_tracker_snapshot_t trk;
        uint32_t age;

        encoder_tracker_tick();
        s_enc_theta_e = encoder_tracker_get_electrical_angle_rad();
        age = encoder_tracker_get_sample_age_ticks();

        if (encoder_tracker_get_snapshot(&trk)) {
            s_enc_raw16 = trk.raw16;
            s_enc_speed_raw = (int16_t)encoder_tracker_get_speed_rad_s();
            s_dbg_enc_raw = trk.raw16;
            s_dbg_enc_theta_mrad = trk.elec_mrad;
            s_dbg_enc_errors = s_enc_error_cnt;
            s_dbg_enc_alive = (age < ENC_FAIL_THRESHOLD) ? 1u : 0u;
        } else {
            s_enc_error_cnt++;
            s_dbg_enc_errors = s_enc_error_cnt;
            s_dbg_enc_alive = 0u;
        }

        if (age >= ENC_FAIL_THRESHOLD) {
            fault_manager_set(FAULT_SENSOR);
        }
    }

    /* ===== ENABLED: 按模式分支 ===== */
    /* Stage 3: VBUS 用实测值 (1kHz 刷新缓存), 替代 Stage 2 硬编码 12V */
    if (vbus < 1.0f) {
        vbus = VBUS_FALLBACK_V;   /* ADC 异常时回退, 避免 SVPWM 除零 */
    }

    switch (mc->mode) {
        case MOTOR_CONTROL_MODE_ALIGN:
            /* Stage 4: ALIGN 模式, 固定 theta=0 锁定转子到 d 轴 (spec §4.5.3) */
            if (!s_align_active) {
                motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);
                return;
            }
            s_dbg_align_hits++;
            theta = 0.0f;   /* d 轴 0° 方向 */
            vd = s_align_vd;
            s_dbg_theta_mrad = 0;

            foc_ipark(vd, 0.0f, theta, &v_alpha, &v_beta);
            s_dbg_v_alpha_mv = (int32_t)(v_alpha * VOLTS_TO_MV_F);
            s_dbg_v_beta_mv  = (int32_t)(v_beta  * VOLTS_TO_MV_F);

            foc_svpwm_3phase_high_side(v_alpha, v_beta, vbus, &ta, &tb, &tc);
            s_dbg_ta = ta; s_dbg_tb = tb; s_dbg_tc = tc;
            motor_pwm_at32m412_set_duty_ticks(ta, tb, tc);
            break;

        case MOTOR_CONTROL_MODE_OPEN_LOOP:
            if (!s_ol_active) {
                /* 安全兜底: 开环未激活时输出 50% */
                motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);
                return;
            }
            s_dbg_ol_hits++;

            vd = s_ol_vd;

            /* Stage 4: 电角度来源 (--enc 开关) */
            if (s_ol_use_enc) {
                /* 用编码器电角度, 验证编码器方向/零点 */
                theta = s_enc_theta_e;
            } else {
                /* 斜坡递增 (Stage 2 默认行为) */
                speed = s_ol_speed_rad_s;
                theta = s_ol_theta_e + speed * ISR_DT_S;
                while (theta < 0.0f)        theta += TWO_PI_F;
                while (theta >= TWO_PI_F)   theta -= TWO_PI_F;
                s_ol_theta_e = theta;
            }
            s_dbg_theta_mrad = (int32_t)(theta * RAD_TO_MRAD_F);

            /* IPark: Vd/Vq -> alpha/beta (Vq=0) */
            foc_ipark(vd, 0.0f, theta, &v_alpha, &v_beta);
            s_dbg_v_alpha_mv = (int32_t)(v_alpha * VOLTS_TO_MV_F);
            s_dbg_v_beta_mv  = (int32_t)(v_beta  * VOLTS_TO_MV_F);

            /* SVPWM */
            foc_svpwm_3phase_high_side(v_alpha, v_beta, vbus, &ta, &tb, &tc);
            s_dbg_ta = ta;
            s_dbg_tb = tb;
            s_dbg_tc = tc;

            motor_pwm_at32m412_set_duty_ticks(ta, tb, tc);
            break;

        case MOTOR_CONTROL_MODE_CURRENT: {
            float id, iq;
            float i_alpha, i_beta;
            float vd_ref, vq_ref;

            if (!s_cur_active) {
                motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);
                return;
            }
            s_dbg_cur_hits++;

            /* theta 来源 (enc/ramp). enc 模式依赖有效标定 (shell 层已拒 CAL_INVALID). */
            if (s_cur_use_enc) {
                theta = s_enc_theta_e;
            } else {
                s_cur_theta_e += s_cur_speed_rad_s * ISR_DT_S;
                while (s_cur_theta_e < 0.0f)      s_cur_theta_e += TWO_PI_F;
                while (s_cur_theta_e >= TWO_PI_F) s_cur_theta_e -= TWO_PI_F;
                theta = s_cur_theta_e;
            }
            s_dbg_theta_mrad = (int32_t)(theta * RAD_TO_MRAD_F);

            /* Clarke + Park: ia/ib/ic -> id/iq */
            foc_clarke(ia, ib, ic, &i_alpha, &i_beta);
            foc_park(i_alpha, i_beta, theta, &id, &iq);
            s_dbg_id_ma = (int32_t)(id * 1000.0f);
            s_dbg_iq_ma = (int32_t)(iq * 1000.0f);

            /* 电流环 PI (Id 目标 0, Iq 目标 = current_loop_set_targets 设定值) */
            current_loop_run(id, iq, &vd_ref, &vq_ref);

            /* IPark + SVPWM */
            foc_ipark(vd_ref, vq_ref, theta, &v_alpha, &v_beta);
            s_dbg_v_alpha_mv = (int32_t)(v_alpha * VOLTS_TO_MV_F);
            s_dbg_v_beta_mv  = (int32_t)(v_beta  * VOLTS_TO_MV_F);
            foc_svpwm_3phase_high_side(v_alpha, v_beta, vbus, &ta, &tb, &tc);
            s_dbg_ta = ta; s_dbg_tb = tb; s_dbg_tc = tc;
            motor_pwm_at32m412_set_duty_ticks(ta, tb, tc);
            break;
        }

        case MOTOR_CONTROL_MODE_SPEED:
        {
            float id, iq;
            float i_alpha, i_beta;
            float vd_ref, vq_ref;
            float iq_ref;

            if (!s_spd_active) {
                motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);
                return;
            }
            s_dbg_spd_hits++;

            theta = s_enc_theta_e;
            s_dbg_theta_mrad = (int32_t)(theta * RAD_TO_MRAD_F);

            iq_ref = speed_loop_run(encoder_tracker_get_speed_rad_s());
            current_loop_set_targets(0.0f, iq_ref);

            foc_clarke(ia, ib, ic, &i_alpha, &i_beta);
            foc_park(i_alpha, i_beta, theta, &id, &iq);
            s_dbg_id_ma = (int32_t)(id * 1000.0f);
            s_dbg_iq_ma = (int32_t)(iq * 1000.0f);

            current_loop_run(id, iq, &vd_ref, &vq_ref);

            foc_ipark(vd_ref, vq_ref, theta, &v_alpha, &v_beta);
            s_dbg_v_alpha_mv = (int32_t)(v_alpha * VOLTS_TO_MV_F);
            s_dbg_v_beta_mv  = (int32_t)(v_beta  * VOLTS_TO_MV_F);
            foc_svpwm_3phase_high_side(v_alpha, v_beta, vbus, &ta, &tb, &tc);
            s_dbg_ta = ta; s_dbg_tb = tb; s_dbg_tc = tc;
            motor_pwm_at32m412_set_duty_ticks(ta, tb, tc);
            break;
        }

        case MOTOR_CONTROL_MODE_POSITION:
            /* Stage 7+ 实现, 暂输出 50% */
            motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);
            break;

        default:
            motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);
            break;
    }

    /* ===== Stage 4b: 标定采集钩子 (SPIN_FWD/REV 状态下累加直方图) ===== */
    motor_calibration_tick();
    s_dbg_cal_state = (uint32_t)motor_calibration_get_state();
    s_dbg_cal_progress = motor_calibration_get_progress();
}

int motor_control_isr_open_loop_start(float vd_volts, float speed_rad_per_s)
{
    motor_control_t *mc;

    /* 参数限幅 */
    if (vd_volts < OPEN_LOOP_VD_MIN || vd_volts > OPEN_LOOP_VD_MAX) {
        return -2;
    }
    if (speed_rad_per_s < -OPEN_LOOP_SPEED_MAX || speed_rad_per_s > OPEN_LOOP_SPEED_MAX) {
        return -2;
    }

    mc = motor_app_get_control_rw();

    /* 致命故障未清不允许启动 (CAL_INVALID 告警不阻止, spec §4.7.3) */
    if (fault_manager_any_fatal()) {
        return -1;
    }

    /* 设置开环参数 */
    s_ol_vd          = vd_volts;
    s_ol_speed_rad_s = speed_rad_per_s;
    s_ol_theta_e     = 0.0f;
    s_ol_active      = true;
    s_align_active   = false;   /* 互斥: 启动 open_loop 前清 ALIGN */
    s_cur_active     = false;   /* 互斥: 清 CURRENT (Stage 5, 根因1对称清理) */
    s_spd_active     = false;   /* 互斥: 清 SPEED */
    encoder_tracker_reset();

    /* 切模式 + 使能 */
    mc->mode  = MOTOR_CONTROL_MODE_OPEN_LOOP;
    mc->state = MOTOR_CONTROL_STATE_ENABLED;

    /* 使能 MP6540H (拉高 EN) */
    motor_pwm_at32m412_enable_output();

    /* 使能 TMR1_OVF 中断, 启动 16kHz FOC ISR */
    motor_pwm_at32m412_enable_ovf_irq();

    return 0;
}

void motor_control_isr_open_loop_stop(void)
{
    motor_control_t *mc;

    /* 互斥清理: open_loop 与 ALIGN 共用 ISR/EN, 任一 stop 都清两个 active 标志.
     * 否则 mc_align 后 mc_stop (open_loop_stop) 不清 s_align_active, 后续 mc_open
     * 会让两个分支同时命中, enc_cal_start 检查 align_active() 误报 "motor running". */
    s_ol_active = false;
    s_align_active = false;
    s_spd_active = false;
    s_ol_vd     = 0.0f;
    s_ol_theta_e = 0.0f;
    s_align_vd  = 0.0f;
    encoder_tracker_reset();

    /* 先停 ISR, 再改输出 (避免竞态) */
    motor_pwm_at32m412_disable_ovf_irq();

    /* 立即输出 50% 三相同电位 */
    motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);

    /* 禁用 MP6540H */
    motor_pwm_at32m412_disable_output();

    mc = motor_app_get_control_rw();
    mc->state = MOTOR_CONTROL_STATE_DISABLED;
}

bool motor_control_isr_open_loop_active(void)
{
    return s_ol_active;
}

void motor_control_isr_open_loop_set_encoder_angle(bool use_enc)
{
    s_ol_use_enc = use_enc;
}

/* ===== Stage 4: ALIGN 零点对齐接口 (spec §4.5.3) ===== */

int motor_control_isr_align_start(float vd_volts)
{
    motor_control_t *mc;

    /* 参数限幅 */
    if (vd_volts < 0.0f || vd_volts > ALIGN_VD_MAX_VOLTS) {
        return -2;
    }

    mc = motor_app_get_control_rw();

    /* 致命故障未清不允许启动 */
    if (fault_manager_any_fatal()) {
        return -1;
    }

    /* 设置 ALIGN 参数 */
    s_align_vd = vd_volts;
    s_align_active = true;
    s_align_tick_cnt = 0u;
    s_align_sum = 0u;
    s_align_sample_cnt = 0u;
    s_ol_active = false;   /* 互斥: 启动 ALIGN 前清 open_loop */
    s_cur_active = false;  /* 互斥: 清 CURRENT (Stage 5, 根因1对称清理) */
    s_spd_active = false;  /* 互斥: 清 SPEED */

    /* 切 ALIGN 模式 + 使能 */
    mc->mode = MOTOR_CONTROL_MODE_ALIGN;
    mc->state = MOTOR_CONTROL_STATE_ENABLED;

    motor_pwm_at32m412_enable_output();
    motor_pwm_at32m412_enable_ovf_irq();

    return 0;
}

void motor_control_isr_align_stop(void)
{
    motor_control_t *mc;

    /* 互斥清理: 与 open_loop_stop 对称, 清两个 active 标志.
     * 注: 去掉原 "if (!s_align_active) return" 早返回, 否则 ALIGN 未激活时
     * 调 align_stop (如 mc_stop 路径) 无法清理可能残留的 s_ol_active. */
    s_align_active = false;
    s_ol_active = false;
    s_spd_active = false;
    s_align_vd = 0.0f;
    s_ol_vd = 0.0f;
    s_ol_theta_e = 0.0f;
    encoder_tracker_reset();

    motor_pwm_at32m412_disable_ovf_irq();
    motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);
    motor_pwm_at32m412_disable_output();

    mc = motor_app_get_control_rw();
    mc->state = MOTOR_CONTROL_STATE_DISABLED;
}

bool motor_control_isr_align_active(void)
{
    return s_align_active;
}

uint16_t motor_control_isr_get_align_angle(void)
{
    /* 返回采样窗口内角度的平均 (uint16 环绕平均) */
    if (s_align_sample_cnt == 0u) {
        return 0u;
    }
    return (uint16_t)(s_align_sum / s_align_sample_cnt);
}

void motor_control_isr_on_encoder_sample(uint16_t raw16)
{
    if (s_align_active) {
        s_align_tick_cnt++;
        if (s_align_tick_cnt > ALIGN_SETTLE_MS) {
            s_align_sum += raw16;
            s_align_sample_cnt++;
        }
    }
}

/* ===== Stage 5: CURRENT 模式接口 (spec-stage5 §5.3) ===== */

int motor_control_isr_current_start(float iq_ref_A)
{
    motor_control_t *mc;

    /* 参数限幅: Iq 目标不超过 IQ_MAX_A (defense-in-depth, 同 open_loop_start) */
    if (iq_ref_A < -IQ_MAX_A || iq_ref_A > IQ_MAX_A) {
        return -2;
    }

    /* 致命故障未清不允许启动 */
    if (fault_manager_any_fatal()) {
        return -1;
    }

    /* 互斥: 清 OPEN_LOOP/ALIGN 残留 (Stage 4b 根因 1 教训) */
    s_ol_active = false;
    s_align_active = false;
    s_spd_active = false;
    s_ol_vd = 0.0f;
    s_align_vd = 0.0f;

    /* 清积分 + 设目标 (Id=0). reset 须在 ISR 非激活时调 (current_loop 约定).
     * 注: s_cur_use_enc/s_cur_speed_rad_s 由调用方在 start 前通过
     *     set_encoder_angle/set_speed 设置 (同 open_loop_start 取参数的风格),
     *     不在此重置, 以免覆盖调用方意图. */
    current_loop_reset();
    current_loop_set_targets(0.0f, iq_ref_A);
    s_cur_theta_e = 0.0f;
    s_cur_active = true;
    encoder_tracker_reset();

    /* 切 CURRENT 模式 + 使能 (同 align_start 模式) */
    mc = motor_app_get_control_rw();
    mc->mode = MOTOR_CONTROL_MODE_CURRENT;
    mc->state = MOTOR_CONTROL_STATE_ENABLED;

    motor_pwm_at32m412_enable_output();
    motor_pwm_at32m412_enable_ovf_irq();

    return 0;
}

void motor_control_isr_current_stop(void)
{
    motor_control_t *mc;

    s_cur_active = false;
    s_spd_active = false;
    s_cur_theta_e = 0.0f;
    current_loop_reset();
    encoder_tracker_reset();

    motor_pwm_at32m412_disable_ovf_irq();
    motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);
    motor_pwm_at32m412_disable_output();

    mc = motor_app_get_control_rw();
    mc->state = MOTOR_CONTROL_STATE_DISABLED;
}

bool motor_control_isr_current_active(void)
{
    return s_cur_active;
}

void motor_control_isr_current_set_encoder_angle(bool use_enc)
{
    s_cur_use_enc = use_enc;
}

void motor_control_isr_current_set_speed(float rad_per_s)
{
    /* 限幅: 复用 OPEN_LOOP_SPEED_MAX 安全上界 (ramp 调试用, 同 open_loop) */
    if (rad_per_s < -OPEN_LOOP_SPEED_MAX) rad_per_s = -OPEN_LOOP_SPEED_MAX;
    if (rad_per_s >  OPEN_LOOP_SPEED_MAX) rad_per_s =  OPEN_LOOP_SPEED_MAX;
    s_cur_speed_rad_s = rad_per_s;
}

/* ===== Stage 6: SPEED 模式接口 ===== */

int motor_control_isr_speed_start(float target_rad_s)
{
    motor_control_t *mc;
    float max_speed;

    max_speed = (float)RPM_MAX * 6.28318530718f / 60.0f;
    if (target_rad_s < -max_speed || target_rad_s > max_speed) {
        return -2;
    }
    if (fault_manager_any_fatal()) {
        return -1;
    }

    s_ol_active = false;
    s_align_active = false;
    s_cur_active = false;
    s_ol_vd = 0.0f;
    s_align_vd = 0.0f;
    current_loop_reset();
    speed_loop_reset();
    speed_loop_set_target_rad_s(target_rad_s);
    current_loop_set_targets(0.0f, 0.0f);
    s_spd_active = true;
    encoder_tracker_reset();

    mc = motor_app_get_control_rw();
    mc->mode = MOTOR_CONTROL_MODE_SPEED;
    mc->state = MOTOR_CONTROL_STATE_ENABLED;

    motor_pwm_at32m412_enable_output();
    motor_pwm_at32m412_enable_ovf_irq();

    return 0;
}

void motor_control_isr_speed_stop(void)
{
    motor_control_t *mc;

    s_spd_active = false;
    speed_loop_reset();
    current_loop_reset();
    current_loop_set_targets(0.0f, 0.0f);
    encoder_tracker_reset();

    motor_pwm_at32m412_disable_ovf_irq();
    motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);
    motor_pwm_at32m412_disable_output();

    mc = motor_app_get_control_rw();
    mc->state = MOTOR_CONTROL_STATE_DISABLED;
}

bool motor_control_isr_speed_active(void)
{
    return s_spd_active;
}

void motor_control_isr_get_debug(motor_control_isr_debug_t *dbg)
{
    if (dbg == 0) {
        return;
    }
    dbg->theta_mrad     = s_dbg_theta_mrad;
    dbg->v_alpha_mv     = s_dbg_v_alpha_mv;
    dbg->v_beta_mv      = s_dbg_v_beta_mv;
    dbg->ta             = s_dbg_ta;
    dbg->tb             = s_dbg_tb;
    dbg->tc             = s_dbg_tc;
    dbg->tick_count     = s_dbg_tick_count;
    dbg->ol_branch_hits = s_dbg_ol_hits;
    dbg->fault_hits     = s_dbg_fault_hits;
    dbg->disabled_hits  = s_dbg_disabled_hits;
    dbg->ia_ma          = s_dbg_ia_ma;
    dbg->ib_ma          = s_dbg_ib_ma;
    dbg->ic_ma          = s_dbg_ic_ma;
    dbg->vbus_mv        = s_dbg_vbus_mv;
    dbg->ia_raw         = s_dbg_ia_raw;
    dbg->ib_raw         = s_dbg_ib_raw;
    dbg->ic_raw         = s_dbg_ic_raw;
    dbg->oc_hits        = s_dbg_oc_hits;
    dbg->imbal_hits     = s_dbg_imbal_hits;
    dbg->enc_raw        = s_dbg_enc_raw;
    dbg->enc_theta_mrad = s_dbg_enc_theta_mrad;
    dbg->enc_errors     = s_dbg_enc_errors;
    dbg->enc_spikes     = s_dbg_enc_spikes;
    dbg->enc_bus_errors = s_dbg_enc_bus_errors;
    dbg->enc_alive      = s_dbg_enc_alive;
    dbg->align_hits     = s_dbg_align_hits;
    dbg->cal_state      = s_dbg_cal_state;
    dbg->cal_progress   = s_dbg_cal_progress;
    dbg->cur_hits       = s_dbg_cur_hits;
    dbg->id_ma          = s_dbg_id_ma;
    dbg->iq_ma          = s_dbg_iq_ma;
    dbg->id_ref_ma      = (int32_t)(current_loop_get_id_ref_A() * 1000.0f);
    dbg->iq_ref_ma      = (int32_t)(current_loop_get_iq_ref_A() * 1000.0f);
    dbg->spd_hits       = s_dbg_spd_hits;
    dbg->spd_target_mrad_s = (int32_t)(speed_loop_get_target_rad_s() * 1000.0f);
    dbg->spd_cmd_mrad_s    = (int32_t)(speed_loop_get_command_rad_s() * 1000.0f);
    dbg->spd_meas_mrad_s   = (int32_t)(speed_loop_get_measured_rad_s() * 1000.0f);
    dbg->spd_iq_ref_ma     = (int32_t)(speed_loop_get_iq_ref_A() * 1000.0f);
}
