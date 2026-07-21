/*
 * motor_calibration.c - 旁轴磁编非线性标定状态机 (Stage 4b, spec §4.7)
 *
 * 原理 (spec §4.7.2 正反恒速拖动对消法):
 *   1. 恒速正转记录 θ_raw(t), 速度脉动 + 几何误差混合
 *   2. 恒速反转记录 θ_raw(t), 速度脉动反号, 几何误差同号
 *   3. 平均 (fwd+rev)/2 -> 速度脉动消除, 剩纯几何误差作"假基准"
 *   4. 残差 e(θ_raw) = θ_raw - 假基准, 按 θ_raw 分 256 箱直方图平均
 *
 * 上下文边界 (spec §3.5 PRIMASK 约束):
 *   - motor_calibration_tick(): ISR 内, 仅累加直方图 + 计数切状态, 不调 RT-Thread/FLASH API
 *   - motor_calibration_poll(): 线程上下文, 处理含阻塞的状态 (ALIGN 等待/COMPUTE/WRITE_FLASH)
 *
 * RAM 占用: 两个 256×int32 直方图 = 2KB + s_cal 528B, 共约 2.5KB
 */
#include "motor_calibration.h"
#include "encoder_service.h"
#include "fault_manager.h"
#include "flash_calibration_at32m412.h"
#include "motor_params.h"
#include <rtthread.h>
#include <string.h>

/* ===== 前向声明: motor_control_isr 的接口 (避免头文件循环依赖) =====
 * 这些函数在 motor_control_isr.c 实现, 此处声明以便标定状态机调用. */
extern int      motor_control_isr_align_start(float vd_volts);
extern void     motor_control_isr_align_stop(void);
extern uint16_t motor_control_isr_get_align_angle(void);
extern int      motor_control_isr_open_loop_start(float vd_volts, float speed_rad_per_s);
extern void     motor_control_isr_open_loop_stop(void);
extern void     motor_control_isr_open_loop_set_encoder_angle(bool use_enc);

/* ===== 全局零点 (供 motor_encoder_at32m412.c 读取) ===== */
uint16_t g_motor_zero_raw = 0u;

/* ===== 静态状态 ===== */
static motor_calibration_t s_cal;
static bool     s_cal_valid = false;
static cal_state_t s_cal_state = CAL_STATE_IDLE;
static cal_mode_t s_cal_mode = CAL_MODE_AUTO_OPEN_LOOP;
static motor_calibration_quality_t s_cal_quality;
static int16_t  s_max_residual = 0;       /* 0.001° 单位 */

/* 直方图: 每箱累加 (raw16 - bin_base), bin_base = idx * 256.
 * 单箱 raw 偏移范围 ±128 LSB, 累加 N 次后除 N 得平均偏移.
 * 用 int32 累加, 最大 20480 次 × 128 = 2.6M, 不溢出. */
static int32_t  s_hist_fwd[CAL_HIST_BINS];
static int32_t  s_hist_rev[CAL_HIST_BINS];
/* 每箱采样计数 (归一化用): compute 时 e[idx] = hist[idx] / count[idx] 得平均偏移.
 * 早期实现缺这一步, 把累加和当平均值, 值放大 N 倍饱和. uint16 够 (max 20480 < 65535). */
static uint16_t s_bin_count_fwd[CAL_HIST_BINS];
static uint16_t s_bin_count_rev[CAL_HIST_BINS];
static uint32_t s_hist_count_fwd = 0u;    /* 已累加样本数 */
static uint32_t s_hist_count_rev = 0u;

/* poll 节流与阶段计时 */
static uint32_t s_poll_tick_last = 0u;
static uint32_t s_phase_start_tick = 0u;  /* 当前阶段起始 tick (rt_tick_get) */

/* 子状态标志: ZERO_ALIGN 阶段内区分 "ALIGN 进行中" vs "ALIGN 完成" */
static bool     s_align_in_progress = false;
/* SPIN_FWD 阶段内区分 "ALIGN 刚完成待启动正转" vs "正转采集中" */
static bool     s_fwd_spin_started = false;
/* SPIN_REV 阶段内区分 "正转刚完成待启动反转" vs "反转采集中" */
static bool     s_rev_spin_started = false;

/* 标定用开环速度 (rad/s, 由 CAL_SPIN_SPEED_RPM 转换) */
#define CAL_SPIN_RAD_PER_S      ((float)CAL_SPIN_SPEED_RPM * 6.28318530718f / 60.0f)
#define CAL_VD_VOLTS            ALIGN_VD_VOLTS   /* 标定用开环 Vd, 与 ALIGN 一致 */

/* ============================================================ */
/*                      开机加载 (spec §4.7.7)                  */
/* ============================================================ */

void motor_calibration_load(void)
{
    /* 清空 RAM 表, 查表退化为不校正 */
    memset(&s_cal, 0, sizeof(s_cal));

    if (flash_calibration_read(&s_cal)) {
        s_cal_valid = true;
        g_motor_zero_raw = s_cal.mech_zero_raw;
        encoder_service_set_zero(g_motor_zero_raw);
        encoder_service_set_calibration_table(s_cal.table, true);
        fault_manager_clear_bits(FAULT_CAL_INVALID);
    } else {
        s_cal_valid = false;
        encoder_service_set_calibration_table(0, false);
        /* spec §4.7.7: 失败时置 FAULT_CAL_INVALID (告警级, 不阻止使能) */
        fault_manager_set_bits(FAULT_CAL_INVALID);
    }
}

bool motor_calibration_is_valid(void) { return s_cal_valid; }
const motor_calibration_t *motor_calibration_get(void) { return &s_cal; }
int16_t motor_calibration_get_max_residual(void) { return s_max_residual; }
const int16_t *motor_calibration_get_table(void) { return s_cal.table; }
uint16_t motor_calibration_get_zero(void) { return g_motor_zero_raw; }
void motor_calibration_set_zero(uint16_t raw)
{
    g_motor_zero_raw = raw;
    encoder_service_set_zero(raw);
}

cal_state_t motor_calibration_get_state(void) { return s_cal_state; }

uint8_t motor_calibration_get_progress(void)
{
    /* FWD/REV 阶段按旋转时间进度计算 (非样本数), 其它阶段固定比例 */
    switch (s_cal_state) {
        case CAL_STATE_IDLE:        return 0u;
        case CAL_STATE_ZERO_ALIGN:  return 5u;
        case CAL_STATE_SPIN_FWD: {
            uint32_t elapsed = (rt_tick_get() - s_phase_start_tick) * 1000u / RT_TICK_PER_SECOND;
            uint32_t pct = elapsed * 40u / (CAL_SPIN_DURATION_MS + 1u);
            if (pct > 40u) pct = 40u;
            return 5u + (uint8_t)pct;
        }
        case CAL_STATE_SPIN_REV: {
            uint32_t elapsed = (rt_tick_get() - s_phase_start_tick) * 1000u / RT_TICK_PER_SECOND;
            uint32_t pct = elapsed * 40u / (CAL_SPIN_DURATION_MS + 1u);
            if (pct > 40u) pct = 40u;
            return 45u + (uint8_t)pct;
        }
        case CAL_STATE_COMPUTE:     return 90u;
        case CAL_STATE_WRITE_FLASH: return 95u;
        case CAL_STATE_DONE:        return 100u;
        case CAL_STATE_ABORTED:     return 0u;
        default:                    return 0u;
    }
}

/* ============================================================ */
/*                  ISR 内采集 (spec §4.7.5)                    */
/* ============================================================ */

void motor_calibration_tick(void)
{
    uint16_t raw_16 = 0u;
    uint16_t idx;
    int32_t  delta;
    encoder_snapshot_t snap;

    if (s_cal_state != CAL_STATE_SPIN_FWD && s_cal_state != CAL_STATE_SPIN_REV) {
        return;
    }
    /* FWD/REV 的 ALIGN 启动过渡期 (s_*_spin_started=false) 不采集, 等 poll 启动开环 */
    if (s_cal_state == CAL_STATE_SPIN_FWD && !s_fwd_spin_started) return;
    if (s_cal_state == CAL_STATE_SPIN_REV && !s_rev_spin_started) return;

    if (!encoder_service_get_snapshot(&snap)) {
        return;
    }
    raw_16 = snap.raw16;

    /* 分箱: 256 箱, 每箱 256 LSB (65536/256) */
    idx = (uint16_t)(raw_16 >> 8);       /* 0..255 */
    /* 箱内偏移: raw - bin_base, bin_base = idx*256. 范围 0..255, 中心化到 -128..127 */
    delta = (int32_t)raw_16 - (int32_t)(idx * 256u) - 128;

    if (s_cal_state == CAL_STATE_SPIN_FWD) {
        s_hist_fwd[idx] += delta;
        s_bin_count_fwd[idx]++;
        s_hist_count_fwd++;
        /* 状态切换由 poll 按时间判断 (CAL_SPIN_DURATION_MS), 不在此按样本数切.
         * 早期按 CAL_SAMPLES_PER_DIRECTION 切状态, 但 20480@16kHz=1.28s 采满,
         * 而物理旋转需 70s, 采样远快于旋转, 采满时电机几乎没转. */
    } else { /* CAL_STATE_SPIN_REV */
        s_hist_rev[idx] += delta;
        s_bin_count_rev[idx]++;
        s_hist_count_rev++;
    }
}

/* ============================================================ */
/*              线程上下文状态机推进 (spec §4.7.5)             */
/* ============================================================ */

void motor_calibration_abort(void)
{
    if (s_cal_state == CAL_STATE_IDLE || s_cal_state == CAL_STATE_ABORTED) {
        s_cal_state = CAL_STATE_IDLE;
        return;
    }
    /* 停电机 (无论当前在开环还是 ALIGN) */
    motor_control_isr_open_loop_stop();
    motor_control_isr_align_stop();
    s_align_in_progress = false;
    s_fwd_spin_started = false;
    s_rev_spin_started = false;
    s_cal_state = CAL_STATE_ABORTED;
}

void motor_calibration_start_mode(cal_mode_t mode)
{
    s_cal_mode = mode;
    motor_calibration_start();
}

void motor_calibration_stop_manual(void)
{
    if ((s_cal_mode == CAL_MODE_MANUAL) && (s_cal_state == CAL_STATE_SPIN_FWD)) {
        s_cal_state = CAL_STATE_COMPUTE;
    }
}

bool motor_calibration_get_quality(motor_calibration_quality_t *out)
{
    if (out == 0) {
        return false;
    }
    *out = s_cal_quality;
    return true;
}

/* CAL_COMPUTE: 计算校正表 (spec §4.7.5 step 4) */
static void cal_compute_table(void)
{
    int32_t e_fwd;
    int32_t e_rev;
    int32_t e;
    int32_t sum_valid;
    int32_t mean;
    int32_t max_abs;
    uint16_t i;
    uint16_t valid_bins;
    uint16_t min_count;
    uint16_t bin_count;
    encoder_snapshot_t snap;

    /* 每箱平均偏移 = hist[idx] / count[idx] (归一化).
     * 早期实现直接 (fwd+rev)/2 把累加和当平均值, 值放大 N 倍饱和. */
    sum_valid = 0;
    valid_bins = 0;
    min_count = 0xFFFFu;
    for (i = 0; i < CAL_HIST_BINS; i++) {
        e_fwd = (s_bin_count_fwd[i] > 0u)
              ? (s_hist_fwd[i] / (int32_t)s_bin_count_fwd[i])
              : 0;
        e_rev = (s_bin_count_rev[i] > 0u)
              ? (s_hist_rev[i] / (int32_t)s_bin_count_rev[i])
              : 0;
        /* 正反平均去速度脉动 (spec §4.7.2). 仅当正反都采到才有效, 否则用单边. */
        if (s_bin_count_fwd[i] > 0u && s_bin_count_rev[i] > 0u) {
            e = (e_fwd + e_rev) / 2;
        } else if (s_bin_count_fwd[i] > 0u) {
            e = e_fwd;
        } else if (s_bin_count_rev[i] > 0u) {
            e = e_rev;
        } else {
            e = 0;   /* 未采到, 后续去直流后留 0 */
            continue; /* 不计入 mean */
        }
        s_hist_fwd[i] = e;   /* 复用 fwd 数组暂存归一化后的 e[idx] */
        sum_valid += e;
        valid_bins++;
        bin_count = (uint16_t)(s_bin_count_fwd[i] + s_bin_count_rev[i]);
        if (bin_count < min_count) {
            min_count = bin_count;
        }
    }

    /* 去直流: 仅对采到的箱算 mean, 保证表平均偏移为 0 */
    mean = (valid_bins > 0u) ? (sum_valid / (int32_t)valid_bins) : 0;

    max_abs = 0;
    for (i = 0; i < CAL_HIST_BINS; i++) {
        e = s_hist_fwd[i] - mean;
        /* LSB -> 0.001°: × 360000 / 65536 ≈ × 5.493 */
        e = (e * 360000) / 65536;
        /* 限幅 int16 */
        if (e > CAL_HIST_VALUE_MAX)  e = CAL_HIST_VALUE_MAX;
        if (e < -CAL_HIST_VALUE_MAX) e = -CAL_HIST_VALUE_MAX;
        s_cal.table[i] = (int16_t)e;
        if (e < 0) e = -e;
        if (e > max_abs) max_abs = e;
    }
    s_max_residual = (int16_t)max_abs;
    s_cal_quality.sample_count = s_hist_count_fwd + s_hist_count_rev;
    s_cal_quality.covered_bins = valid_bins;
    s_cal_quality.min_bin_count = (valid_bins > 0u) ? min_count : 0u;
    s_cal_quality.max_residual_mdeg = s_max_residual;
    if (encoder_service_get_snapshot(&snap)) {
        s_cal_quality.spike_count_end = snap.spike_count;
    }
    s_cal_quality.quality_ok = ((valid_bins >= 240u) &&
                                (s_max_residual <= CAL_MAX_RESIDUAL_MDEG)) ? 1u : 0u;
}

void motor_calibration_poll(void)
{
    uint32_t now;
    uint32_t elapsed_ms;

    now = rt_tick_get();
    /* 节流: 每 CAL_POLL_INTERVAL_MS 处理一次 (FWD/REV 采集中也节流, 仅检查超时) */
    if ((now - s_poll_tick_last) < (CAL_POLL_INTERVAL_MS * RT_TICK_PER_SECOND / 1000u)) {
        return;
    }
    s_poll_tick_last = now;

    /* 中止检查: 任意阶段致命故障 -> abort */
    if (s_cal_state != CAL_STATE_IDLE && s_cal_state != CAL_STATE_DONE &&
        s_cal_state != CAL_STATE_ABORTED) {
        if (fault_manager_any_fatal()) {
            motor_calibration_abort();
            return;
        }
    }

    switch (s_cal_state) {
        case CAL_STATE_IDLE:
        case CAL_STATE_DONE:
        case CAL_STATE_ABORTED:
            break;

        case CAL_STATE_ZERO_ALIGN:
            if (s_cal_mode == CAL_MODE_MANUAL) {
                encoder_snapshot_t snap;

                if (encoder_service_get_snapshot(&snap)) {
                    g_motor_zero_raw = snap.raw16;
                    encoder_service_set_zero(g_motor_zero_raw);
                }
                s_cal_state = CAL_STATE_SPIN_FWD;
                s_fwd_spin_started = true;
                s_phase_start_tick = now;
                break;
            }
            if (!s_align_in_progress) {
                /* 启动 ALIGN: 锁定转子到 d 轴 0° (spec §4.5.3) */
                int ret = motor_control_isr_align_start(ALIGN_VD_VOLTS);
                if (ret != 0) {
                    motor_calibration_abort();
                    return;
                }
                s_align_in_progress = true;
                s_phase_start_tick = now;
            } else {
                /* ALIGN 进行中: 等 ZERO_ALIGN_HOLD_MS (500ms) */
                elapsed_ms = (now - s_phase_start_tick) * 1000u / RT_TICK_PER_SECOND;
                if (elapsed_ms >= ZERO_ALIGN_HOLD_MS) {
                    /* 采对齐角度作零点 */
                    g_motor_zero_raw = motor_control_isr_get_align_angle();
                    motor_control_isr_align_stop();
                    s_align_in_progress = false;
                    /* 切正转采集: poll 下次启动开环 */
                    s_cal_state = CAL_STATE_SPIN_FWD;
                    s_fwd_spin_started = false;
                    s_hist_count_fwd = 0u;
                    s_hist_count_rev = 0u;
                    memset(s_hist_fwd, 0, sizeof(s_hist_fwd));
                    memset(s_hist_rev, 0, sizeof(s_hist_rev));
                    memset(s_bin_count_fwd, 0, sizeof(s_bin_count_fwd));
                    memset(s_bin_count_rev, 0, sizeof(s_bin_count_rev));
                }
            }
            break;

        case CAL_STATE_SPIN_FWD:
            if (!s_fwd_spin_started) {
                /* 启动正转开环 (纯斜坡, 不用编码器角度) */
                motor_control_isr_open_loop_set_encoder_angle(false);
                if (motor_control_isr_open_loop_start(CAL_VD_VOLTS, CAL_SPIN_RAD_PER_S) != 0) {
                    motor_calibration_abort();
                    return;
                }
                s_fwd_spin_started = true;
                s_phase_start_tick = now;
            } else {
                elapsed_ms = (now - s_phase_start_tick) * 1000u / RT_TICK_PER_SECOND;
                if (elapsed_ms >= CAL_SPIN_DURATION_MS) {
                    /* 旋转时长到, 切反转. 状态切换由 poll 按时间判断 (非样本数),
                     * 保证物理旋转覆盖足够机械圈. */
                    s_cal_state = CAL_STATE_SPIN_REV;
                    s_fwd_spin_started = false;   /* poll 启动反转时置 true */
                } else if (elapsed_ms > CAL_SPIN_TIMEOUT_MS) {
                    motor_calibration_abort();
                }
            }
            break;

        case CAL_STATE_SPIN_REV:
            if (!s_rev_spin_started) {
                /* 正转刚完成, 停正转, 启反转 */
                motor_control_isr_open_loop_stop();
                motor_control_isr_open_loop_set_encoder_angle(false);
                if (motor_control_isr_open_loop_start(CAL_VD_VOLTS, -CAL_SPIN_RAD_PER_S) != 0) {
                    motor_calibration_abort();
                    return;
                }
                s_rev_spin_started = true;
                s_phase_start_tick = now;
            } else {
                elapsed_ms = (now - s_phase_start_tick) * 1000u / RT_TICK_PER_SECOND;
                if (elapsed_ms >= CAL_SPIN_DURATION_MS) {
                    s_cal_state = CAL_STATE_COMPUTE;
                } else if (elapsed_ms > CAL_SPIN_TIMEOUT_MS) {
                    motor_calibration_abort();
                }
            }
            break;

        case CAL_STATE_COMPUTE:
            /* 停电机, 计算表 */
            motor_control_isr_open_loop_stop();
            cal_compute_table();
            s_cal_state = CAL_STATE_WRITE_FLASH;
            break;

        case CAL_STATE_WRITE_FLASH: {
            bool ok;
            if (!s_cal_quality.quality_ok) {
                fault_manager_set_bits(FAULT_CAL_INVALID);
                s_cal_state = CAL_STATE_ABORTED;
                break;
            }
            /* 填结构体 */
            s_cal.magic = CAL_MAGIC;
            s_cal.version = CAL_VERSION;
            s_cal.timestamp_ms = now * 1000u / RT_TICK_PER_SECOND;
            s_cal.mech_zero_raw = g_motor_zero_raw;
            s_cal.pole_pairs = MOTOR_POLE_PAIRS;
            s_cal.reserved2 = 0u;
            /* CRC 覆盖 table+mech_zero+pole_pairs+reserved2, 516 字节 */
            s_cal.crc32 = flash_calibration_crc32((const uint8_t *)&s_cal.table[0],
                                                   CAL_CRC_PAYLOAD_SIZE);
            ok = flash_calibration_write(&s_cal);
            if (ok) {
                s_cal_valid = true;
                encoder_service_set_zero(g_motor_zero_raw);
                encoder_service_set_calibration_table(s_cal.table, true);
                fault_manager_clear_bits(FAULT_CAL_INVALID);
                s_cal_state = CAL_STATE_DONE;
            } else {
                /* 写入失败, abort 但保留旧标定 (若有) */
                motor_calibration_abort();
            }
            break;
        }

        default:
            break;
    }
}

void motor_calibration_start(void)
{
    encoder_snapshot_t snap;

    /* 前置检查: 故障未清或电机运行中不允许启动 */
    if (fault_manager_any_fatal()) {
        return;
    }
    if (s_cal_state != CAL_STATE_IDLE && s_cal_state != CAL_STATE_DONE &&
        s_cal_state != CAL_STATE_ABORTED) {
        return;   /* 已在标定中 */
    }

    /* 重置采集状态 */
    s_hist_count_fwd = 0u;
    s_hist_count_rev = 0u;
    s_max_residual = 0;
    memset(&s_cal_quality, 0, sizeof(s_cal_quality));
    if (encoder_service_get_snapshot(&snap)) {
        s_cal_quality.spike_count_start = snap.spike_count;
        s_cal_quality.spike_count_end = snap.spike_count;
    }
    s_align_in_progress = false;
    s_fwd_spin_started = false;
    s_rev_spin_started = false;
    memset(s_hist_fwd, 0, sizeof(s_hist_fwd));
    memset(s_hist_rev, 0, sizeof(s_hist_rev));
    memset(s_bin_count_fwd, 0, sizeof(s_bin_count_fwd));
    memset(s_bin_count_rev, 0, sizeof(s_bin_count_rev));
    memset(&s_cal, 0, sizeof(s_cal));

    s_cal_state = CAL_STATE_ZERO_ALIGN;
}
