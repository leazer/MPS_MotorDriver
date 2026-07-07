/*
 * motor_shell.c - 电机调试 msh 命令 (finsh)
 *
 * 集中放置电机相关调试命令, 后续 Stage 持续扩充:
 *   pwm_info / pwm_duty / pwm_en / led / mc_state / fault / fault_clear / encoder
 *
 * 串口: USART1 PB6(TX)/PB7(RX) 115200 8N1, finsh msh 提示符 "msh />"
 *
 * 注意: ARMCC V5.06 默认 C90, 变量声明必须在块开头 (无语句后声明).
 */
#include <rtthread.h>
#include <finsh.h>
#include <stdlib.h>     /* atoi / strtol */

#include "board_motor_pins.h"
#include "at32m412_416.h"
#include "board_led_at32m412.h"
#include "motor_pwm_at32m412.h"
#include "current_sense_at32m412.h"
#include "motor_encoder_at32m412.h"
#include "encoder_service.h"
#include "encoder_tracker.h"
#include "motor_control.h"
#include "motor_control_isr.h"
#include "motor_calibration.h"
#include "flash_calibration_at32m412.h"
#include "fault_manager.h"
#include "motor_params.h"
#include "motor_app.h"

/* ---- pwm_info: 打印 TMR1 PWM 配置 ---- */
static void pwm_info(int argc, char **argv)
{
    (void)argc; (void)argv;
    rt_kprintf("=== TMR1 PWM ===\n");
    rt_kprintf("clock    : %u MHz\n", TMR1_CLOCK_HZ / 1000000u);
    rt_kprintf("ARR      : %u\n", TMR1_ARR);
    rt_kprintf("freq     : %u Hz (center-aligned TWO_WAY_3)\n", PWM_FREQUENCY_HZ);
    rt_kprintf("duty_max : %u (95%%)\n", PWM_DUTY_MAX);
    rt_kprintf("CCR1/2/3 : %u / %u / %u\n",
               TMR1->c1dt, TMR1->c2dt, TMR1->c3dt);
    rt_kprintf("CCR4     : %u (ADC top trigger)\n", TMR1->c4dt);
    rt_kprintf("EN(PB10) : %u\n",
               (gpio_input_data_bit_read(PWM_EN_GPIO_PORT, PWM_EN_PIN) ? 1 : 0));
}
MSH_CMD_EXPORT(pwm_info, show TMR1 PWM config and current duty);

/* ---- pwm_duty <u> <v> <w>: 手动设置三相占空比 ticks (限幅) ---- */
static void pwm_duty(int argc, char **argv)
{
    uint16_t u, v, w;
    if (argc != 4) {
        rt_kprintf("usage: pwm_duty <u> <v> <w>  (ticks 0..%u)\n", TMR1_ARR);
        return;
    }
    u = (uint16_t)strtol(argv[1], NULL, 0);
    v = (uint16_t)strtol(argv[2], NULL, 0);
    w = (uint16_t)strtol(argv[3], NULL, 0);
    motor_pwm_at32m412_set_duty_ticks(u, v, w);
    rt_kprintf("duty set: u=%u v=%u w=%u (clamped to %u)\n", u, v, w, PWM_DUTY_MAX);
}
MSH_CMD_EXPORT(pwm_duty, set 3-phase duty ticks: pwm_duty <u> <v> <w>);

/* ---- pwm_en <0|1>: 控制 MP6540H EN 引脚 ---- */
static void pwm_en(int argc, char **argv)
{
    int on;
    if (argc != 2) {
        rt_kprintf("usage: pwm_en <0|1>  (0=disable MP6540H, 1=enable)\n");
        return;
    }
    on = atoi(argv[1]);
    if (on) {
        motor_pwm_at32m412_enable_output();
        rt_kprintf("MP6540H EN=HIGH (enabled)\n");
    } else {
        motor_pwm_at32m412_disable_output();
        rt_kprintf("MP6540H EN=LOW (disabled)\n");
    }
}
MSH_CMD_EXPORT(pwm_en, control MP6540H EN pin: pwm_en <0|1>);

/* ---- led <0|1>: 控制 LED (PA0, 低电平点亮 - 负极接 IO) ---- */
static void led(int argc, char **argv)
{
    int on;
    if (argc != 2) {
        rt_kprintf("usage: led <0|1>\n");
        return;
    }
    on = atoi(argv[1]);
    board_led_at32m412_set(on ? true : false);
    rt_kprintf("LED=%d\n", on);
}
MSH_CMD_EXPORT(led, control LED: led <0|1>);

/* ---- mc_state: 打印电机控制状态机 ---- */
static void mc_state(int argc, char **argv)
{
    const motor_control_t *mc;
    (void)argc; (void)argv;
    mc = motor_app_get_control();
    rt_kprintf("=== motor_control ===\n");
    rt_kprintf("state : %d\n", (int)motor_control_get_state(mc));
    rt_kprintf("mode  : %d\n", (int)motor_control_get_mode(mc));
    rt_kprintf("fault : 0x%08X\n", (unsigned)fault_manager_get());
}
MSH_CMD_EXPORT(mc_state, show motor control state/mode/fault);

/* ---- mc_open <vd_mv> <speed_rpm_elec> [enc|ramp]: 启动开环旋转 (Stage 2/4) ----
 * vd_mv          : d 轴目标电压 (毫伏), 典型 500..3000, 上限 18000
 * speed_rpm_elec : 电角度转速 (rpm), 正=正转 负=反转
 *   例: mc_open 1000 300        (纯斜坡, Stage 2 行为)
 *       mc_open 1000 300 enc    (用编码器电角度, Stage 4 验证编码器)
 * 安全: 启动前确认限流电源已接, MP6540H EN 会自动拉高.
 * 注意: 参数用整数毫伏, 因 rt_kprintf 不支持 %f.
 */
static void mc_open(int argc, char **argv)
{
    long vd_mv;
    long rpm_elec;
    float vd_volts;
    float rad_per_s;
    int ret;
    bool use_enc = false;
    if (argc < 3 || argc > 4) {
        rt_kprintf("usage: mc_open <vd_mv> <speed_rpm_elec> [enc|ramp]\n");
        rt_kprintf("  vd_mv: d-axis voltage in mV (0..18000), typ 500..3000\n");
        rt_kprintf("  speed_rpm_elec: electrical rpm (sign=direction)\n");
        rt_kprintf("  enc: use encoder electrical angle (Stage 4)\n");
        return;
    }
    vd_mv     = strtol(argv[1], NULL, 0);
    rpm_elec  = strtol(argv[2], NULL, 0);
    vd_volts  = (float)vd_mv / 1000.0f;
    rad_per_s = (float)rpm_elec * 6.28318530718f / 60.0f;

    /* Stage 4: 可选第三参数 enc/ramp */
    if (argc == 4) {
        if (argv[3][0] == 'e' || argv[3][0] == 'E') {
            use_enc = true;
        } else if (argv[3][0] == 'r' || argv[3][0] == 'R') {
            use_enc = false;
        } else {
            rt_kprintf("FAIL: third arg must be 'enc' or 'ramp'\n");
            return;
        }
    }
    motor_control_isr_open_loop_set_encoder_angle(use_enc);

    ret = motor_control_isr_open_loop_start(vd_volts, rad_per_s);
    if (ret == 0) {
        rt_kprintf("OPEN_LOOP started: vd=%ld mV, speed=%ld rpm_elec, angle=%s\n",
                   vd_mv, rpm_elec, use_enc ? "encoder" : "ramp");
        rt_kprintf("MP6540H EN=HIGH, TMR1_OVF IRQ enabled (16kHz ISR)\n");
    } else if (ret == -1) {
        rt_kprintf("FAIL: fault not cleared. Run 'fault_clear' first.\n");
    } else {
        rt_kprintf("FAIL: param out of range. vd=[0..18000] mV, speed=[-600..600] rpm_elec\n");
    }
}
MSH_CMD_EXPORT(mc_open, start open-loop: mc_open <vd_mv> <speed_rpm_elec> [enc|ramp]);

/* ---- mc_stop: 停止所有模式, 切回 DISABLED, 关 MP6540H ---- */
static void mc_stop(int argc, char **argv)
{
    (void)argc; (void)argv;
    motor_control_isr_current_stop();      /* Stage 5: 停电流环 */
    motor_control_isr_align_stop();        /* Stage 4: 停 ALIGN */
    motor_control_isr_open_loop_stop();    /* Stage 2: 停开环 */
    rt_kprintf("all modes stopped. MP6540H EN=LOW, PWM=50%%, state=DISABLED\n");
}
MSH_CMD_EXPORT(mc_stop, stop all motor modes and disable MP6540H);

/* ---- mc_debug: 打印 ISR 内部状态 (定点: mrad/mV, 因 kprintf 不支持 %f) ---- */
static void mc_debug(int argc, char **argv)
{
    motor_control_isr_debug_t dbg;
    (void)argc; (void)argv;
    motor_control_isr_get_debug(&dbg);
    rt_kprintf("=== FOC ISR debug ===\n");
    rt_kprintf("active    : %d\n", motor_control_isr_open_loop_active() ? 1 : 0);
    rt_kprintf("theta_e   : %ld mrad (%ld mdeg)\n",
               (long)dbg.theta_mrad, (long)(dbg.theta_mrad * 180 / 3141));
    rt_kprintf("v_alpha   : %ld mV\n", (long)dbg.v_alpha_mv);
    rt_kprintf("v_beta    : %ld mV\n", (long)dbg.v_beta_mv);
    rt_kprintf("CCR1/2/3  : %u / %u / %u\n", dbg.ta, dbg.tb, dbg.tc);
    rt_kprintf("tick_count: %lu (%lu ms)\n", dbg.tick_count, dbg.tick_count / 16u);
    rt_kprintf("branch hits: ol=%lu fault=%lu disabled=%lu\n",
               dbg.ol_branch_hits, dbg.fault_hits, dbg.disabled_hits);
    rt_kprintf("current   : ia=%ld ib=%ld ic=%ld mA\n",
               (long)dbg.ia_ma, (long)dbg.ib_ma, (long)dbg.ic_ma);
    rt_kprintf("adc_raw   : ia=%u ib=%u ic=%u\n",
               dbg.ia_raw, dbg.ib_raw, dbg.ic_raw);
    rt_kprintf("vbus      : %ld mV\n", (long)dbg.vbus_mv);
    rt_kprintf("protect   : oc=%lu imbal=%lu\n",
               dbg.oc_hits, dbg.imbal_hits);
    rt_kprintf("encoder   : raw=%u theta=%ld mrad (%ld mdeg) err=%u alive=%d\n",
               dbg.enc_raw, (long)dbg.enc_theta_mrad,
               (long)(dbg.enc_theta_mrad * 180 / 3141),
               dbg.enc_errors, (int)dbg.enc_alive);
    rt_kprintf("align     : hits=%lu active=%d\n",
               dbg.align_hits, motor_control_isr_align_active() ? 1 : 0);
    rt_kprintf("cal       : state=%lu progress=%u%%\n",
               dbg.cal_state, (unsigned)dbg.cal_progress);
    rt_kprintf("cur       : active=%d hits=%lu id=%ldmA iq=%ldmA id_ref=%ldmA iq_ref=%ldmA\n",
               motor_control_isr_current_active() ? 1 : 0,
               (unsigned long)dbg.cur_hits,
               (long)dbg.id_ma, (long)dbg.iq_ma,
               (long)dbg.id_ref_ma, (long)dbg.iq_ref_ma);
}
MSH_CMD_EXPORT(mc_debug, show FOC ISR internal state);

/* ---- mc_current: 打印三相电流 + VBUS 详细 (Stage 3) ---- */
static void mc_current(int argc, char **argv)
{
    motor_control_isr_debug_t dbg;
    uint16_t ofs_a, ofs_b, ofs_c;
    (void)argc; (void)argv;
    motor_control_isr_get_debug(&dbg);
    current_sense_at32m412_get_offset(&ofs_a, &ofs_b, &ofs_c);
    rt_kprintf("=== current sense ===\n");
    rt_kprintf("offset    : a=%u b=%u c=%u (valid=%d)\n",
               ofs_a, ofs_b, ofs_c,
               current_sense_at32m412_offset_valid() ? 1 : 0);
    rt_kprintf("raw       : ia=%u ib=%u ic=%u\n",
               dbg.ia_raw, dbg.ib_raw, dbg.ic_raw);
    rt_kprintf("current   : ia=%ld ib=%ld ic=%ld mA\n",
               (long)dbg.ia_ma, (long)dbg.ib_ma, (long)dbg.ic_ma);
    rt_kprintf("sum       : %ld mA (threshold %ld mA)\n",
               (long)(dbg.ia_ma + dbg.ib_ma + dbg.ic_ma),
               (long)(IMBALANCE_THRESHOLD_A * 1000.0f));
    rt_kprintf("vbus      : %ld mV (%ld.%03ld V)\n",
               (long)dbg.vbus_mv,
               (long)(dbg.vbus_mv / 1000),
               (long)(dbg.vbus_mv % 1000));
}
MSH_CMD_EXPORT(mc_current, show 3-phase current and VBUS detail);

/* ---- mc_cur: 启动电流环 (CURRENT 模式, Stage 5) ----
 * 注: 与 mc_current (电流采样显示) 区分, 本命令启动电流环控制.
 *     mc_cur <iq_ma> [enc|ramp] [speed_rpm_elec]
 *       iq_ma           : Iq 目标电流 (毫安), 范围 +-IQ_MAX_MA. Id 目标恒 0.
 *       enc|ramp        : theta 来源. enc=编码器电角度 (需有效标定),
 *                         ramp=斜坡递增 (调试用, 无需标定). 默认 enc.
 *       speed_rpm_elec  : 仅 ramp 有效, 斜坡角速度 (电角度 rpm), 默认 0 (锁定方向).
 */
static void mc_cur(int argc, char **argv)
{
    long iq_ma;
    float iq_A;
    bool use_enc = true;
    long speed_rpm = 0;

    if (argc < 2) {
        rt_kprintf("usage: mc_cur <iq_ma> [enc|ramp] [speed_rpm_elec]\n");
        return;
    }
    iq_ma = strtol(argv[1], NULL, 0);
    if (iq_ma > IQ_MAX_MA || iq_ma < -IQ_MAX_MA) {
        rt_kprintf("FAIL: iq_ma range +-%d\n", IQ_MAX_MA);
        return;
    }
    iq_A = (float)iq_ma / 1000.0f;

    if (argc >= 3) {
        if (rt_strcmp(argv[2], "ramp") == 0) {
            use_enc = false;
        } else if (rt_strcmp(argv[2], "enc") == 0) {
            use_enc = true;
        } else {
            rt_kprintf("FAIL: mode must be enc or ramp\n");
            return;
        }
    }
    if (argc >= 4 && !use_enc) {
        speed_rpm = strtol(argv[3], NULL, 0);
    }

    /* 前置检查 (shell 层, 不进 ISR) */
    if (fault_manager_any_fatal()) {
        rt_kprintf("FAIL: fault active, clear first (fault_clear)\n");
        return;
    }
    if (use_enc && !motor_calibration_is_valid()) {
        rt_kprintf("FAIL: cal invalid, run mc_calibrate or use ramp mode\n");
        return;
    }

    /* 设 theta 来源 + 速度 (须在 start 前设, current_start 不重置以免覆盖) */
    motor_control_isr_current_set_encoder_angle(use_enc);
    if (!use_enc) {
        float rad_per_s = (float)speed_rpm * 6.28318530718f / 60.0f;
        motor_control_isr_current_set_speed(rad_per_s);
    }

    /* 启动 */
    if (motor_control_isr_current_start(iq_A) != 0) {
        rt_kprintf("FAIL: current start failed (fault or iq out of range)\n");
        return;
    }
    rt_kprintf("current loop: iq_ref=%ldmA theta=%s%s\n",
               (long)iq_ma, use_enc ? "enc" : "ramp",
               (!use_enc && speed_rpm != 0) ? " spinning" : "");
}
MSH_CMD_EXPORT(mc_cur, start current loop: mc_cur <iq_ma> [enc|ramp] [rpm_elec]);

/* ---- mc_cal: 零偏标定 (PWM 50% 时采 1024 次平均, spec §4.3.3) ----
 * 前置条件: PWM 已输出 50% (mc_stop 或开机默认), MP6540H 可使能或禁用.
 * 标定期间 ISR 若未启动, 用软件触发读取; 若已启动, 直接读注入结果.
 * 建议: 先 mc_stop (确保 50% + DISABLED), 再 mc_cal.
 */
static void mc_cal(int argc, char **argv)
{
    bool ok;
    uint16_t ofs_a, ofs_b, ofs_c;
    (void)argc; (void)argv;

    /* 标定前确保 PWM 50% 三相同电位 (无电流) */
    motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);

    /* MP6540H 必须使能 (EN=HIGH): 电流镜需要芯片上电才能输出有效 V_REF.
     * 50% 占空比时三相同电位, 无电流流过, SO 输出 = V_REF (2048 LSB). */
    motor_pwm_at32m412_enable_output();
    rt_kprintf("PWM=50%%, MP6540H EN=HIGH, calibrating offset (1024 samples)...\n");

    /* 开环激活时拒绝标定 (会读到带电流的数据) */
    if (motor_control_isr_open_loop_active()) {
        rt_kprintf("FAIL: open-loop active. Run 'mc_stop' first.\n");
        motor_pwm_at32m412_disable_output();
        return;
    }

    /* 启动 ISR 以驱动 ADC 注入序列 (TMR1_CH4 触发), 标定完停止.
     * state=DISABLED 时 ISR 输出 50%, 不出力. */
    motor_pwm_at32m412_enable_ovf_irq();
    rt_thread_mdelay(100);   /* 等待 ADC 稳定 + MP6540H 上电 */

    ok = current_sense_at32m412_calibrate_offset();

    motor_pwm_at32m412_disable_ovf_irq();
    motor_pwm_at32m412_disable_output();   /* 标定完禁用 MP6540H */

    current_sense_at32m412_get_offset(&ofs_a, &ofs_b, &ofs_c);
    if (ok) {
        rt_kprintf("offset OK: a=%u b=%u c=%u (deviation < %u LSB)\n",
                   ofs_a, ofs_b, ofs_c, 50u);
    } else {
        rt_kprintf("offset FAIL: a=%u b=%u c=%u (deviation > %u LSB from 2048)\n",
                   ofs_a, ofs_b, ofs_c, 50u);
        rt_kprintf("check hardware: SOA/SOB/SOC wiring, MP6540H EN, VREF divider\n");
    }
}
MSH_CMD_EXPORT(mc_cal, calibrate current zero offset (PWM 50%%));

/* ---- fault: 打印故障位 ---- */
static void fault(int argc, char **argv)
{
    uint32_t f;
    (void)argc; (void)argv;
    f = fault_manager_get();
    rt_kprintf("fault = 0x%08X\n", (unsigned)f);
    if (f == 0) {
        rt_kprintf("  (none)\n");
        return;
    }
    if (f & FAULT_DRIVER)       rt_kprintf("  DRIVER (MP6540H nFAULT)\n");
    if (f & FAULT_OVERCURRENT)  rt_kprintf("  OVERCURRENT\n");
    if (f & FAULT_SENSOR)       rt_kprintf("  SENSOR\n");
    if (f & FAULT_UNDERVOLTAGE) rt_kprintf("  UNDERVOLTAGE\n");
    if (f & FAULT_OVERVOLTAGE)  rt_kprintf("  OVERVOLTAGE\n");
    if (f & FAULT_CAN_TIMEOUT)  rt_kprintf("  CAN_TIMEOUT\n");
    if (f & FAULT_CAL_INVALID)  rt_kprintf("  CAL_INVALID\n");
}
MSH_CMD_EXPORT(fault, show fault flags);

/* ---- fault_clear: 清除所有故障 ---- */
static void fault_clear(int argc, char **argv)
{
    (void)argc; (void)argv;
    fault_manager_clear_all();
    rt_kprintf("all faults cleared\n");
}
MSH_CMD_EXPORT(fault_clear, clear all fault flags);

/* ---- encoder: 打印 MA600A 编码器数据 (定点, 因 kprintf 不支持 %f) ---- */
static void encoder(int argc, char **argv)
{
    uint16_t raw16;
    uint16_t zero;
    int32_t  theta_mrad;
    int32_t  angle_mdeg;
    (void)argc; (void)argv;

    raw16 = motor_encoder_get_last_raw();
    zero  = motor_encoder_get_zero();
    /* 电角度 (毫弧度): ISR 快照更准, 但此处独立算一次供验证 */
    theta_mrad = (int32_t)(motor_encoder_to_electrical_angle(raw16) * 1000.0f);
    angle_mdeg = (int32_t)((uint32_t)raw16 * 360000u / 65536u);

    rt_kprintf("=== MA600A ===\n");
    rt_kprintf("raw_16bit : %u\n", raw16);
    rt_kprintf("raw_12bit : %u\n", (unsigned)(raw16 >> 4));
    rt_kprintf("angle_mdeg: %ld (%ld.%03ld deg)\n",
               (long)angle_mdeg,
               (long)(angle_mdeg / 1000),
               (long)(angle_mdeg % 1000));
    rt_kprintf("elec_mrad : %ld (%ld mdeg)\n",
               (long)theta_mrad, (long)(theta_mrad * 180 / 3141));
    rt_kprintf("zero_raw  : %u\n", zero);
    rt_kprintf("errors    : %u\n", motor_encoder_get_error_count());
    rt_kprintf("alive     : %d\n", motor_encoder_is_alive() ? 1 : 0);
    rt_kprintf("cal_valid : %d\n", motor_calibration_is_valid() ? 1 : 0);
}
MSH_CMD_EXPORT(encoder, show MA600A encoder angle and speed);

/* ---- vbus: 独立读取母线电压 (软件触发, 不依赖 ISR) ---- */
static void vbus(int argc, char **argv)
{
    float v;
    uint16_t raw;
    (void)argc; (void)argv;
    raw = current_sense_at32m412_read_vbus_raw();
    v = (float)raw * VBUS_VOLTS_PER_LSB;
    rt_kprintf("vbus_raw : %u\n", raw);
    rt_kprintf("vbus     : %ld mV (%ld.%03ld V)\n",
               (long)(v * 1000.0f),
               (long)(v),
               (long)(v * 1000.0f) % 1000);
    rt_kprintf("threshold: uv=%ld mV  ov=%ld mV\n",
               (long)(VBUS_UNDERVOLTAGE_THRESHOLD_V * 1000.0f),
               (long)(VBUS_OVERVOLTAGE_THRESHOLD_V * 1000.0f));
}
MSH_CMD_EXPORT(vbus, read VBUS voltage (software triggered));

/* ===== Stage 4: ALIGN 零点对齐 + 手动零点 + 旁轴标定 ===== */

/* ---- mc_align <vd_mv>: 启动 ALIGN 模式, 转子对齐 d 轴 0° (spec §4.5.3) ----
 * vd_mv: d 轴锁定电压 (毫伏), 典型 500..2000, 上限 18000
 * 持续期间转子被强制对齐, 手转有阻力. 停止用 mc_stop.
 * ALIGN 结束后用 mc_zero 读取对齐角度并写入零点.
 */
static void mc_align(int argc, char **argv)
{
    long vd_mv;
    float vd_volts;
    int ret;
    if (argc != 2) {
        rt_kprintf("usage: mc_align <vd_mv>\n");
        rt_kprintf("  vd_mv: d-axis lock voltage in mV (0..18000), typ 500..2000\n");
        return;
    }
    vd_mv = strtol(argv[1], NULL, 0);
    vd_volts = (float)vd_mv / 1000.0f;
    ret = motor_control_isr_align_start(vd_volts);
    if (ret == 0) {
        rt_kprintf("ALIGN started: vd=%ld mV. Rotor locking to d-axis 0deg.\n", vd_mv);
        rt_kprintf("Wait 500ms, then 'mc_zero' to capture alignment angle.\n");
        rt_kprintf("Stop with 'mc_stop'.\n");
    } else if (ret == -1) {
        rt_kprintf("FAIL: fault not cleared. Run 'fault_clear' first.\n");
    } else {
        rt_kprintf("FAIL: vd out of range [0..18000] mV\n");
    }
}
MSH_CMD_EXPORT(mc_align, start ALIGN mode: mc_align <vd_mv>);

/* ---- mc_zero [raw16]: 读取或设置零点 (mech_zero_raw, 16-bit) ----
 * 无参: 显示当前零点 + ALIGN 采集到的对齐角度
 * 有参: 手动设置零点 (raw 16-bit, 0..65535)
 */
static void mc_zero(int argc, char **argv)
{
    if (argc == 1) {
        uint16_t align_angle;
        align_angle = motor_control_isr_get_align_angle();
        rt_kprintf("zero_raw    : %u\n", motor_encoder_get_zero());
        rt_kprintf("align_angle : %u (from ALIGN, 0 if ALIGN not done)\n", align_angle);
        rt_kprintf("To set: mc_zero <raw16>\n");
    } else {
        long raw;
        raw = strtol(argv[1], NULL, 0);
        if (raw < 0 || raw > 65535) {
            rt_kprintf("FAIL: raw must be 0..65535\n");
            return;
        }
        motor_encoder_set_zero((uint16_t)raw);
        rt_kprintf("zero set to %u\n", (unsigned)raw);
    }
}
MSH_CMD_EXPORT(mc_zero, get/set encoder zero: mc_zero [raw16]);

/* ---- mc_calibrate: 触发旁轴非线性标定 (spec §4.7.5, Stage 4b) ----
 * 流程: ALIGN 对齐 -> 正转 5 圈 -> 反转 5 圈 -> 计算表 -> 写 FLASH
 * 全程约 25s. 期间电机自动启停. 进度用 mc_cal_status 查看.
 * 中止: fault_clear 后 mc_cal_status 显示 ABORTED, 旧标定保留.
 */
static void mc_calibrate(int argc, char **argv)
{
    (void)argc; (void)argv;
    if (fault_manager_any_fatal()) {
        rt_kprintf("FAIL: fault not cleared. Run 'fault_clear' first.\n");
        return;
    }
    if (motor_control_isr_open_loop_active() || motor_control_isr_align_active()) {
        rt_kprintf("FAIL: motor running. Run 'mc_stop' first.\n");
        return;
    }
    motor_calibration_start_mode(CAL_MODE_AUTO_OPEN_LOOP);
    rt_kprintf("Calibration started. ~25s (ALIGN 0.5s + FWD 10s + REV 10s + compute + flash).\n");
    rt_kprintf("Progress: 'mc_cal_status'. Motor will spin automatically.\n");
}
MSH_CMD_EXPORT(mc_calibrate, start off-axis calibration);

/* ---- mc_cal_status: 打印标定状态/进度/残差 (Stage 4b) ---- */
static void mc_cal_status(int argc, char **argv)
{
    static const char *state_names[] = {
        "IDLE", "ZERO_ALIGN", "SPIN_FWD", "SPIN_REV",
        "COMPUTE", "WRITE_FLASH", "DONE", "ABORTED"
    };
    cal_state_t st;
    (void)argc; (void)argv;
    st = motor_calibration_get_state();
    rt_kprintf("=== calibration ===\n");
    rt_kprintf("state     : %d (%s)\n", (int)st,
               (st <= CAL_STATE_ABORTED) ? state_names[st] : "?");
    rt_kprintf("progress  : %u%%\n", (unsigned)motor_calibration_get_progress());
    rt_kprintf("valid     : %d\n", motor_calibration_is_valid() ? 1 : 0);
    rt_kprintf("max_resid : %ld mdeg (%ld.%03ld deg)\n",
               (long)motor_calibration_get_max_residual(),
               (long)(motor_calibration_get_max_residual() / 1000),
               (long)(motor_calibration_get_max_residual() % 1000));
    rt_kprintf("threshold : %ld mdeg\n", (long)CAL_MAX_RESIDUAL_MDEG);
}
MSH_CMD_EXPORT(mc_cal_status, show calibration state and progress);

/* ---- mc_cal_dump: 打印 256 点校正表 (每行 8 点, 单位 0.001°) ---- */
static void mc_cal_dump(int argc, char **argv)
{
    const int16_t *table;
    uint16_t i;
    (void)argc; (void)argv;
    if (!motor_calibration_is_valid()) {
        rt_kprintf("calibration not valid. Run 'mc_calibrate' first.\n");
        return;
    }
    table = motor_calibration_get_table();
    rt_kprintf("=== calibration table (256 pts, unit 0.001 deg) ===\n");
    for (i = 0u; i < CAL_TABLE_POINTS; i++) {
        rt_kprintf("%6d", (int)table[i]);
        if ((i & 7u) == 7u || i == CAL_TABLE_POINTS - 1u) {
            rt_kprintf("\n");
        }
    }
}
MSH_CMD_EXPORT(mc_cal_dump, dump 256-point calibration table);

/* ---- mc_cal_erase: 擦除 FLASH 标定区 (Stage 4b) ----
 * 擦除后重启会触发 FAULT_CAL_INVALID, 可重新标定.
 */
static void mc_cal_erase(int argc, char **argv)
{
    bool ok;
    (void)argc; (void)argv;
    ok = flash_calibration_erase();
    if (ok) {
        rt_kprintf("FLASH calibration sector erased (0x0801FC00).\n");
        rt_kprintf("Reboot -> FAULT_CAL_INVALID will be set.\n");
    } else {
        rt_kprintf("FAIL: erase failed (flash protected or timeout)\n");
    }
}
MSH_CMD_EXPORT(mc_cal_erase, erase FLASH calibration sector);

/* ---- enc_status: 打印 encoder_service 快照与诊断 ---- */
static void enc_status(int argc, char **argv)
{
    encoder_snapshot_t snap;
    encoder_tracker_snapshot_t trk;
    bool motor_active;

    (void)argc; (void)argv;
    motor_active = motor_control_isr_open_loop_active() ||
                   motor_control_isr_align_active() ||
                   motor_control_isr_current_active();

    if (!motor_active) {
        (void)encoder_service_poll_once_thread();
    }

    if (!encoder_service_get_snapshot(&snap)) {
        rt_kprintf("encoder snapshot invalid\n");
        return;
    }

    rt_kprintf("=== encoder service ===\n");
    rt_kprintf("raw16     : %u\n", snap.raw16);
    rt_kprintf("delta     : %d\n", snap.raw_delta);
    rt_kprintf("unwrap    : %ld\n", (long)snap.raw_unwrapped);
    rt_kprintf("mech_mdeg : %ld\n", (long)snap.mech_mdeg);
    rt_kprintf("elec_mrad : %ld\n", (long)snap.elec_mrad);
    rt_kprintf("speed_raw : %d\n", snap.speed_raw);
    rt_kprintf("counts    : sample=%lu accept=%lu bus=%lu spike=%lu stale=%lu\n",
               snap.sample_count, snap.accept_count, snap.bus_error_count,
               snap.spike_count, snap.stale_count);
    rt_kprintf("last_rej  : raw=%u delta=%d\n",
               snap.last_rejected_raw16, snap.last_rejected_delta);
    rt_kprintf("zero_raw  : %u\n", encoder_service_get_zero());
    rt_kprintf("cal_valid : %d\n", motor_calibration_is_valid() ? 1 : 0);

    if (encoder_tracker_get_snapshot(&trk)) {
        rt_kprintf("=== encoder tracker ===\n");
        rt_kprintf("trk_raw   : %u\n", trk.raw16);
        rt_kprintf("trk_theta : %ld mrad\n", (long)trk.elec_mrad);
        rt_kprintf("trk_speed : %ld mrad/s\n", (long)trk.speed_mrad_s);
        rt_kprintf("trk_age   : %lu ticks\n", trk.stale_ticks);
        rt_kprintf("trk_count : %lu\n", trk.sample_count);
    }
}
MSH_CMD_EXPORT(enc_status, show encoder service snapshot and diagnostics);

static void enc_diag_reset(int argc, char **argv)
{
    (void)argc; (void)argv;
    encoder_service_reset_diagnostics();
    rt_kprintf("encoder diagnostics reset\n");
}
MSH_CMD_EXPORT(enc_diag_reset, reset encoder diagnostics);

static void enc_zero(int argc, char **argv)
{
    long raw;

    if (argc == 1) {
        rt_kprintf("zero_raw  : %u\n", encoder_service_get_zero());
        return;
    }
    raw = strtol(argv[1], NULL, 0);
    if ((raw < 0) || (raw > 65535)) {
        rt_kprintf("FAIL: raw must be 0..65535\n");
        return;
    }
    motor_calibration_set_zero((uint16_t)raw);
    rt_kprintf("zero set to %u\n", (unsigned)raw);
}
MSH_CMD_EXPORT(enc_zero, get/set encoder zero: enc_zero [raw16]);

static void enc_cal_start(int argc, char **argv)
{
    cal_mode_t mode;

    mode = CAL_MODE_AUTO_OPEN_LOOP;
    if (argc >= 2) {
        if (rt_strcmp(argv[1], "manual") == 0) {
            mode = CAL_MODE_MANUAL;
        } else if (rt_strcmp(argv[1], "auto") == 0) {
            mode = CAL_MODE_AUTO_OPEN_LOOP;
        } else {
            rt_kprintf("usage: enc_cal_start [auto|manual]\n");
            return;
        }
    }
    motor_calibration_start_mode(mode);
    rt_kprintf("encoder calibration started: %s\n",
               (mode == CAL_MODE_MANUAL) ? "manual" : "auto");
}
MSH_CMD_EXPORT(enc_cal_start, start encoder calibration: enc_cal_start [auto|manual]);

static void enc_cal_stop(int argc, char **argv)
{
    (void)argc; (void)argv;
    motor_calibration_stop_manual();
    rt_kprintf("encoder manual calibration stop requested\n");
}
MSH_CMD_EXPORT(enc_cal_stop, stop manual encoder calibration collection);

static void enc_cal_status(int argc, char **argv)
{
    motor_calibration_quality_t q;

    (void)argc; (void)argv;
    mc_cal_status(0, 0);
    if (motor_calibration_get_quality(&q)) {
        rt_kprintf("quality   : samples=%lu bins=%u min_bin=%u residual=%d ok=%u\n",
                   q.sample_count, q.covered_bins, q.min_bin_count,
                   q.max_residual_mdeg, q.quality_ok);
        rt_kprintf("spikes    : start=%lu end=%lu nonmono=%lu\n",
                   q.spike_count_start, q.spike_count_end, q.nonmonotonic_count);
    }
}
MSH_CMD_EXPORT(enc_cal_status, show encoder calibration status and quality);

static void enc_cal_dump(int argc, char **argv)
{
    mc_cal_dump(argc, argv);
}
MSH_CMD_EXPORT(enc_cal_dump, dump encoder calibration table);
