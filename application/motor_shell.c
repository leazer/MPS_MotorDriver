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
#include "motor_pwm_at32m412.h"
#include "motor_control.h"
#include "fault_manager.h"
#include "motor_app.h"
#include "ma600a_debug.h"

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
    if (on) {
        gpio_bits_reset(LED_GPIO_PORT, LED_PIN);   /* 低电平点亮 */
    } else {
        gpio_bits_set(LED_GPIO_PORT, LED_PIN);     /* 高电平熄灭 */
    }
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

/* ---- encoder: 打印 MA600A 编码器数据 ---- */
static void encoder(int argc, char **argv)
{
    (void)argc; (void)argv;
    rt_kprintf("=== MA600A ===\n");
    rt_kprintf("raw_angle : %u\n", g_ma600a_raw_angle);
    rt_kprintf("angle_deg : %.2f\n", g_ma600a_angle_deg);
    rt_kprintf("speed_raw : %d\n", g_ma600a_speed_raw);
    rt_kprintf("speed_rpm : %.2f\n", g_ma600a_speed_rpm);
    rt_kprintf("status    : %d\n", g_ma600a_status);
    rt_kprintf("samples   : %u\n", g_ma600a_sample_count);
    rt_kprintf("errors    : %u\n", g_ma600a_error_count);
    rt_kprintf("bct/axis  : %u / %u\n", g_ma600a_bct, g_ma600a_axis);
}
MSH_CMD_EXPORT(encoder, show MA600A encoder angle and speed);
