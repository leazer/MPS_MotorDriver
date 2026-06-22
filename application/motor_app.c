#include "motor_app.h"
#include <rtthread.h>   /* rt_thread_mdelay */
#include "motor_control.h"
#include "fault_manager.h"
#include "motor_calibration.h"
#include "motor_pwm_at32m412.h"
#include "current_sense_at32m412.h"
#include "motor_encoder_at32m412.h"
#include "ma600a_debug.h"

/* 全局电机控制实例 (motor_control 接口需要实例指针) */
static motor_control_t s_motor_control;

const motor_control_t *motor_app_get_control(void)
{
    return &s_motor_control;
}

motor_control_t *motor_app_get_control_rw(void)
{
    return &s_motor_control;
}

void motor_app_init(void)
{
    /* 板级初始化 (时钟/GPIO/NVIC) 已在 rt_hw_board_init 完成, 此处不重复 */

    /* PWM: TMR1 中心对齐 16kHz, 初始 50% 三相同电位, MP6540H EN 保持低 */
    motor_pwm_at32m412_safe_init();

    /* Stage 3: ADC2 注入序列初始化 (TMR1_CH4 顶点触发, 需在 PWM 之后).
     * 初始化后 ADC 开始由硬件触发转换, 但 ISR 未启动, 结果不会被读取. */
    current_sense_at32m412_init();

    /* Stage 4: MA600A 编码器初始化 (SPI2 硬件 + ma600a_init).
     * 需在 board_clock_init (已开 GPIOB) 之后, SPI2 时钟由本函数开启. */
    motor_encoder_at32m412_init();

    /* 应用层模块 */
    fault_manager_init();
    motor_control_init(&s_motor_control);  /* 已有状态机 */
    motor_calibration_load();              /* 开机加载标定 (Stage 4b) */

    /* MA600A 调试路径 (轮询读角度到全局变量, 供 encoder 命令观察).
     * Stage 4 起 ISR 也会读编码器, 此处保留作为非 ISR 上下文的独立观察路径. */
    ma600a_debug_init();
}

void motor_app_run(void)
{
    while (1) {
        /* Stage 4b: 标定状态机推进 (线程上下文, 处理 ALIGN 等待/COMPUTE/WRITE_FLASH) */
        motor_calibration_poll();

        /* MA600A 调试轮询 (非 ISR 上下文, 更新全局变量供 encoder 命令) */
        ma600a_debug_poll();

        /* 让出 CPU 给 finsh 线程 (优先级 21, 低于 main 的 10).
         * main 线程若死循环不让出, finsh 线程得不到调度, msh 提示符不出现. */
        rt_thread_mdelay(10);
    }
}
