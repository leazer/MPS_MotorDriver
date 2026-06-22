#include "motor_app.h"
#include "motor_control.h"
#include "fault_manager.h"
#include "motor_calibration.h"
#include "motor_pwm_at32m412.h"
#include "ma600a_debug.h"

/* 全局电机控制实例 (motor_control 接口需要实例指针) */
static motor_control_t s_motor_control;

const motor_control_t *motor_app_get_control(void)
{
    return &s_motor_control;
}

void motor_app_init(void)
{
    /* 板级初始化 (时钟/GPIO/NVIC) 已在 rt_hw_board_init 完成, 此处不重复 */

    /* PWM: TMR1 中心对齐 16kHz, 初始 50% 三相同电位, MP6540H EN 保持低 */
    motor_pwm_at32m412_safe_init();

    /* 应用层模块 */
    fault_manager_init();
    motor_control_init(&s_motor_control);  /* 已有状态机 */
    motor_calibration_load();              /* 开机加载标定 */

    /* MA600A 调试路径: 需要 SPI2 时钟, Stage 4 接编码器时恢复启用
     * ma600a_debug_init();
     */
}

void motor_app_run(void)
{
    while (1) {
        /* MA600A 调试轮询: Stage 4 恢复
         * ma600a_debug_poll();
         */
        /* Plan 5 加入 CAN 收发 / 状态上报 */

        /* 让出 CPU 给 finsh 线程 (优先级 21, 低于 main 的 10).
         * main 线程若死循环不让出, finsh 线程得不到调度, msh 提示符不出现. */
        rt_thread_mdelay(10);
    }
}
