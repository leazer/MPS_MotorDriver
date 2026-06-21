#include "motor_app.h"
#include "motor_control.h"
#include "fault_manager.h"
#include "motor_calibration.h"
#include "board_init_at32m412.h"
#include "motor_pwm_at32m412.h"
#include "ma600a_debug.h"

/* 全局电机控制实例 (motor_control 接口需要实例指针) */
static motor_control_t s_motor_control;

void motor_app_init(void)
{
    /* 板级初始化: 外设时钟 + GPIO + NVIC 优先级 (spec 方案 Y) */
    board_clock_init();
    board_gpio_init();
    board_nvic_init();

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
    }
}
