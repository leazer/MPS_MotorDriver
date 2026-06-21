#include "motor_app.h"
#include "motor_control.h"
#include "fault_manager.h"
#include "motor_calibration.h"
#include "ma600a_debug.h"

/* 全局电机控制实例 (motor_control 接口需要实例指针) */
static motor_control_t s_motor_control;

void motor_app_init(void)
{
    fault_manager_init();
    motor_control_init(&s_motor_control);  /* 已有状态机 */
    motor_calibration_load();  /* 开机加载标定 */
    ma600a_debug_init();       /* 保留 bring-up 路径 */
}

void motor_app_run(void)
{
    while (1) {
        ma600a_debug_poll();
        /* Plan 5 加入 CAN 收发 / 状态上报 */
    }
}
