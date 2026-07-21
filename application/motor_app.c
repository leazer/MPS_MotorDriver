#include "motor_app.h"
#include <limits.h>
#include <rtthread.h>   /* rt_thread_mdelay */
#include "motor_control.h"
#include "fault_manager.h"
#include "motor_calibration.h"
#include "current_loop.h"
#include "motor_control_isr.h"
#include "speed_loop.h"
#include "position_loop.h"
#include "can_motion_service.h"
#include "joint_config_service.h"
#include "can_at32m412.h"
#include "motor_pwm_at32m412.h"
#include "current_sense_at32m412.h"
#include "motor_encoder_at32m412.h"
#include "encoder_service.h"
#include "encoder_tracker.h"
#include "motor_params.h"
#include "can_motion_timer_at32m412.h"
#include "encoder_acq_timer_at32m412.h"

#define MOTOR_APP_RAD_S_TO_MDEG_S (180000.0f / 3.14159265359f)

/* 全局电机控制实例 (motor_control 接口需要实例指针) */
static motor_control_t s_motor_control;
static bool s_can_init_attempted;
static bool s_can_ready;

static bool motor_app_can_rx_pop(can_frame_t *out)
{
    return can_at32m412_rx_pop(out);
}

static bool motor_app_can_tx_push(const can_frame_t *frame)
{
    return can_at32m412_tx_push(frame);
}

static int motor_app_can_position_start(const position_setpoint_t *setpoint)
{
    return motor_control_isr_position_start(setpoint);
}

static int motor_app_can_position_submit(const position_setpoint_t *setpoint)
{
    return motor_control_isr_position_submit(setpoint);
}

static void motor_app_can_position_stop(void)
{
    motor_control_isr_position_stop();
}

static int32_t motor_app_can_position_mdeg(void)
{
    return position_loop_sensor_to_joint_mdeg(
        encoder_service_get_control_position_mdeg());
}

static int32_t motor_app_can_velocity_mdeg_s(void)
{
    float velocity_mdeg_s;
    int32_t control_velocity_mdeg_s;

    velocity_mdeg_s = encoder_tracker_get_speed_rad_s() *
                      MOTOR_APP_RAD_S_TO_MDEG_S /
                      (float)MOTOR_POLE_PAIRS;
    if (velocity_mdeg_s != velocity_mdeg_s) {
        control_velocity_mdeg_s = 0;
    } else if (velocity_mdeg_s >= (float)INT32_MAX) {
        control_velocity_mdeg_s = INT32_MAX;
    } else if (velocity_mdeg_s <= (float)INT32_MIN) {
        control_velocity_mdeg_s = INT32_MIN;
    } else {
        control_velocity_mdeg_s = (int32_t)velocity_mdeg_s;
    }
    return position_loop_control_to_joint_velocity_mdeg_s(
        control_velocity_mdeg_s);
}

static uint16_t motor_app_can_vbus_10mv(void)
{
    float vbus;

    vbus = current_sense_at32m412_read_vbus();
    if (!(vbus > 0.0f)) {
        return 0u;
    }
    if (vbus >= ((float)UINT16_MAX / 100.0f)) {
        return UINT16_MAX;
    }
    return (uint16_t)(vbus * 100.0f + 0.5f);
}

static uint32_t motor_app_can_fault_get(void)
{
    return fault_manager_get();
}

static void motor_app_can_fault_set(uint32_t bits)
{
    fault_manager_set_bits(bits);
}

static void motor_app_can_fault_clear(void)
{
    fault_manager_clear_bits(FAULT_CAN_TIMEOUT | FAULT_CAN_BUS);
}

static const can_motion_ops_t s_can_motion_ops = {
    motor_app_can_rx_pop,
    motor_app_can_tx_push,
    motor_app_can_position_start,
    motor_app_can_position_submit,
    motor_app_can_position_stop,
    motor_app_can_position_mdeg,
    motor_app_can_velocity_mdeg_s,
    motor_app_can_vbus_10mv,
    motor_app_can_fault_get,
    motor_app_can_fault_set,
    motor_app_can_fault_clear
};

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

    /* Stage 3: ADC2 注入序列初始化 (TMR1_CH4 近顶点低边窗口触发, 需在 PWM 之后).
     * 初始化后 ADC 开始由硬件触发转换, 但 ISR 未启动, 结果不会被读取. */
    current_sense_at32m412_init();

    /* Stage 4: MA600A 编码器初始化 (SPI2 硬件 + ma600a_init).
     * 需在 board_clock_init (已开 GPIOB) 之后, SPI2 时钟由本函数开启. */
    motor_encoder_at32m412_init();
    encoder_service_init();
    encoder_tracker_init();

    /* 应用层模块 */
    fault_manager_init();
    motor_control_init(&s_motor_control);  /* 已有状态机 */
    motor_calibration_load();              /* 开机加载标定 (Stage 4b) */
    current_loop_init();                   /* Stage 5: 电流环 PID 参数初始化 */
    motor_control_isr_sampling_init();     /* 电流重构保护与诊断初始化 */
    speed_loop_init();                     /* Stage 6: 速度环 PI 参数初始化 */
    position_loop_init();                  /* Stage 7: 位置环与运行时关节零点 */
    joint_config_service_init();           /* 恢复持久化关节坐标 */
    can_motion_service_init(&s_can_motion_ops);
    s_can_init_attempted = false;
    s_can_ready = false;
    can_motion_timer_at32m412_init();       /* 始终运行的 1kHz CAN 服务 */
    encoder_acq_timer_at32m412_init();     /* 4kHz 低优先级编码器采集 */
}

void motor_app_run(void)
{
    while (1) {
        can_at32m412_diag_t can_diag;

        /* Stage 4b: 标定状态机推进 (线程上下文, 处理 ALIGN 等待/COMPUTE/WRITE_FLASH) */
        motor_calibration_poll();
        joint_config_service_poll();

        if (!s_can_init_attempted) {
            uint8_t node_id;

            if (joint_config_service_lock_runtime(&node_id)) {
                s_can_init_attempted = true;
                if (can_at32m412_init(node_id)) {
                    s_can_ready = true;
                    can_motion_service_set_joint_config(true, node_id);
                } else {
                    can_motion_service_force_stop();
                }
            }
        }

        if (s_can_ready) {
            can_at32m412_get_diag(&can_diag);
            if (can_diag.fatal_latched) {
                can_motion_service_force_stop();
            }
            can_motion_service_poll_tx();
            can_at32m412_tx_kick();
        }

        /* 让出 CPU 给 finsh 线程 (优先级 21, 低于 main 的 10).
         * main 线程若死循环不让出, finsh 线程得不到调度, msh 提示符不出现. */
        rt_thread_mdelay(10);
    }
}
