#ifndef MOTOR_APP_H
#define MOTOR_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* 应用层初始化: 加载标定, 初始化各模块 */
void motor_app_init(void);

/* 应用层主循环 (在 main while(1) 内调用, 内部可 rt_thread_mdelay) */
void motor_app_run(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_APP_H */
