#ifndef CLOCK_AT32M412_H
#define CLOCK_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

/* 系统时钟初始化 (96MHz), Plan 1 暂沿用 wk_system_clock_config, Plan 2 重写 */
void clock_at32m412_init(void);

#ifdef __cplusplus
}
#endif

#endif /* CLOCK_AT32M412_H */
