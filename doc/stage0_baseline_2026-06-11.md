# Stage 0 Baseline - 2026-06-11

本文记录 AT32M412 + MP6540H + MA600A 有感 FOC 控制器项目的 Stage 0 基线状态。后续智能体继续开发前，应先阅读本文件、根目录 `CLAUDE.md` 和 `doc/FOC控制器开发记录.md`。

## 当前工程状态

- 当前分支：`main`。
- 当前工作区已有未提交改动：`.gitignore`、`project/src/main.c`、`CLAUDE.md`、`doc/FOC控制器开发记录.md`、本文件。
- `project/src/main.c` 已包含 MA600A 调试入口：`ma600a_debug_init()` 和 `ma600a_debug_poll()`；该改动不是本次 Stage 0 新写入，后续修改前必须先确认用户意图。
- 当前工程入口仍以 AT Workbench 生成初始化为主：GPIO、ADC2、USART1、CAN1、SPI2、WDT、TMR1、RT-Thread。
- 当前尚未接入 FOC 控制框架；现状适合作为传感器验证和外设基线。

## 构建与测试基线

已执行：

```text
python middlewares\msp\ma600\test_ma600a_static.py
```

结果：

```text
8 ma600a static tests passed
```

PowerShell 直接执行：

```text
cmake --build build\Debug
```

结果：失败，原因是当前 PowerShell 环境找不到 `cmake` 命令。

WSL 执行：

```text
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_电机驱动/MPS_MotorDriver && cmake --build build/Debug"
```

结果：构建通过，生成 `MPS_MotorDriver.elf`。

资源占用：

```text
FLASH: 26504 B / 128 KB, 20.22%
RAM: 3520 B / 16 KB, 21.48%
```

## 当前编译警告分类

当前 GCC/CMake 构建通过，但存在警告。主要来源如下：

- `project/src/rtthread_app.c:105`：`my_task01_func(void *parameter)` 的 `parameter` 未使用。
- `middlewares/rt-thread/libcpu/arm/cortex-m4/cpuport.c:378`：`context` 未使用。原因与 Keil 警告一致：`RT_USING_CONSOLE` 未启用时，`rt_kprintf(...)` 展开为空，导致只为打印准备的局部变量未被引用。
- RT-Thread 源码中大量 `RTM_EXPORT(...)` 在当前 `-Wpedantic` 下被报为 `ISO C does not allow extra ';' outside of a function`。
- RT-Thread 源码中有若干 `unused parameter`、函数指针转换和 `fall through` 警告，属于第三方/中间件代码在严格 GCC 警告配置下暴露的问题。

当前警告处理策略：

- 不急于修改 RT-Thread 第三方源码。
- 如果要清理 GCC 构建噪声，优先在 CMake 中对 RT-Thread 目标降低 `-Wpedantic` 或单独加 suppress 选项。
- Keil 的 `context` 未使用警告可以通过启用 `RT_USING_CONSOLE` 或在 `cpuport.c` 中加 `(void)context;` 处理；但这属于中间件改动，建议等确认日志/控制台策略后再改。
- 项目自有源码中的未使用参数可以按需加 `(void)parameter;`。

## 外设基线结论

- MA600A 驱动位于 `middlewares/msp/ma600/`，已有平台无关驱动和 AT32 SPI2 适配层。
- `ma600a_debug.c` 当前读取角度、速度、BCT、轴向配置，并暴露全局变量用于调试观察。
- 当前 MA600A SPI2 适配为轮询传输，适合 bring-up 和低频调试；闭环 FOC 前需要评估采样时序和实时性。
- Workbench 当前 ADC2 配置为单通道 `ADC_CHANNEL_6`，普通/抢占触发边沿均为 `NONE`，还不是电流环所需的 PWM 同步采样配置。
- Workbench 当前 TMR1 已存在三相 PWM 相关通道初始化，但仍需按 MP6540H 驱动方式确认互补输出、死区、刹车/故障输入、默认安全态。

## 后续 Stage 1 前置条件

进入硬件 bring-up 前必须完成：

1. 固定 MP6540H 引脚映射：PWM/EN/nFAULT/电流采样/母线电压/温度/刹车。
2. 明确 MA600A SPI2 引脚、CS 默认电平、供电电压、SPI 模式、最大时钟。
3. 明确电流采样拓扑：单电阻、双电阻或三电阻；采样放大倍数、零点偏置、ADC 通道。
4. 确认 TMR1 PWM 频率、中心对齐模式、死区时间、输出极性、刹车输入。
5. 确认低压限流上电方案，首次 PWM 输出必须在 MP6540H 禁用或低压限流状态下示波器验证。

## 下一步建议

- 先建立 `motor_control` 与 `platform` 的最小目录框架，只放接口和安全默认状态，不立即闭环。
- 把 Workbench 生成文件作为底层初始化输入，电机控制 ISR、PWM 更新、ADC 采样、MA600A 位置反馈放到独立模块。
- Stage 2 的第一个可验证目标应是：MP6540H 禁用状态下，TMR1 输出符合预期 PWM、死区和默认电平。

## 2026-06-11 Continue Record

已完成最小控制框架落地：

- 新增 `application/motor_control/motor_control.h`。
- 新增 `application/motor_control/motor_control.c`。
- 新增 `platform/at32m412/motor_pwm_at32m412.h`。
- 新增 `platform/at32m412/motor_pwm_at32m412.c`。
- 新增 `tests/motor_control/test_motor_control_state.c`。
- 更新顶层 `CMakeLists.txt`，把上述生产源文件纳入 WSL/CMake 交叉构建。

当前设计边界：

- `motor_control` 只实现状态机和目标值保存，不直接操作寄存器。
- 默认初始化状态为 `MOTOR_CONTROL_STATE_DISABLED`，目标电流、速度、位置均为 0。
- 故障通过 `motor_control_set_fault()` 锁存，进入 `MOTOR_CONTROL_STATE_FAULT` 后会清零目标值。
- 故障未清除前，`motor_control_enable()` 返回 `MOTOR_CONTROL_STATUS_FAULT`。
- `platform/at32m412/motor_pwm_at32m412.c` 只提供安全关断、PWM 使能和三相占空比 ticks 写入接口。
- 当前未在 `main.c` 中调用 `motor_pwm_at32m412_enable_output()`，固件默认不会主动打开 MP6540H。

TDD/验证记录：

1. 先写 `tests/motor_control/test_motor_control_state.c`。
2. 使用 `arm-none-eabi-gcc` 编译测试，确认因缺少 `motor_control.h/.c` 失败。
3. 实现最小 `motor_control` 状态机。
4. 再次执行接口语法验证，通过。

实际通过命令：

```text
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_电机驱动/MPS_MotorDriver && arm-none-eabi-gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control -fsyntax-only tests/motor_control/test_motor_control_state.c application/motor_control/motor_control.c"
```

交叉构建结果：

```text
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_电机驱动/MPS_MotorDriver && cmake --build build/Debug"
```

结果：构建通过，仍有既有 `project/src/rtthread_app.c:105` unused parameter 警告。

资源占用：

```text
FLASH: 26504 B / 128 KB, 20.22%
RAM: 3520 B / 16 KB, 21.48%
```

已知限制：

- 当前 WSL 环境没有主机 `gcc`，因此测试只能用 `arm-none-eabi-gcc -fsyntax-only` 做接口语法验证，不能直接运行 `assert()` 可执行文件。
- 后续如果需要主机单元测试，应安装主机 GCC 或引入适合 Windows/WSL 的 C 测试环境。
- `motor_pwm_at32m412_set_duty_ticks()` 当前未做周期上限裁剪，Stage 2 接入真实 PWM 前必须补充基于 TMR1 period 的限幅。

下一步建议：

1. 读取并确认 MP6540H 原理图引脚映射。
2. 给 `platform/at32m412` 增加只读硬件映射说明或 `board_motor_pins.h`。
3. 明确 TMR1 周期、PWM 频率、计数模式、死区和输出极性。
4. 在 MP6540H 禁用状态下做 PWM 示波器验证后，再允许调用 `motor_pwm_at32m412_enable_output()`。
