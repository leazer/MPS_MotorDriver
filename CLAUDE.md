# CLAUDE.md

## Latest Stage 0 Baseline - 2026-06-11

新增阶段记录：`doc/stage0_baseline_2026-06-11.md`。

当前基线结论：

- WSL 下 `cmake --build build/Debug` 已构建通过，生成 `MPS_MotorDriver.elf`。
- 资源占用：FLASH 26504 B / 128 KB，RAM 3520 B / 16 KB。
- `python middlewares\msp\ma600\test_ma600a_static.py` 已通过，结果为 `8 ma600a static tests passed`。
- PowerShell 环境当前找不到 `cmake` 命令，构建需走 WSL 或配置 Windows CMake。
- 当前主要警告来自 RT-Thread 与严格 GCC 警告选项：`-Wall -Wextra -Wpedantic`。
- Keil/GCC 均可复现 `cpuport.c` 中 `context` 未使用警告，原因是 `RT_USING_CONSOLE` 未启用后 `rt_kprintf(...)` 展开为空。
- `project/src/main.c` 当前已有 MA600A 调试初始化和轮询读取，后续改动前必须先确认是否保留该 bring-up 路径。
- 已新增最小 `application/motor_control` 状态机和 `platform/at32m412` PWM 安全接口。
- 新增测试 `tests/motor_control/test_motor_control_state.c`，当前通过 `arm-none-eabi-gcc -fsyntax-only` 做接口语法验证；WSL 环境暂无主机 `gcc`，不能直接运行 assert 可执行文件。
- 当前未在 `main.c` 中调用 `motor_pwm_at32m412_enable_output()`，默认不主动使能 MP6540H。

后续继续开发前，先读 `doc/stage0_baseline_2026-06-11.md`，再进入 Stage 1 硬件 bring-up。

本文件用于让智能体在中断或重新进入项目后快速恢复上下文。继续开发前必须阅读本文和 `doc/FOC控制器开发记录.md`。

## Stage 0 Complete - 2026-06-22

Stage 0 (基线整理与目录重构) 已完成. 详见 `docs/superpowers/specs/2026-06-22-mps-foc-design.md` 与 `docs/superpowers/plans/2026-06-22-plan1-project-baseline.md`.

完成内容:
- 新增分层目录骨架: `application/motor_control/` (foc_core/loops/isr/fault/calib), `platform/at32m412/` (current_sense/encoder/protect/flash/clock), `communication/` (can_protocol/can_at32m412)
- 新增 `board_motor_pins.h` (引脚与时序常数集中定义), `motor_params.h` (所有参数宏)
- Workbench `wk_*_init()` 函数体已清空 (保留 `wk_system_clock_config`), 按 spec 方案 Y
- `main.c` 精简为 `clock_at32m412_init / motor_app_init / motor_app_run`
- `CMakeLists.txt` 纳入全部新源, WSL 构建通过
- `AT32M412xB_FLASH.ld` 预留末 1 KB 给标定区 (LENGTH 128K -> 127K)
- 新增单元测试: test_foc_clarke / test_current_sense_calc / test_fault_manager (syntax-check)

资源占用 (Stage 0 完成后): FLASH 14712 B / 127 KB (11.31%), RAM 3552 B / 16 KB (21.68%).

已知限制:
- wk_*_init 已清空, 板上外设不再初始化, 固件烧录后 MA600A 调试路径暂不可用 (预期, Plan 2 重建)
- foc_park / foc_ipark / foc_svpwm / pid_f32_exec / 各 loop_run / isr_tick 均为 stub, Plan 2-4 实现
- DSP 三角函数 (arm_sin_cos_f32) 未接入, Plan 4 处理
- main.c 中 clock_at32m412_init 暂为 wk_system_clock_config 包装 (board.c 内 RT-Thread 启动也会调一次, 幂等无副作用)

后续 Stage 1 前置条件已满足: 引脚映射 / 时序常数 / 参数宏 / 分层骨架全部就位.

## Stage 1 Complete - 2026-06-22

Stage 1 (硬件 Bring-up) 代码完成, 详见 `docs/superpowers/specs/2026-06-22-mps-foc-design.md` §7 Stage 1.

完成内容:
- `board_motor_pins.h` 时序常数适配 180MHz sclk (TMR1_CLOCK_HZ=180M, ARR=5624, 验算 180e6/(2*5625)=16kHz)
- 新增 `board_init_at32m412.[ch]`: 外设时钟 (GPIOA/B + TMR1 + SCFG + PWC) + GPIO (nFAULT/LED/SPI2_CS/PWM_EN) + NVIC 优先级 (组4, TMR1_OVF=0/EXINT2=1/ADC=2/CAN=3/SysTick=14/PendSV=15, 仅设优先级不使能)
- `motor_pwm_at32m412.c` 实现 TMR1 完整初始化: 中心对齐 TWO_WAY_3 + RCR=1 + ARR=5624 + 3路 PWM_MODE_A + CH4(ADC顶点触发预留) + brkdt 禁用 + 50% 初始占空比三相同电位
- `motor_app_init()` 接入调用链: board_clock_init -> board_gpio_init -> board_nvic_init -> motor_pwm_at32m412_safe_init (MP6540H EN 保持低)
- Keil .uvprojx 纳入 board_init_at32m412.c

资源占用 (Stage 1 完成后):
- WSL GCC: FLASH 16440 B / 127 KB (12.64%), RAM 3504 B / 16 KB (21.39%)
- Keil ARMCC -O1: Code 9602 B + RO 806 + RW 184, ZI 3384

关键决策 (与 spec 原文不同):
- 系统时钟保持 wk_system_clock_config 的 180MHz (spec §1.1 原标 96MHz 已更正为 180MHz, 芯片实际规格支持 180MHz, Flash 等待周期 5)
- TMR1_OVF / EXINT2 中断仅设优先级不使能 (各 Stage 实现 ISR 后再 nvic_irq_enable, 避免空 ISR 死循环)
- RT-Thread libcpu 用 PRIMASK (非 BASEPRI), spec §3.5 原写的 RT_KERNEL_BASEPRI 不适用; FOC ISR 屏蔽风险留到 Stage 2/3 实测

已知限制:
- ma600a_debug_init/poll 暂注释 (需 SPI2 时钟, Stage 4 接编码器时恢复)
- 台架示波器验收待执行 (接好板子用 flash.bat 烧录, 看 PA8/PA9/PA10 = 16kHz 中心对齐 50%)
- SPI2/CAN1/USART1/ADC2 时钟未开 (各模块 Stage 初始化时自行开启)

## 调试串口 + msh 命令 - 2026-06-22

finsh/msh 调试控制台已接入, 后续 Stage 可通过 msh 命令自主闭环验证.

### 串口配置
- 物理引脚: PB6 (TX) / PB7 (RX), USART1, MUX_7
- 参数: 115200 8N1, 无流控
- 初始化位置: `platform/at32m412/board_init_at32m412.c` 的 `board_usart1_init()`
- 启动链: `rt_hw_board_init` -> `rt_components_board_init` -> `uart_init()` (INIT_BOARD_EXPORT) -> `board_usart1_init()`
- 提示符: `msh />` (finsh msh 模式)

### 配置开关
- `project/inc/finsh_config.h`: `RT_USING_FINSH` 已开启
- `project/inc/rtconfig.h`: `RT_USING_CONSOLE` 已开启
- `RT_USING_HEAP` 仍关闭 (finsh 走静态分配分支, 够用)
- finsh 源码: `shell.c / msh.c / msh_parse.c / cmd.c` 已纳入 CMake + Keil

### 资源占用 (含 finsh + msh 命令)
- WSL GCC: FLASH 33968 B / 127 KB (26.12%), RAM 5544 B / 16 KB (33.84%)
- Keil ARMCC -O1: Code 20936 + RO 2268, ZI 4944

### 可用 msh 命令 (application/motor_shell.c)

| 命令 | 用途 | Stage |
| --- | --- | --- |
| `pwm_info` | 打印 TMR1 配置 (ARR/频率/当前 CCR/EN 状态) | 1 |
| `pwm_duty <u> <v> <w>` | 手动设置三相占空比 ticks (限幅 95%) | 2 开环 |
| `pwm_en <0\|1>` | 控制 MP6540H EN 引脚 (0=禁用, 1=使能) | 安全测试 |
| `led <0\|1>` | 控制 LED (PA0) | GPIO 验证 |
| `mc_state` | 打印电机控制状态机 (state/mode/fault) | 全 Stage |
| `fault` | 打印故障位明细 | 全 Stage |
| `fault_clear` | 清除所有故障 | 全 Stage |
| `encoder` | 打印 MA600A 角度/速度/状态 (Stage 4 接入后有效) | 4 |

finsh 自带命令: `help` / `ps` / `version` / `list_thread` / `free` / `reboot` 等.

### 台架验收步骤
1. 接好 JLink + 板子供电
2. `cd project\MDK_V5 && flash.bat rebuild`
3. 串口工具连 PB6(TX)/PB7(RX), 115200 8N1
4. 上电后应见 RT-Thread 版本横幅 + `msh />` 提示符
5. 输入 `pwm_info` 验证 TMR1 配置, `led 1` 验证 GPIO, `mc_state` 验证状态机
6. 示波器探 PA8/PA9/PA10 看 16kHz 中心对齐 50% (Stage 1 PWM 验收)

## 项目定位

项目路径：`E:\WorkSpaces\2_电机驱动\MPS_MotorDriver`

目标是开发基于 AT32M412 + MP6540H + MA600A 的无刷有感 FOC 控制器。功能路线包括开环控制、电流环、速度环、角度环，后续通过 CAN 通讯进行指令控制。

参考 AT 官方电机库：

`E:\WorkSpaces\01_ChipInfomation\AT32\M412\AT32M412_LV_MC_Library_Porject_V2.1.5`

重点参考示例：

`at32m412_lv_motor_ev\at32m412\pmsm_foc_magnetic_encoder`

## 当前工程事实

- 当前工程由 AT Workbench/Keil/CMake 相关文件组成。
- MCU：AT32M412，当前链接脚本为 128KB Flash、16KB RAM。
- 位置传感器：MA600A，当前已有 `middlewares/msp/ma600/` 驱动基础。
- 当前 MA600A AT32 适配使用 SPI2 和轮询传输。
- 当前工程已有 RT-Thread Nano 相关代码，但电机实时控制路径不应依赖复杂线程调度。
- AT 官方 M412 电机库示例的硬实时路径主要在 `mc_isr.c`，硬件初始化主要在 `mc_hwio.c`，配置主要在 `motor_control_drive_param.h` 和 `mc_hwio_m412_lv_v1_0.h`。

## 架构原则

- 不完整移植 VESC。
- 不把 AT 官方 demo 原样塞进 Workbench 生成工程。
- 参考 AT 官方 `pmsm_foc_magnetic_encoder` 的 PWM、ADC、DMA、编码器和中断时序。
- 工程主架构由本项目控制，AT 官方 `mclib` 只能作为第三方库或参考实现接入。
- Workbench 生成代码允许重生成，不应写入长期业务逻辑。
- MA600A 驱动保持独立，通过位置传感器抽象接入 FOC。
- MCU/板级差异集中在 `platform/` 和 `board` 层，控制算法层不直接操作具体寄存器。

## 开发记录位置

主开发记录：

`doc/FOC控制器开发记录.md`

该文档必须持续维护以下内容：

- 开发计划
- 开发进度
- 调试记录模板
- 参数调试记录
- CAN 协议草案
- 阶段性结论
- 已知问题

已有相关文档：

- `doc/调试记录.md`
- `doc/AI协同开发计划.md`
- `doc/第一阶段上电检查清单.md`
- `doc/MA600A.pdf`

注意：部分旧中文文档在当前终端中可能显示乱码，编辑时优先新建或保持 UTF-8 编码。

## 开发阶段

当前路线按以下阶段推进：

1. 阶段 0：基线整理与安全约束
2. 阶段 1：硬件 Bring-up
3. 阶段 2：PWM 与开环控制
4. 阶段 3：ADC 同步采样与电流反馈
5. 阶段 4：MA600A 有感角度闭环准备
6. 阶段 5：电流环控制
7. 阶段 6：速度环控制
8. 阶段 7：角度环/位置环控制
9. 阶段 8：CAN 指令控制
10. 阶段 9：保护、参数与发布整理

详细任务和完成标准见 `doc/FOC控制器开发记录.md`。

## 每次继续开发前的检查

必须执行或读取：

- `git status --short`
- 当前构建日志或终端警告
- `doc/FOC控制器开发记录.md`
- 最近一次调试记录
- 涉及硬件的原理图、引脚和外设映射结论

不要回退用户已有改动。不要使用 `git reset --hard` 或 `git checkout --` 清理工作区，除非用户明确要求。

## 记录规范

每完成一个可验证节点，必须更新 `doc/FOC控制器开发记录.md` 的开发进度表。

每次硬件调试必须记录：

```text
日期：
记录编号：
阶段：
目标：
硬件版本：
固件版本/提交号：
供电条件：
电机/负载：
仪器：
连接方式：
操作步骤：
观察结果：
异常现象：
初步判断：
处理措施：
验证方法：
结论：
照片/波形/日志路径：
下一步：
```

参数调试必须记录母线电压、限流、PWM 频率、极对数、MA600A 零点、电流采样比例、各环 PID 参数、目标值、现象、波形或日志路径。

## 安全约束

- 上电和电机调试必须使用限流电源。
- 首次 PWM 输出必须在 MP6540H 禁用或低电压限流条件下验证波形。
- 使能 MP6540H 前确认 PWM 默认态、死区、刹车输入、故障脚状态。
- 闭环前必须验证 MA600A 角度连续性、方向、零点和电角度换算。
- 电流环前必须验证 ADC 偏置、采样点、相电流方向和比例。
- 任意异常电流、异味、异常温升、驱动故障，立即停机并记录。

## 当前已知事项

- Keil 构建曾出现 RT-Thread `cpuport.c` 的 `context` 未使用警告，原因是 `RT_USING_CONSOLE` 未启用导致 `rt_kprintf(...)` 展开为空。
- CMake 配置曾引用 `project/src/syscalls.c` 和 `project/src/sysmem.c`，如果这些文件缺失会影响 GCC/CMake 构建。
- AT 官方磁编码器示例默认支持 AS5047P/TLE5012B，需要为 MA600A 增加适配层，不建议直接改官方传感器驱动深处。

## 下一步建议

优先完成阶段 0：

1. 固定可编译基线。
2. 清理或记录当前构建警告。
3. 明确 MP6540H 引脚、MA600A 引脚、电流采样拓扑。
4. 从 AT 官方 `pmsm_foc_magnetic_encoder` 提取 PWM/ADC/中断时序设计。
5. 规划 `position_sensor` 和 `motor_control` 最小接口。
