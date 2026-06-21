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
