# CLAUDE.md

## Token 节省规范（所有 Stage 适用，agent 每次会话开头必读）

1. **优先 Grep 定位，再按需 Read**: 避免整文件读入大文件（spec 1050 行 ~15k tokens）。先用 Grep 找行号，再 `Read offset/limit` 读必要片段。
2. **spec/大文件只在首次需要时读**: 后续引用 CLAUDE.md 中已记录的结论，不重复读 spec 全文。
3. **关键结论写进 CLAUDE.md**: bug 根因、接口设计、硬件约束等写入文件，不依赖对话上下文记忆（文件按需读一次，对话每轮付费）。
4. **每个 Stage 完成后提示用户开新会话**: 用 compact 摘要做 handoff，避免单会话累积到 100k+ 后每轮为旧内容付费。新会话从 CLAUDE.md + compact 摘要起步，上下文更小。
5. **工具结果及时消化**: 构建日志/测试输出读完后提取结论，不需要的内容不留在上下文里（比如 Keil 编译中间过程的 .c 文件列表无信息量）。

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

## Stage 2 Complete - 2026-06-22

Stage 2 (PWM 与开环控制) 代码完成, 详见 `docs/superpowers/specs/2026-06-22-mps-foc-design.md` §7 Stage 2.

完成内容:
- `foc_core.c` 实现:
  - 自包含 sin/cos 查表 (256 点 + 线性插值, 误差 < 1.2e-4), 不依赖 CMSIS-DSP, ISR 时序确定性. spec §4.1 允许后期替换 arm_sin_cos_f32
  - `foc_park` / `foc_ipark` (等幅值变换, 用 LUT)
  - `foc_svpwm_3phase_high_side`: min-max 零序注入法 (与经典 7 段法 SVPWM 输出数值等价, 无扇区边界 bug, 过调制区由硬限幅自然退化六拍). 针对 MP6540H 3 路高边: 直接写 CCR1/2/3, 无互补/死区
- `motor_control_isr.[ch]` 实现 OPEN_LOOP 模式:
  - 角度斜坡: theta_e += speed_rad_per_s * dt (dt = 1/16000)
  - Vd = 用户设定, Vq = 0 -> IPark -> SVPWM -> 写 CCR
  - DISABLED/FAULT 强制 50% 三相同电位; FAULT 态同步关 MP6540H EN
  - ISR 不调 RT-Thread API (spec §3.5 PRIMASK 约束)
  - 暴露 `motor_control_isr_open_loop_start/stop/get_debug` 供 shell 调用
- `motor_pwm_at32m412.[ch]` 新增:
  - `TMR1_OVF_TMR10_IRQHandler`: TMR1 溢出中断 (16kHz 中心对齐顶点), 清标志后调 `motor_control_isr_tick()`
  - `motor_pwm_at32m412_enable_ovf_irq / disable_ovf_irq`: NVIC + TMR_OVF_INT 使能控制
- `motor_app.[ch]` 新增 `motor_app_get_control_rw()`: ISR / 开环接口需可写 motor_control 实例
- `motor_shell.c` 新增 3 个 msh 命令: `mc_open` / `mc_stop` / `mc_debug`

关键决策 (与 spec 原文不同):
- SVPWM 用 min-max 零序注入而非经典扇区表: 数值等价但无扇区边界 bug, 实现更短更易验证. spec §4.2.3 单元测试检查的是 ta/tb/tc 输出值, min-max 结果吻合
- VBUS 暂用固定 12V (ADC 采样 Stage 3 接入后替换为实测值)
- 三角函数用自建 LUT 而非 CMSIS-DSP: 当前工程未编译进 arm_sin_cos_f32, 自建 LUT 零外部依赖, 后期可平滑替换

资源占用 (Stage 2 完成后):
- WSL GCC: FLASH 58920 B / 127 KB (45.31%), RAM 5976 B / 16 KB (36.47%)
- Keil ARMCC -O1: Code 30764 + RO 3732, ZI 4948, 0 Error 0 Warning

新增 msh 命令:

| 命令 | 用途 |
| --- | --- |
| `mc_open <vd_volts> <speed_rpm_elec>` | 启动开环旋转 (vd 0..18V, 电角度 rpm 正=正转) |
| `mc_stop` | 停止开环, 关 MP6540H, 切 DISABLED |
| `mc_debug` | 打印 ISR 内部状态 (theta/valpha/vbeta/CCR/tick_count) |

台架验收步骤 (首次让电机带电旋转, 必须限流电源):
1. 接好 JLink + 板子, 限流电源设 12V 限流 0.3A
2. `cd project\MDK_V5 && flash.bat rebuild` 烧录
3. 串口连 PB6(TX)/PB7(RX) 115200, 见 msh 提示符
4. `mc_state` 确认 state=DISABLED, fault=0
5. `mc_open 1000 60` (1000mV=1V d 轴电压, 60rpm 电角度 ≈ 8.6rpm 机械 @7对极)
6. 示波器看 PA8/PA9/PA10: 应为 16kHz 中心对齐, 三相互差 120°, 调制比 ~ m=1/12
7. 电流探头看三相电流: 应为正弦, 平衡度 < 5%, 电流 < 设定限流值
8. `mc_debug` 观察 theta_mrad 持续递增, v_alpha/v_beta 为正弦 (mV), CCR 在限幅内, ol_branch_hits 递增
9. `mc_stop` 停止, 确认 MP6540H EN=低, PWM 回 50%
10. 异常 (过流/异味/堵转) 立即 `mc_stop` 并记录

关键约束 (调试发现):
- **rt_kprintf 不支持 %f**: RT-Thread Nano kservice.c 是最小化整数 printf, %f 会原样打印为 "%f". 所有 shell 浮点输出必须转整数定点 (mV/mrad/mdeg/mrpm). mc_open 参数已改为整数毫伏 (vd_mv), 非 V.
- **故障分级**: `FAULT_CAL_INVALID` 是告警级, 不阻止电机使能 (spec §4.7.3). ISR/mc_open 用 `fault_manager_any_fatal()` (FAULT_FATAL_MASK) 判断, 非 `fault_manager_any()`. 开机 motor_calibration_load() 必置 CAL_INVALID (Stage 4b 标定前无有效数据).
- **finsh getchar 轮询限制**: `rt_hw_console_getchar()` (board.c) 是轮询读 USART, 无 RX 中断/FIFO. 一次性发送整行会 overrun 丢字符. 自动化测试必须逐字符发送+等待回显 (tests/com9_test.py). Stage 8+ 若需高频调试可改 RX 中断驱动.
- **ISR 诊断计数器**: mc_debug 输出 ol_branch_hits/fault_hits/disabled_hits, 用于判断 ISR 走了哪个分支. 若 CCR=0 但 active=1, 先看这三个计数器定位.

已知限制:
- VBUS 固定 12V, Stage 3 接 ADC 后改实测
- 开环不带编码器同步, 电机可能不转 (预期, spec §7 Stage 2 验收只要求三相电流正弦)
- CURRENT/SPEED/POSITION 模式分支为空, Stage 5+ 填充

## Stage 3 Complete - 2026-06-22

Stage 3 (ADC 同步采样与电流反馈) 代码完成, 详见 spec §7 Stage 3.

完成内容:
- `current_sense_at32m412.[ch]` 实现:
  - ADC2 注入序列 3 通道 [SOA(PB1/CH9), SOB(PB0/CH8), SOC(PA7/CH7)], TMR1_CH4 上升沿触发
  - ADC2 普通序列 1 通道 [VBUS(PA6/CH6)], 软件触发读取
  - ADC_CLK = 180MHz/6 = 30MHz, 12-bit, 电流采样 1.5 cycle, VBUS 13.5 cycle
  - ADC 校准 (adc_calibration_init/start) + 各 while 循环超时保护 (防硬件异常挂死)
  - 零偏标定: PWM 50% + MP6540H EN=HIGH 时采 1024 次平均, 偏差 < 50 LSB
  - `current_sense_calc`: raw -> 安培 (gain ~3.16 mA/LSB typ)
  - VBUS 软件触发读取: `current_sense_at32m412_read_vbus()` 返回电压 (V)
- `motor_pwm_at32m412.c` 修改:
  - CH4 从仅设比较值改为完整 `tmr_output_channel_config` (必须配置为输出比较才产生比较事件触发 ADC)
  - **CH4 触发点从顶点 (ARR) 改为谷底 (1)**: MP6540H 电流镜仅在高边导通时反映相电流, 顶点采样高边关闭读数为 V_REF. 谷底三相高边全部导通, 是唯一保证三相均可测的采样点
- `motor_control_isr.[ch]` 修改:
  - ISR 每 tick 读 ADC 注入序列 (ia/ib/ic raw), 算电流 (A)
  - VBUS 1kHz 分频采样 (16kHz/16), 缓存供 SVPWM 使用, 替代 Stage 2 硬编码 12V
  - 欠压/过压检查 (与 VBUS 采样同步 1kHz)
  - 过流保护: 任一相 |I| > 5A -> FAULT_OVERCURRENT
  - 不平衡保护: |ia+ib+ic| > 1.5A -> FAULT_OVERCURRENT
  - mc_debug 新增电流/VBUS/保护计数器快照
- `motor_app.c` 修改: `motor_app_init()` 调用 `current_sense_at32m412_init()` (PWM 之后, ADC 由 TMR1_CH4 触发)
- `motor_shell.c` 新增 3 个 msh 命令: `mc_current` / `mc_cal` / `vbus`

关键决策 (与 spec 原文不同):
- **CH4 触发点改为谷底**: spec §3.2 原设计顶点采样, 基于低边采样电阻拓扑. MP6540H 电流镜在高边关闭时输出 V_REF (零电流), 必须在高边导通时采样. 谷底 (counter≈0) 三相高边全部导通. 实测验证: 顶点采样 raw 全部接近 2048 (V_REF), 谷底采样能读到实际电流
- **零偏窗口 50 LSB**: spec §4.3.3 原文 20 LSB. 实测零偏 ~2068 (偏差 22 LSB), 由 4.7k/4.7k 分压电阻 1% 容差 + MP6540H 偏置决定. 50 LSB = 0.16A, 安全范围内
- **ADC2 时钟**: 需同时开 CRM_ADC1_PERIPH_CLOCK + CRM_ADC2_PERIPH_CLOCK (ADC common 配置依赖 ADC1 时钟). 仅开 ADC2 会导致 ADC 校准 while 循环挂死, 板子无法启动
- **mc_cal 标定时使能 MP6540H**: 电流镜需芯片上电才能输出有效 V_REF. EN=LOW 时 SO 引脚偏置不正常

资源占用 (Stage 3 完成后):
- Keil ARMCC -O1: Code 27428 + RO 3780, ZI 4948, 0 Error 0 Warning

新增 msh 命令:

| 命令 | 用途 |
| --- | --- |
| `mc_current` | 打印三相电流 + VBUS 详细 (raw/offset/mA/mV) |
| `mc_cal` | 零偏标定 (PWM 50% + MP6540H EN, 1024 次平均) |
| `vbus` | 独立读取 VBUS 电压 (软件触发, 不依赖 ISR) |

台架验证结果 (COM9 串口自动化测试):
- VBUS 读取: 11.9V (3 次一致性 < 0.1V) ✓
- 零偏标定: a=2068 b=2068 c=2064 (偏差 20-22 LSB < 50 LSB) PASS ✓
- 静态电流 (50% PWM 无负载): ia=63 ib=-37 ic=91 mA (接近 0, 残余为噪声) ✓
- 开环旋转 (1V/60rpm, 2V/120rpm): 电流 -116~138 mA, 无过流/不平衡故障 ✓
- VBUS 旋转中稳定: 11.87-11.93V ✓
- 停止后电流归零, fault=0x00 ✓

关键约束 (调试发现):
- **ADC2 时钟必须同时开 ADC1+ADC2**: 仅 CRM_ADC2_PERIPH_CLOCK 不够, ADC common 配置需 ADC1 时钟. 漏开会导致校准循环挂死
- **CH4 必须调用 tmr_output_channel_config**: 仅 tmr_channel_value_set 设比较值不产生比较事件, ADC 注入序列无法触发. 参考工程 mc_hwio.c 同款做法
- **MP6540H 电流镜采样时序**: 高边关闭时 I_LOAD=0, V_SO=V_REF=2048. 必须在高边导通期采样. 中心对齐谷底 (counter≈0) 三相高边全开, 是正确采样点
- **ISR 读 ADC 时序**: TMR1_CH4 谷底触发 ADC -> 转换 3µs -> TMR1_OVF 顶点中断读结果. 延迟半个周期 (31µs), 对 16kHz 控制环可忽略

## Stage 4 + 4b Complete - 2026-06-22

Stage 4 (MA600A 有感角度闭环准备) + Stage 4b (旁轴非线性标定) 代码完成, 详见 spec §7 Stage 4/4b.

完成内容:
- `motor_encoder_at32m412.[ch]` 实现 (Stage 4):
  - **SPI2 硬件初始化** (重建 Stage 0 清空的 wk_spi2_init): CRM_SPI2 时钟 + GPIO MUX (PB3/PB4/PB5 AF3) + spi_init (Mode1/MSB/16bit/DIV_64=2.8MHz/软件CS) + spi_enable. 参考 AT 官方 AS5047P.c
  - `motor_encoder_read_angle_speed()`: 封装 ma600a_read_angle_and_speed_raw, 12-bit->16-bit 扩展, 失败递增 error_count
  - `motor_encoder_to_electrical_angle()`: 旁轴查表校正 (256 点 Q16 插值, spec §4.7.6) -> 减零点 -> ×极对数 -> mod 2π
  - 调试接口: get_last_raw / get_error_count / is_alive / set_zero / get_zero
- `flash_calibration_at32m412.[ch]` 实现 (Stage 4b):
  - `flash_calibration_read()`: memcpy 从 0x0801FC00 读 532 字节 -> 校验 magic/version -> CRC32 校验
  - `flash_calibration_write()`: flash_unlock -> sector_erase -> word_program×133 -> flash_lock -> 回读校验
  - `flash_calibration_erase()`: 擦除末页 sector
  - `flash_calibration_crc32()`: 硬件 CRC (CRM_CRC 时钟 + crc_block_calculate), 默认 IEEE CRC-32, 处理任意字节长度 (尾部补 0)
- `motor_calibration.[ch]` 实现 (Stage 4b):
  - `motor_calibration_t` 结构体 (532 字节, 含编译期断言确保 <= 1KB)
  - 标定状态机: IDLE -> ZERO_ALIGN -> SPIN_FWD -> SPIN_REV -> COMPUTE -> WRITE_FLASH -> DONE (+ ABORTED)
  - `motor_calibration_tick()` (ISR 内): CAL_SPIN_FWD/REV 状态下累加 256 段直方图, 计数满自动切状态. 不调 RT-Thread API
  - `motor_calibration_poll()` (线程): 处理 ALIGN 等待/COMPUTE/WRITE_FLASH/DONE/ABORTED. 含超时与故障中止
  - `cal_compute_table()`: (hist_fwd+hist_rev)/2 去速度脉动 -> 去直流 -> LSB 转 0.001° -> 限幅 ±32767
  - 开机加载: flash_calibration_read 成功则 g_cal_valid=true + 写零点; 失败置 FAULT_CAL_INVALID (告警级)
  - RAM 占用: 两个 256×int32 直方图 (2KB) + s_cal (532B)
- `motor_control.[ch]` 修改: 新增 `MOTOR_CONTROL_MODE_ALIGN` (模式 4), set_mode 范围检查更新
- `motor_control_isr.[ch]` 修改:
  - **ALIGN 模式分支**: theta=0 (固定 d 轴 0°), Vd=s_align_vd, Vq=0, 转子锁定. 前 400ms settle, 后 100ms 累加角度作零点
  - **编码器读取**: ENABLED 状态每 tick 读 MA600A (~6µs), 连续 32 次失败置 FAULT_SENSOR
  - **OPEN_LOOP --enc 开关**: `motor_control_isr_open_loop_set_encoder_angle(bool)`, true=用编码器电角度, false=纯斜坡 (向后兼容)
  - **标定采集钩子**: ISR 末尾调 motor_calibration_tick()
  - 新增接口: align_start/stop/get_align_angle, open_loop_set_encoder_angle
  - debug 结构体扩展: enc_raw/enc_theta_mrad/enc_errors/enc_alive/align_hits/cal_state/cal_progress
- `motor_app.c` 修改: motor_app_init 接入 motor_encoder_at32m412_init + ma600a_debug_init 恢复; motor_app_run 接入 motor_calibration_poll
- `motor_shell.c` 修改:
  - 扩展 `encoder`: raw_16bit/raw_12bit/angle_mdeg/elec_mrad/zero/errors/alive/cal_valid + ma600a_debug 全局变量
  - 扩展 `mc_open`: 第三参数可选 `enc`/`ramp`
  - 扩展 `mc_debug`: 新增 encoder/align/cal 快照行
  - 新增 `mc_align <vd_mv>`: 启动 ALIGN 模式
  - 新增 `mc_zero [raw16]`: 读取/设置零点
  - 新增 `mc_calibrate`: 触发旁轴标定
  - 新增 `mc_cal_status`: 标定状态/进度/残差
  - 新增 `mc_cal_dump`: 打印 256 点表
  - 新增 `mc_cal_erase`: 擦除 FLASH 标定区
- `motor_params.h` 修改: CAL_HIST_BINS / CAL_CRC_PAYLOAD_SIZE / CAL_SPIN_TIMEOUT_MS / CAL_MAX_RESIDUAL_MDEG / ALIGN_VD_VOLTS / ALIGN_SETTLE_MS / ALIGN_SAMPLE_TICKS

关键决策 (与 spec 原文不同):
- **SPI2 时钟 DIV_64 (2.8125MHz)**: spec 参考 AS5047P 用 DIV_32 (5.625MHz), 但 MA600A 最大 SPI 时钟 4.16MHz, DIV_32 超限. DIV_64 安全且读角度耗时 ~6µs, ISR 62.5µs 预算充裕
- **结构体实际 532 字节而非 spec 标注 528**: table 偏移 12 (非 8), 因 timestamp_ms(4) 在 reserved[3] 后无 padding. 产品代码用 sizeof 处理, 不硬编码. 已加编译期断言确保 <= 1KB FLASH sector
- **ALIGN 零点采样用累加器而非环形缓冲**: spec §4.5.3 原意"最后 100ms 平均", 环形缓冲 1600×2=3.2KB RAM 太费. 改为前 400ms settle + 后 100ms 累加器平均, 省 RAM 且数学等价
- **标定用纯斜坡开环而非编码器电角度**: 标定期间 OPEN_LOOP 用 ramp 模式 (set_encoder_angle(false)), 避免零点刚设影响采集. 编码器仅被动读取角度
- **直方图箱内偏移中心化**: delta = raw - idx*256 - 128, 范围 -128..127 (而非 0..255), 便于去直流与限幅

资源占用 (Stage 4+4b 完成后):
- WSL GCC: FLASH 56292 B / 127 KB (43.29%), RAM 8408 B / 16 KB (51.32%)
- 相比 Stage 3+finsh: FLASH +22324 B, RAM +2864 B (两个直方图 2KB + 标定结构体 0.5KB + 编码器/ALIGN 状态)

新增 msh 命令:

| 命令 | 用途 | Stage |
| --- | --- | --- |
| `mc_align <vd_mv>` | 启动 ALIGN 模式 (转子锁定 d 轴 0°) | 4 |
| `mc_zero [raw16]` | 读取/设置编码器零点 | 4 |
| `mc_calibrate` | 触发旁轴非线性标定 (~25s) | 4b |
| `mc_cal_status` | 标定状态/进度/残差 | 4b |
| `mc_cal_dump` | 打印 256 点校正表 | 4b |
| `mc_cal_erase` | 擦除 FLASH 标定区 | 4b |

扩展命令: `encoder` (新增 elec_mrad/zero/errors/alive/cal_valid), `mc_open` (新增 enc/ramp 第三参数), `mc_debug` (新增 encoder/align/cal 行)

台架验收步骤 (首次接入编码器, 必须限流电源):
1. 接好 JLink + 板子, 限流电源 12V 限流 0.5A
2. `cd project\MDK_V5 && flash.bat rebuild` 烧录 (或 WSL cmake 产物用 JLink 烧)
3. 串口连 PB6(TX)/PB7(RX) 115200, 见 msh 提示符
4. `encoder` 确认: raw_16bit 随手转轴变化 (0..65535 连续无跳变), errors=0, alive=1
5. `mc_align 1000` (1V): 转子锁定 d 轴 0°, 手转有阻力, 持续 500ms
6. `mc_zero` 显示 ALIGN 采集的对齐角度, 确认写入零点
7. `mc_open 1000 300 enc`: 开环用编码器电角度, 电流应正弦, 编码器跟踪
8. `mc_calibrate`: 电机自动正反拖动 5+5 圈 (~25s), 期间 `mc_cal_status` 看进度
9. 标定完成: `mc_cal_status` 显示 DONE, max_resid < 1000 mdeg (1°)
10. `mc_cal_dump`: 256 点表非零, 数值在 ±32767 范围
11. 断电重启: `fault` 无 CAL_INVALID, `encoder` cal_valid=1 (校正自动加载)
12. `mc_cal_erase` + 重启: `fault` 有 CAL_INVALID, 可重新 `mc_calibrate`
13. 异常 (过流/异味/堵转) 立即 `mc_stop` 并记录

关键约束 (实现发现):
- **SPI2 GPIO MUX_3**: PB3/PB4/PB5 的 AF3 = SPI2_SCK/MISO/MOSI (与 AT 官方 PB13/14/15 AF5 是同外设不同引脚不同复用号)
- **MA600A SPI Mode 1**: CPOL=0 CPHA=1, 与 AS5047P 一致. 16-bit 帧, MSB first
- **ISR 内 SPI 读取安全**: ma600a_read_angle_and_speed_raw 只用 delay_us 忙等 (~1µs), 不触发 RT-Thread API. delay_ms (rt_thread_mdelay) 仅在 ma600a_init 读 BCT 时用, 在线程上下文
- **编码器连续失败 32 次置 FAULT_SENSOR**: 避免单次 SPI 干扰误触发, 但持续故障必须保护
- **标定中止规则**: 任意阶段 fault_fatal 或超时 -> abort, 停电机, 状态回 IDLE, FLASH 不写, 旧标定保留

已知限制:
- ALIGN_VD_VOLTS 初值 1.0f (估算 1A×1Ω), 台架需实测相电阻后修正
- 极对数仍为默认 7, 台架实测后改 motor_params.h MOTOR_POLE_PAIRS
- CURRENT/SPEED/POSITION 模式分支仍为空 (50%), Stage 5+ 填充
- 标定用开环恒速, 速度脉动影响残差; 闭环速度环 (Stage 6) 后可重标定提升精度
- 单元测试 (test_motor_calibration.c) 验证算法数学 (结构体布局/CRC32/电角度/查表), 硬件交互需台架验证

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
| `mc_open <vd_mv> <rpm_elec> [enc\|ramp]` | 启动开环旋转 (vd 毫伏, 电角度 rpm; enc=用编码器电角度) | 2/4 |
| `mc_stop` | 停止开环, 关 MP6540H | 2 |
| `mc_debug` | 打印 ISR 内部状态 (mrad/mV/CCR/tick/分支命中/电流/VBUS/编码器/标定) | 2+3+4 |
| `mc_current` | 打印三相电流 + VBUS 详细 (raw/offset/mA/mV) | 3 |
| `mc_cal` | 零偏标定 (PWM 50% + MP6540H EN, 1024 次平均) | 3 |
| `vbus` | 独立读取 VBUS 电压 (软件触发) | 3 |
| `mc_align <vd_mv>` | 启动 ALIGN 模式 (转子锁定 d 轴 0°) | 4 |
| `mc_zero [raw16]` | 读取/设置编码器零点 | 4 |
| `mc_calibrate` | 触发旁轴非线性标定 (~25s) | 4b |
| `mc_cal_status` | 标定状态/进度/残差 | 4b |
| `mc_cal_dump` | 打印 256 点校正表 | 4b |
| `mc_cal_erase` | 擦除 FLASH 标定区 | 4b |

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
