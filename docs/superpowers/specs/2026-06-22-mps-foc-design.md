# MPS_MotorDriver FOC 软件设计文档

- 项目: AT32M412 + MP6540H + MA600A 有感 FOC 控制器
- 设计日期: 2026-06-22
- 状态: 待用户 review
- 工程路径: `E:\WorkSpaces\2_MotorDriver\MPS_MotorDriver`
- 参考工程: `E:\WorkSpaces\2_MotorDriver\AT32M412_LV_MC_Library_Porject_V2.1.5`
- 前置文档: `CLAUDE.md` / `doc/FOC控制器开发记录.md` / `doc/stage0_baseline_2026-06-11.md`

## 0. 设计目标与非目标

### 0.1 目标

基于 MPS_MotorDriver 当前工程, 复用现有 RT-Thread Nano + CMake/WSL + MA600A 驱动栈, 完成一套针对 MPS 板 (MP6540H + MA600A + MP4583) 定制的有感 FOC 控制器固件. 控制流程包括开环 -> 电流环 -> 速度环 -> 位置环, 通过 CAN 总线接收指令并周期上报状态.

### 0.2 非目标

- 不完整移植 AT 官方 mclib 工程结构.
- 不引入 AT 官方预编译 `.lib` 文件 (与当前 GCC/CMake 构建栈不兼容, 且与 MP6540H 拓扑不匹配).
- 不直接移植 LV demo 的 PWM 互补输出与 OPA 电流采样路径 (硬件不一致).
- 不引入 VESC / ODrive 完整协议栈, CAN 协议保持简单.
- 不写在 Workbench 生成的 `add user code begin/end` 留白里 (Workbench 生成文件未来不再使用).

### 0.3 关键架构决策摘要 (经用户逐题确认)

| 决策 | 选定方案 |
| --- | --- |
| 算法来源 | 全自研 (对照 LV demo 算法思路与变量命名); AT mclib 开源的仅是滤波器/定点PID, 核心 Clarke/Park/SVPWM 在 .lib 黑盒中无法复用 (见 §4.1) |
| Workbench 文件 | Y: 保留头文件中的引脚 `#define`, `wk_*_init()` 函数全部废弃 |
| PWM 拓扑 | A: TMR1 中心对齐 + 16 kHz + 重复计数 1 |
| 电流采样 | 3.1 顶点采样 + 3.2 三路同时采样 + SVPWM 占空比硬限幅 95% |
| MA600A 读取 | A: FOC ISR 内 SPI 同步阻塞读 (~5 µs) |
| 控制环分层 | A: LV demo 同款, 全部环路在 FOC ISR 内分频执行 |
| CAN 协议 | B: 精简 1 帧控制 + 1 帧状态 + 1 帧扩展状态, 节点 ID 编址 |
| 故障管理 | X: 最小集 6 项 (nFAULT/过流/欠压/过压/MA600A 超时/CAN 超时) |

## 1. 硬件事实参考 (固定输入, 不可变)

### 1.1 芯片与板载关键参数

- MCU: AT32M412KBU7-4 (QFN32), 128 KB Flash, 16 KB RAM, Cortex-M4 180 MHz (实际配置, Flash 等待周期 5), FPU
- 三相功率级: MP6540H, 内置三半桥 + 电流镜, **3 路高边 PWM 输入, 死区芯片内部**
- 位置传感器: MA600A, SPI 接口, 12-bit 绝对角度, 集成速度寄存器
- 电源: MP4583 宽压输入降压, 输出 5 V / 3.3 V

### 1.2 引脚映射 (Workbench 初始分配, 保留作硬件设计参考)

| 功能 | 引脚 | 备注 |
| --- | --- | --- |
| PWM A (TMR1_CH1) | PA8 | MP6540H INHA |
| PWM B (TMR1_CH2) | PA9 | MP6540H INHB |
| PWM C (TMR1_CH3) | PA10 | MP6540H INHC |
| PWM_EN | PB10 | MP6540H EN, 低有效 nSLEEP/EN |
| nFAULT | PB2 | MP6540H 故障, 低有效, 接 EXINT2 |
| SOA (Phase A current) | PB1 / ADC2_IN9 | MP6540H CS A |
| SOB (Phase B current) | PB0 / ADC2_IN8 | MP6540H CS B |
| SOC (Phase C current) | PA7 / ADC2_IN7 | MP6540H CS C |
| VBUS | PA6 / ADC2_IN6 | 母线电压采样 |
| SPI2_SCK  | PB3  | MA600A |
| SPI2_MISO | PB4  | MA600A |
| SPI2_MOSI | PB5  | MA600A |
| SPI2_CS | PA15 | MA600A 片选, 软件管理 |
| CAN1_TX | PA12 | TJA1051 收发器 |
| CAN1_RX | PA11 | TJA1051 收发器 |
| USART1_TX | PB6  | finsh shell 调试 |
| USART1_RX | PB7  | finsh shell 调试 |
| LED | PA0 | 状态指示 |

> 注: USART1 已确认移至 PB6 (TX) / PB7 (RX), 与 PWM 引脚无冲突.

### 1.3 关键时序常数

- TMR1 时钟: 180 MHz (APB2 = sclk, APB2_DIV_1 定时器不翻倍)
- PWM 频率: 16 kHz
- PWM 周期 (中心对齐, 重复计数 1): `ARR = 180e6 / (2 * 16e3) - 1 = 5624`
- 单 PWM 半周期: 31.25 µs, 全周期 62.5 µs
- ADC 转换时间: ~0.7 µs/通道 (180 MHz / 12-bit / ~12 cycles sample + conv)
- MA600A SPI 时钟: 12 MHz (远低于 25 MHz 上限), 16-bit 传输 ~1.4 µs, 32-bit ~2.7 µs
- 控制环频率: 电流环 16 kHz / 速度环 1 kHz (FOC ISR 分频 16) / 位置环 200 Hz (FOC ISR 分频 80)
- VBUS 分压比: 1/6 (硬件分压电阻已固定), ADC 满量程对应母线 19.8 V, 1 LSB = 4.834 mV
- MP6540H 电流采样硬件: SOA/SOB/SOC 各通过 4.7 kΩ 上拉到 3.3 V, 4.7 kΩ 下拉到 GND (即数据手册 Figure 2 推荐拓扑), 零电流偏置 V_REF = 1.65 V, 等效 R_TERM = 2.35 kΩ
- FLASH 标定区: `AT32M412xB_FLASH.ld` 链接脚本将 FLASH 容量从 128 KB 缩为 127 KB, 末 1 KB 扇区 (起始 `0x0801FC00`) 专用于存储旁轴标定数据 (`motor_calibration_t`, 见 §4.7)

### 1.4 ISR 时间预算 (16 kHz, 周期 62.5 µs)

| 工作 | 耗时 | 累计 |
| --- | --- | --- |
| ADC 注入序列三通道 | ~3 µs | 3 µs |
| MA600A SPI 同步读 | ~5 µs | 8 µs |
| Clarke + Park | ~1 µs | 9 µs |
| 电流环 Id/Iq PI | ~1 µs | 10 µs |
| IPark + SVPWM + 写占空比 | ~3 µs | 13 µs |
| (1/16 次) 速度环 | +1 µs | 14 µs |
| (1/80 次) 位置环 | +1 µs | 15 µs |
| 故障检查 + 寄存器维护 | ~1 µs | 16 µs |

峰值 ISR 占用 ~16 µs / 62.5 µs = 25.6%, 余量充足. (180 MHz sclk 下指令周期 5.5 ns, 实际各环节更快, 预算更宽松.)

## 2. 软件架构

### 2.1 分层与目录结构

```
MPS_MotorDriver/
├── application/                    业务层
│   └── motor_control/
│       ├── motor_control.[ch]      状态机 + 模式 + 目标值 (已有, 需扩展)
│       ├── motor_control_isr.c     FOC ISR 主体 (新增, 16 kHz)
│       ├── foc_core.[ch]           Clarke / Park / IPark / SVPWM (新增)
│       ├── current_loop.[ch]       Id/Iq PI 控制器 (新增)
│       ├── speed_loop.[ch]         速度环 PI + 速度滤波 (新增, 1 kHz)
│       ├── position_loop.[ch]      位置环 P/PI (新增, 200 Hz)
│       ├── motor_params.h          电机参数 / PID 默认值 / 保护阈值 (新增)
│       └── fault_manager.[ch]      故障锁存与清除 (新增)
│
├── platform/                       平台层 (重新组织)
│   └── at32m412/
│       ├── motor_pwm_at32m412.[ch]      TMR1 PWM 三相驱动 (重写)
│       ├── current_sense_at32m412.[ch]  ADC2 注入序列 + 触发 (新增)
│       ├── motor_encoder_at32m412.[ch]  MA600A 适配封装 (新增, 调用 middlewares)
│       ├── motor_protect_at32m412.[ch]  nFAULT EXINT + Vbus 监测 (新增)
│       ├── board_motor_pins.h           引脚与时序常数集中定义 (新增)
│       └── clock_at32m412.[ch]          系统时钟初始化 (新增, 替代 wk_system_clock_config)
│
├── middlewares/                    中间件 (保留)
│   ├── msp/ma600/                  MA600A 平台无关驱动 + AT32 SPI2 适配
│   └── rt-thread/                  RT-Thread Nano
│
├── communication/                  通信层 (新增)
│   ├── can_protocol.[ch]           CAN 协议帧解析 + 上报组装
│   └── can_at32m412.[ch]           CAN1 硬件适配
│
├── application/
│   ├── motor_app.c                 应用层主入口 (新增, 替代 main 业务逻辑)
│   └── ma600a_debug.c              (从 project/src 迁出, 保留 bring-up 路径)
│
├── tests/                          主机端测试 (已有, 持续扩展)
│   ├── motor_control/
│   ├── foc_core/                   SVPWM / Clarke / Park 数表对拍 (新增)
│   └── can_protocol/               协议帧编解码测试 (新增)
│
├── libraries/                      AT32 标准外设库 (保留)
├── project/
│   ├── inc/
│   │   ├── at32m412_416_conf.h     保留
│   │   ├── at32m412_416_int.h      保留 (中断向量名)
│   │   ├── at32m412_416_wk_config.h  仅保留引脚 #define, wk_*_init 声明删除
│   │   ├── rtconfig.h              RT-Thread 配置
│   │   └── ma600a_debug.h          (移动到 application/)
│   ├── src/
│   │   ├── main.c                  精简, 只调用 motor_app_init / motor_app_run
│   │   ├── at32m412_416_int.c      中断向量, 调用 motor_control_isr 等
│   │   ├── at32m412_416_wk_config.c  wk_*_init() 函数体全部删除
│   │   ├── board.c                 RT-Thread BSP
│   │   ├── rtthread_app.c          RT-Thread 任务 (CAN 上报 / 调试)
│   │   ├── syscalls.c / sysmem.c   newlib 桩
│   │   └── ma600a_debug.c          (移动到 application/)
│   └── MDK_V5/                     Keil 工程同步存在但是次级
└── ...
```

### 2.2 模块依赖关系

```
                     CAN bus / finsh
                          |
                  +-------+-------+
                  v               v
            can_protocol     rtthread_app
                  |               |
                  +-------+-------+
                          v
                     motor_control (state machine)
                          |
            +---------+---+----+---------+
            v         v        v         v
       position_  speed_  current_  foc_core
         loop     loop      loop    (SVPWM)
                                       |
                            +----------+----------+
                            v                     v
                     mc_pid_controller       motor_pwm_at32m412
                     mc_math (third_party)        |
                            ^                     v
                            |                  TMR1 HW
                     current_sense_at32m412
                            |
                          ADC2 HW

                     motor_encoder_at32m412 -> middlewares/msp/ma600 -> SPI2 HW

                     motor_protect_at32m412 -> nFAULT/EXINT2, Vbus/ADC
                            |
                            v
                     fault_manager <- 软件阈值检测 (FOC ISR 内)
```

依赖原则:
- 上层不直接访问寄存器, 全部经 `platform/at32m412/` 抽象层
- `motor_control` 状态机不依赖具体硬件, 只依赖 platform 接口
- `foc_core` 不依赖 platform, 是纯数学模块, 可主机端单元测试

### 2.3 文件大小预估

| 文件 | 预估行数 | 复杂度 |
| --- | --- | --- |
| `foc_core.c` (SVPWM + Clarke/Park 包装) | 250 | 中, 主要是 SVPWM 扇区判断 |
| `current_loop.c` (Id/Iq PI) | 100 | 低 |
| `speed_loop.c` (速度 PI + 一阶低通) | 80 | 低 |
| `position_loop.c` (位置 P/PI + 角度跨零处理) | 100 | 中 |
| `motor_control_isr.c` (ISR 主体 + 分频) | 200 | 中 |
| `motor_control.c` (状态机扩展) | 300 (已有 ~150) | 中 |
| `fault_manager.c` | 120 | 低 |
| `motor_pwm_at32m412.c` | 200 | 中 |
| `current_sense_at32m412.c` | 150 | 中 |
| `motor_encoder_at32m412.c` | 80 | 低 |
| `motor_protect_at32m412.c` | 100 | 低 |
| `can_protocol.c` | 200 | 低 |
| `can_at32m412.c` | 150 | 低 |
| 合计自研 C 代码 | ~2450 行 | |
| ``libraries/dsp/`` (CMSIS-DSP FastMath) | ~2000 | 整目录拷贝, 不计入自研 |


## 3. 控制流程与时序

### 3.1 FOC ISR (TMR1 Update IRQ, 16 kHz, 优先级 0)

```c
void TMR1_OVF_TMR10_IRQHandler(void)
{
    /* 0. 清中断标志 */
    tmr_flag_clear(TMR1, TMR_OVF_FLAG);

    /* 1. 读 ADC 注入序列结果 (硬件已在上次 PWM 顶点完成转换) */
    uint16_t ia_raw = adc_inject_data_get(ADC2, INJ_CH_SOA);
    uint16_t ib_raw = adc_inject_data_get(ADC2, INJ_CH_SOB);
    uint16_t ic_raw = adc_inject_data_get(ADC2, INJ_CH_SOC);

    /* 2. 电流去零偏 + 比例缩放 (raw -> 浮点 A 或定点 mA) */
    float ia = current_sense_calc(ia_raw, ofs_a, gain_a);
    float ib = current_sense_calc(ib_raw, ofs_b, gain_b);
    float ic = current_sense_calc(ic_raw, ofs_c, gain_c);

    /* 3. MA600A 角度读 (SPI 同步, ~5 µs) */
    uint16_t raw_angle;
    int16_t raw_speed;
    if (motor_encoder_read_angle_speed(&raw_angle, &raw_speed) != 0) {
        fault_manager_set(FAULT_SENSOR);
    }
    float theta_e = encoder_to_electrical_angle(raw_angle);

    /* 4. Clarke + Park */
    float i_alpha, i_beta;
    foc_clarke(ia, ib, ic, &i_alpha, &i_beta);
    float id, iq;
    foc_park(i_alpha, i_beta, theta_e, &id, &iq);

    /* 5. 电流环 PI */
    float vd_ref, vq_ref;
    current_loop_run(id, iq, &vd_ref, &vq_ref);

    /* 6. IPark + SVPWM */
    float v_alpha, v_beta;
    foc_ipark(vd_ref, vq_ref, theta_e, &v_alpha, &v_beta);
    uint16_t ta, tb, tc;
    foc_svpwm_3phase_high_side(v_alpha, v_beta, vbus, &ta, &tb, &tc);

    /* 7. 写占空比 (硬限幅 95%) */
    motor_pwm_set_duty_ticks(ta, tb, tc);

    /* 8. 分频执行速度环 / 位置环 */
    static uint16_t cnt_speed = 0;
    static uint16_t cnt_pos = 0;
    if (++cnt_speed >= 16) {
        cnt_speed = 0;
        speed_loop_run(raw_speed);
    }
    if (++cnt_pos >= 80) {
        cnt_pos = 0;
        position_loop_run(raw_angle);
    }

    /* 9. ISR 内故障检查 (过流) */
    if (fabsf(iq) > IQ_OVERCURRENT || fabsf(id) > IQ_OVERCURRENT) {
        fault_manager_set(FAULT_OVERCURRENT);
    }

    /* 10. 故障态强制 PWM 输出关断 */
    if (motor_control_get_state() == MOTOR_CONTROL_STATE_FAULT) {
        motor_pwm_disable_output();
    }
}
```

> 注: 上面是设计层伪代码, 实现时变量类型 (float vs int32 定点) 在 Phase 0 评估后定. 初版用浮点 (M412 有 FPU), 后期若 ISR 时间紧张再切定点.

### 3.2 ADC 触发时序 (顶点采样)

```
PWM 周期 (中心对齐):
  counter ----^---------v---------^----  (ARR=5624, period=62.5 µs)
              0       2812      5624

low-side    [开通]    [关闭]   [开通]
high-side   [关闭]    [开通]   [关闭]

ADC 触发: TMR1_CH4 比较值 = ARR = 5624 (顶点)
ADC 完成: 触发后 ~3 µs (注入序列 3 通道)
ISR 触发: TMR1 update event (counter overflow at top) -> TMR1_OVF IRQ
```

设计要点:
- `TMR1_CH4` 配置为 PWM 输出比较模式, 比较值 = ARR, 触发 ADC 注入
- ADC2 配置注入序列 3 通道: `[CH9 (SOA), CH8 (SOB), CH7 (SOC)]`, 由 `ADC_PREEMPT_TRIG_TMR1CH4` 上升沿触发
- `TMR1_OVF` 中断使能, ISR 进入时 ADC 数据已就绪 (3 µs 转换 + ISR 进入延迟 ~0.5 µs)
- 占空比硬限幅: 每相最大输出 = `ARR * 0.95 = 2849`, 留 5% 给 CS 信号建立窗口

### 3.3 状态机

```
       motor_control_init()
              |
              v
       +--------------+
       |   DISABLED   |
       +------+-------+
              | enable() + (state == DISABLED && fault == 0)
              v
       +--------------+      set_fault()       +------------+
       |   ENABLED    | ---------------------> |   FAULT    |
       +------+-------+                        +-----+------+
              ^                                      |
              | (clear_fault() && fault_cleared)     |
              +--------------------------------------+
```

状态转换规则:
- DISABLED -> ENABLED: `motor_control_enable()` 返回 OK 当且仅当 `state == DISABLED` 且 `fault_flags == 0`
- 任意状态 -> FAULT: `fault_manager_set()` 锁存故障, 强制 `motor_pwm_disable_output()`, 目标值清零
- FAULT -> DISABLED: `motor_control_clear_fault()`, 但仅当所有故障源的硬件条件均已恢复
- ENABLED -> DISABLED: `motor_control_disable()`, 清目标值, 立即关 PWM
- DISABLED 状态下 FOC ISR 仍运行 (为了 CAN/调试可观察电流读数), 但 SVPWM 输出占空比 = 50% (三相同电位, 不出力)

### 3.4 控制模式

| 模式 ID | 名称 | 输入 | FOC ISR 行为 |
| --- | --- | --- | --- |
| 0 | OPEN_LOOP | duty_ratio | 跳过 Park/电流环, IPark 直接用固定 d 轴目标电压 (用户输入), 电角度按固定角速度递增 |
| 1 | CURRENT | iq_ref_mA | id_ref = 0, iq_ref = 用户值, 电流环执行 |
| 2 | SPEED | rpm_ref | 速度环输出 iq_ref, 电流环执行 |
| 3 | POSITION | mdeg_ref | 位置环输出 rpm_ref, 速度环输出 iq_ref, 电流环执行 |
| 4 | ALIGN | (none) | 输出固定 Vd_align, 强制转子对齐 d 轴, 标定 MA600A 零点 (见 §4.5.3) |
| 5 | CALIBRATE | (none) | 旁轴非线性标定: 自动执行零点对齐 + 正反拖动 + 计算 + 写 FLASH (见 §4.7) |

### 3.5 中断优先级 (NVIC, 优先级组 4)

| 优先级 | 中断 | 用途 |
| --- | --- | --- |
| 0 (最高) | TMR1_OVF_TMR10_IRQ | FOC ISR (电流/速度/位置环) |
| 1 | EXINT2_IRQ | MP6540H nFAULT |
| 2 | ADC1_2_IRQ | (备用, ADC 错误检测) |
| 3 | CAN1_RX0_IRQ / CAN1_RX1_IRQ | CAN 接收 |
| 14 | SysTick | RT-Thread 节拍 (1 kHz) |
| 15 (最低) | PendSV | RT-Thread 调度切换 |

RT-Thread Nano 临界区实现说明: 本工程 libcpu cortex-m4 的 `rt_hw_interrupt_disable` 使用 PRIMASK + CPSID I (全局关中断), 非 BASEPRI. 因此 `RT_KERNEL_BASEPRI` 配置不适用, FOC ISR 会被 RT-Thread 临界区短暂屏蔽. 缓解: 临界区通常 < 1 µs, 16 kHz 周期 62.5 µs 可吸收; 若 Stage 2/3 实测抖动超阈值, 再改 libcpu 为 BASEPRI 方案 (阈值 0xE0 = preempt 14, 仅屏蔽 SysTick/PendSV).

## 4. 算法实现细节

### 4.1 算法来源说明 (全自研, 对照 LV demo)

经查证 AT 官方 mclib 开源部分:

- ``mclib/src/mc_math.c`` (6.4 KB) 实际只包含**移动平均滤波器与低通滤波器** (``moving_average`` / ``lowpass_filtering``), 使用 ``malloc``/``calloc``, 且依赖 ``mc_lib.h`` 总线头文件. **不含 Clarke/Park/SVPWM**.
- ``mclib/src/mc_pid_controller.c`` (11 KB) 是 ``int16_t`` **定点 PI**, 且与 flash 读取/自动调谐耦合, 拉入大量无关依赖.
- 真正的 Clarke/Park/IPark/SVPWM 核心算法封装在 ``mclib/src/mc_foc_kernal.lib`` (77 KB 预编译黑盒) 中, **源码不开源**, 且该 .lib 是 ARMCC v6 编译产物, 无法被当前 GCC/CMake 构建链接.

**结论: AT 开源部分对本工程价值有限, 核心算法无法复用源码.**

**决策: 算法全部自研**, 但实现时对照 LV demo (``user/src/mc_foc.c`` 的 .c 源码部分、``user/src/mc_isr.c`` 的时序逻辑) 的算法思路与变量命名, 降低出错概率. 全部使用 ``float`` 浮点 (M412 有 FPU, 比 AT 的 ``int16_t`` 定点更直观可调试).

自研模块清单:

| 模块 | 文件 | 行数预估 | 实现依据 |
| --- | --- | --- | --- |
| Clarke 变换 | ``foc_core.c`` | ~15 | 标准公式 Iα=Ia, Iβ=(Ia+2Ib)/√3 |
| Park 变换 | ``foc_core.c`` | ~15 | 标准 Id=Iα·cosθ+Iβ·sinθ, Iq=-Iα·sinθ+Iβ·cosθ |
| 逆 Park 变换 | ``foc_core.c`` | ~15 | 标准 Vα=Vd·cosθ-Vq·sinθ, Vβ=Vd·sinθ+Vq·cosθ |
| 三角函数 | ``foc_core.c`` | ~40 | 查表 + 线性插值 (512 点 sin 表), 或直接用 CMSIS-DSP ``arm_sin_cos_f32`` (见下) |
| SVPWM 7 段法 | ``foc_core.c`` | ~150 | 公开教材标准实现, 对照 LV ``mc_foc.c`` 扇区映射表 |
| 抗 windup PI | ``current_loop.c`` / ``speed_loop.c`` | ~80 | 标准积分限幅 + 输出限幅, 浮点实现 |
| 速度低通滤波 | ``speed_loop.c`` | ~20 | 一阶 IIR, 系数可调 |

**三角函数策略**: 优先用 LV demo ``libraries/dsp/Source/FastMathFunctions`` 中的 ``arm_sin_cos_f32`` (已是 CMSIS-DSP 源码, AT 已验证, 直接拷贝到 ``libraries/dsp/`` 下). 该函数内部用 512 点查表 + 插值, 单次调用 ~20 cycles, ISR 内开销可忽略. 如拷贝困难则自研 512 点 sin 查表 (``foc_core.c`` 内), 实现等价.

> 注: ``libraries/dsp/`` 在 LV demo 中已存在, 本工程可整体拷贝该目录作为 DSP 数学件 (属 CMSIS-DSP 开源, Apache-2.0 许可, 无分发限制). 这与 §0.3 "全自研" 不冲突 —— 自研指的是控制算法 (Clarke/Park/SVPWM/PID), 三角函数查表属基础数学件, 复用 CMSIS-DSP 是工业惯例.

### 4.2 自研: SVPWM 三相高边特化
### 4.2 自研: SVPWM 三相高边特化

#### 4.2.1 算法选择

经典 7 段 SVPWM 算法 (与 LV `mc_foc_kernal.lib` 内部算法等价, 但**针对 MP6540H 3 路高边输入**适配).

LV demo 输出 6 路互补 PWM (UH/UL/VH/VL/WH/WL), 死区由 TMR1 死区寄存器生成.
本工程输出 3 路 PWM (PWMA/PWMB/PWMC -> MP6540H INHA/B/C), 低边由 MP6540H 内部互补生成.

差异点:
- TMR1 不使能互补通道 (CHxN)
- TMR1 不配死区 (DTG = 0)
- TMR1 BRK 输入不接 (留作软件故障关断)
- SVPWM 占空比直接写 CCR1/CCR2/CCR3, 不需要考虑互补极性

#### 4.2.2 SVPWM 计算流程

```c
void foc_svpwm_3phase_high_side(float v_alpha, float v_beta, float vbus,
                                 uint16_t *ta, uint16_t *tb, uint16_t *tc)
{
    /* 1. 输入归一化: m = sqrt(va^2 + vb^2) / vbus, theta = atan2(vb, va) */
    /* 2. 扇区判断: 0..5, 经典做法用 va/vb 符号 + |va|/sqrt(3) vs |vb| 比较 */
    /* 3. 计算 T1, T2 (相邻两个有效矢量作用时间) */
    /* 4. 计算 T0 = ARR - T1 - T2 */
    /* 5. 按扇区映射到 Tcm1/Tcm2/Tcm3 */
    /* 6. 硬限幅: each Tcmx in [0, ARR * 0.95] */
    /* 7. 输出到 ta/tb/tc */
}
```

参考实现来源: 公开教材标准实现 (机械工业出版社《永磁同步电机矢量控制 MATLAB/Simulink 仿真》第 3 章) + LV `mc_foc.c` 中的扇区映射表 (该部分是 .c 源码, 非 .lib).

#### 4.2.3 单元测试 (主机端 gcc)

```c
/* tests/foc_core/test_svpwm.c */
- 6 个扇区中心点 (theta = 30°, 90°, 150°, 210°, 270°, 330°) 输出与解析解对拍
- 调制比 m = 0.5 / 0.866 / 1.0 输出范围检查
- m > 1.0 时硬限幅生效
- 零输入 (v_alpha = v_beta = 0) 输出 50% 三相同
```

### 4.3 自研: 电流采样

#### 4.3.1 MP6540H 电流采样硬件原理

依据 MP6540H 数据手册 (`doc/MP6540H.pdf`) 第 7 页 / 第 14 页 Figure 2:

- 电流镜比例: `ISOUT = ILOAD / 9200` (典型, min 1/10500, max 1/7800, 容差 ±15%)
- 输出电压公式: `V_SO = V_REF + (I_LOAD × R_TERM) / 9200`
- 推荐拓扑: SO 引脚上下各接等值电阻到 ADC 供电与 GND, 使零电流时 V_SO = V_REF = ADC_supply / 2 (数据手册原文 "ADC code is half-scale at zero current")

本工程硬件: 每个 SO 引脚通过 **4.7 kΩ 上拉到 3.3 V + 4.7 kΩ 下拉到 GND**, 完全符合数据手册推荐做法.

#### 4.3.2 标度计算 (基于实际硬件)

```
V_REF       = 3.3 V × 4.7k / (4.7k + 4.7k) = 1.65 V   (零电流偏置)
R_TERM_eq   = 4.7k || 4.7k = 2.35 kΩ                   (Thevenin 等效)
ΔV_SO/ΔI    = R_TERM_eq / 9200 = 0.2554 V/A            (typ, ±15% 容差)
ADC_LSB     = 3.3 V / 4096 = 0.806 mV/LSB
LSB_to_A    = 0.806 mV / 0.2554 V/A ≈ 3.16 mA/LSB     (typ)
零电流 ADC  = 2048 LSB (12-bit half-scale)
满量程电流  = ±2048 × 3.16 mA = ±6.47 A (typ)
```

宏定义 (写入 `application/motor_control/motor_params.h`):

```c
/* MP6540H + 4.7k/4.7k pull-up/pull-down -> 3.3V/GND */
#define MP6540H_VREF_VOLTS              1.65f
#define MP6540H_RTERM_OHMS              2350.0f
#define MP6540H_MIRROR_RATIO_TYP        (1.0f / 9200.0f)
#define MP6540H_VSO_PER_AMP_TYP         0.2554f             /* V/A */

#define ADC_VREF_VOLTS                  3.3f
#define ADC_BITS                        12u
#define ADC_FULL_SCALE                  4096u
#define ADC_LSB_VOLTS                   (ADC_VREF_VOLTS / (float)ADC_FULL_SCALE)

#define CURRENT_ZERO_OFFSET_LSB         2048u                /* half-scale */
#define CURRENT_GAIN_DEFAULT_A_PER_LSB  (ADC_LSB_VOLTS / MP6540H_VSO_PER_AMP_TYP)
                                                             /* ~3.16 mA/LSB typ */

#define IQ_OVERCURRENT_A                5.0f                  /* 软件过流阈值, 低于满量程 6.47 A */
#define IMBALANCE_THRESHOLD_A           1.5f                  /* (ia+ib+ic) 不平衡阈值 */
```

#### 4.3.3 零偏标定与 gain 标定 (Stage 3 流程)

零偏标定 (每次使能前自动执行, 用于补偿电阻误差和 MCU ADC 失调):

```
motor_control_state = DISABLED
PWM 输出 50% (三相同电位, 不出力, low-side 都在导通)
等待 100 ms 稳定
采 N = 1024 次 ADC 注入序列
ia_offset = mean(ia_raw[0..N-1])
ib_offset = mean(ib_raw[0..N-1])
ic_offset = mean(ic_raw[0..N-1])
校验: |ia_offset - 2048| < 20 LSB, 否则进入 FAULT_SENSOR
```

Gain 单板标定 (一次性, 写入 flash 或硬编码, Stage 3 台架标定时执行):

```
限流电源给定 1 A 直流电流过 U 相 (W 相回流, V 相断开)
读 100 次平均: ia_avg, ib_avg, ic_avg
gain_a = 1.0 / (ia_avg - ia_offset)
gain_b 与 gain_c 同理 (依次注 1 A 到 V/W)
```

#### 4.3.4 实时采样转换

```c
float current_sense_calc(uint16_t raw, float offset_lsb, float gain_a_per_lsb)
{
    return ((float)raw - offset_lsb) * gain_a_per_lsb;
}
```

#### 4.3.5 三相一致性校验 (FOC ISR 内)

```c
if (fabsf(ia + ib + ic) > IMBALANCE_THRESHOLD_A) {
    fault_manager_set(FAULT_OVERCURRENT);  /* 电流不平衡, 视作过流 */
}
```

#### 4.3.6 VBUS 母线电压采样 (分压比 1/6)

```c
#define VBUS_DIVIDER_RATIO              6.0f
#define VBUS_VOLTS_PER_LSB              (ADC_LSB_VOLTS * VBUS_DIVIDER_RATIO)
                                        /* 4.834 mV/LSB, 满量程 19.8 V */

#define VBUS_UNDERVOLTAGE_THRESHOLD_V   8.0f
#define VBUS_OVERVOLTAGE_THRESHOLD_V    18.0f
```

VBUS 在 1 kHz 检查中读取, ADC2 普通转换序列 (与电流注入序列并存, 不冲突).
### 4.4 PID 参数初值 (浮点实现)

PID 控制器采用浮点抗 windup 实现 (全自研, 不依赖 AT 定点 PID). 结构体:

```c
typedef struct {
    float kp;
    float ki;
    float kd;            /* 当前阶段都为 0, 保留接口 */
    float integral;
    float integral_limit; /* 积分限幅 */
    float out_limit;      /* 输出限幅 */
    float last_error;
} pid_f32_t;

float pid_f32_exec(pid_f32_t *pid, float error);
```

参数初值 (写入 `motor_params.h`):

| 控制环 | Kp | Ki | Kd | 积分限幅 | 输出限幅 | 备注 |
| --- | --- | --- | --- | --- | --- | --- |
| Id 电流环 | 0.5 | 100 | 0 | ±Vbus/2 | ±Vbus/2 | 后期标定 |
| Iq 电流环 | 0.5 | 100 | 0 | ±Vbus/2 | ±Vbus/2 | 后期标定 |
| 速度环 | 0.01 | 0.5 | 0 | ±IQ_MAX_A | ±IQ_MAX_A (8 A) | 后期标定 |
| 位置环 | 5.0 | 0 | 0 | 0 | ±RPM_MAX (3000) | 纯 P 起步 |

> 所有 PID 参数都通过 `motor_params.h` 宏定义, 后期可改为 flash 参数表. 单位: 电流环 Kp 单位 V/A, Ki 单位 V/(A·s); 速度环 Kp 单位 A/rpm, Ki 单位 A/(rpm·s); 位置环 Kp 单位 rpm/deg.
### 4.5 电角度计算与零点标定 (功能需求)

#### 4.5.1 极对数

```c
#define MOTOR_POLE_PAIRS    7u    /* 2808 BLDC 默认 7 对极, 实测后修正 */
```

#### 4.5.2 电角度公式

```c
/* mech_raw / mech_zero_raw / mech_diff 均为 0..65535 的 16-bit 角度
 * (MA600A 输出 12-bit 角度, 左移 4 位扩展为 16-bit 便于计算)
 */
uint16_t mech_diff = (mech_raw - mech_zero_raw) & 0xFFFFu;
float    theta_e   = ((float)mech_diff * (float)MOTOR_POLE_PAIRS * 2.0f * 3.14159265f) / 65536.0f;
/* theta_e 范围 [0, 14π) 需对 2π 取模 */
theta_e = fmodf(theta_e, 2.0f * 3.14159265f);
```

#### 4.5.3 零点标定策略

零点标定是 motor_control 的**显式功能模式**, 不是开机自动行为. 用户/上位机通过 CAN 协议主动触发, 或调试时通过 finsh 手动触发.

启动流程:
```
1. 切换 motor_control 到 ALIGN 模式 (新增模式 4)
2. SVPWM 输出固定 (Vd_align, Vq=0, theta_e=0), 持续 500 ms
   - Vd_align 取值: 标定电流 1 A 对应的 d 轴电压, 由 V = R_phase × I 估算
3. 期间转子被强制对齐到 d 轴 0° 方向
4. 持续读 MA600A 角度, 最后 100 ms 取平均, 记为 mech_zero_raw
5. 写入 motor_params 全局变量, 后期 (Stage 9) 持久化到 flash
6. 切换回 DISABLED 模式
```

宏定义:
```c
#define ZERO_ALIGN_CURRENT_A            1.0f          /* 标定电流 */
#define ZERO_ALIGN_HOLD_MS              500u          /* 对齐持续时间 */
#define ZERO_ALIGN_SAMPLE_WINDOW_MS     100u          /* 角度采样窗口 */
extern uint16_t g_motor_zero_raw;                     /* mech_zero_raw, 初值 0 */
```

CAN 协议层面: 控制帧 `mode` 字段新增 `4 = ALIGN`. 控制帧的 `target_value` 字段在 ALIGN 模式下忽略.

#### 4.5.4 极对数动态修正

极对数在调试阶段可能需要根据实测修正. 提供调试接口:
```c
void motor_set_pole_pairs(uint8_t pp);    /* 仅在 DISABLED 状态下允许调用 */
```
通过 finsh shell 命令暴露, 比赛固件发布前固定为实测值.

### 4.6 安全约束 (代码层面强制)

- `motor_pwm_enable_output()` 仅在 `motor_control_state == ENABLED` 时返回成功
- 任何 `fault_manager_set()` 调用都同步调用 `motor_pwm_disable_output()`
- 启动时 PWM 输出默认 50% (三相同电位), MP6540H_EN = 低 (禁用)
- `motor_control_enable()` 流程: 检查故障 -> 启动零偏标定 -> 标定完成 -> PWM 50% -> MP6540H_EN = 高 -> 进入控制模式
- SVPWM 占空比硬限幅在 `foc_svpwm_3phase_high_side()` 内强制, 调用方传入超限值会被裁剪


### 4.7 旁轴磁编非线性标定 (功能需求)

#### 4.7.1 背景

MA600A 与磁环采用**旁轴 (off-axis) 布局**, 磁场偏心导致 MA600A 输出角度 θ_raw 与真实机械角 θ_true 之间存在与机械角相关的周期性误差:

```
e(θ_true) = θ_raw - θ_true
```

旁轴误差以 1 次和 2 次谐波为主, 峰峰值典型 ±0.5° ~ ±5°, 与转速无关 (纯几何误差), 因此可一次性离线标定后存 FLASH, 运行时查表校正:

```
θ_corrected = θ_raw - lookup(θ_raw)
```

#### 4.7.2 标定原理: 正反恒速拖动对消法

无外部基准, 仅靠电机自身完成. 原理:

1. 电机**恒速正转**记录 θ_raw(t), 速度脉动 + 几何误差混合在内
2. 电机**恒速反转**记录 θ_raw(t), 速度脉动反号, 几何误差同号
3. 取平均 (θ_fwd + θ_rev) / 2 → 速度脉动消除, 剩下纯几何误差作为"假基准"
4. 残差 e(θ_raw) = θ_raw - 假基准, 按 θ_raw 分箱 256 段直方图平均, 得到校正表

业内成熟做法 (ODrive / moteus / SimpleFOC 同款).

#### 4.7.3 数据结构 (存 FLASH 末页 0x0801FC00)

```c
#define CAL_FLASH_ADDR              0x0801FC00u
#define CAL_FLASH_SIZE              1024u
#define CAL_MAGIC                   0x304C4143u   /* 'CAL0' little-endian */
#define CAL_VERSION                 1u
#define CAL_TABLE_POINTS            256u

typedef struct {
    uint32_t magic;                       /* CAL_MAGIC */
    uint8_t  version;                     /* CAL_VERSION */
    uint8_t  reserved[3];
    uint32_t timestamp_ms;                /* 标定完成时 SysTick 计数, 仅作记录 */
    int16_t  table[CAL_TABLE_POINTS];     /* 旁轴非线性误差表, 单位 0.001° */
    uint16_t mech_zero_raw;               /* MA600A 零点 (16-bit 扩展) */
    uint8_t  pole_pairs;                  /* 标定时使用的极对数 */
    uint8_t  reserved2;
    uint32_t crc32;                       /* table + mech_zero_raw + pole_pairs 的 CRC32 */
} motor_calibration_t;                    /* 总大小 528 字节 */
```

#### 4.7.4 标定参数

```c
#define CAL_TURNS_PER_DIRECTION     5u            /* 每方向圈数 */
#define CAL_SPIN_SPEED_RPM          30            /* 标定转速 */
#define CAL_SAMPLES_PER_TURN        4096u         /* 每圈采样点数 (限于 RAM, 5 * 4096 * 4 = 80 KB 不可行) */
#define CAL_SAMPLES_PER_DIRECTION   (CAL_TURNS_PER_DIRECTION * CAL_SAMPLES_PER_TURN)
                                                  /* 5 * 4096 = 20480 个 raw 点 / 方向 */
```

> **关键工程约束 (RAM 限制)**: AT32M412 总 RAM 16 KB, 不能在内存中保留 20480 个 raw 角度点 (40 KB). 实际实现采用**流式直方图**: 边采集边累加到 256 段直方图 (256 × 2 × 4 = 2 KB), 不保留 raw 序列. 速度脉动对消通过"正反两次直方图相减"实现, 数学上等价于先存原始序列再正反平均.

#### 4.7.5 标定状态机 (在 motor_control 状态机内嵌)

```
CAL_IDLE
    ↓ (CAN/finsh 触发 CALIBRATE)
CAL_ZERO_ALIGN          (沿用 §4.5.3 ALIGN 流程, 500 ms)
    ↓
CAL_SPIN_FWD            (速度环 +30 RPM, 5 圈, ~10 s)
    数据流: 每 FOC ISR 累加 (θ_raw, t) → hist_fwd[256] 直方图
    ↓
CAL_SPIN_REV            (速度环 -30 RPM, 5 圈, ~10 s)
    数据流: 同上, 累加到 hist_rev[256]
    ↓
CAL_COMPUTE             (在 RT-Thread 主循环, ~1 s)
    1. 对每个 idx ∈ [0, 256): e[idx] = (hist_fwd[idx] + hist_rev[idx]) / 2
    2. 去直流: e -= mean(e), 保证表的平均偏移为 0
    3. 限幅: clip(e, ±32767)  (i16 范围, 单位 0.001°, ±32.767°)
    4. 计算 CRC32
    ↓
CAL_WRITE_FLASH         (~100 ms, FLASH 擦除末页 + 写入 528 字节)
    ↓
CAL_DONE → CAL_IDLE
    motor_control_state 切回 DISABLED, 通过状态帧上报 CALIBRATION_DONE
```

中止规则:
- 任意阶段进入 FAULT → 中止, 旧标定保留, FLASH 不写入
- 用户发 enable=0 → 中止, 同上
- ``cal_progress`` 字段通过 §5.4 CALIBRATE 状态帧上报 0~100

#### 4.7.6 运行期查表 (FOC ISR 内)

```c
/* 在 motor_encoder_at32m412_read_angle() 内执行 */
uint16_t raw_16 = (uint16_t)(ma600a_raw_12bit << 4);   /* 12-bit -> 16-bit 扩展 */

/* 256 点线性插值 */
uint32_t idx_frac_q24 = (uint32_t)raw_16 * 256u;       /* Q24 fixed-point */
uint16_t idx = (uint16_t)(idx_frac_q24 >> 16);          /* 0..255 */
uint16_t frac = (uint16_t)(idx_frac_q24 & 0xFFFFu);     /* Q16 */
int16_t  off0 = g_cal.table[idx];
int16_t  off1 = g_cal.table[(idx + 1u) & 0xFFu];
int32_t  off_mdeg = off0 + (((int32_t)(off1 - off0) * frac) >> 16);  /* 单位 0.001° */

/* 转换为 16-bit 角度单位再修正 */
int32_t off_raw = (off_mdeg * 65536) / 360000;  /* 0.001° -> raw16 LSB */
uint16_t raw_corrected = (uint16_t)((int32_t)raw_16 - off_raw);
```

ISR 开销: ~12 个时钟周期 = 0.13 µs, ISR 时间预算几乎不变.

#### 4.7.7 开机加载 + 故障处理

开机 ``motor_app_init()`` 流程:

```
1. 从 0x0801FC00 读取 motor_calibration_t (528 字节, 直接读 FLASH)
2. 校验 magic == CAL_MAGIC, version == CAL_VERSION
3. 校验 CRC32(table + mech_zero_raw + pole_pairs) == 存档值
4. 全部通过 → 拷贝到 RAM g_cal, 设置 g_cal_valid = true
5. 任一项失败 → g_cal_valid = false, table 清零 (查表退化为不校正),
   设置 fault_flags |= FAULT_CAL_INVALID
   上报给主机 / 启动后允许 CALIBRATE 模式重做标定
```

注意: ``FAULT_CAL_INVALID`` 不阻止电机使能, 但应作为告警, 提示用户精度可能未达标.

#### 4.7.8 触发接口

CAN 协议: 控制帧 mode = 5 (见 §5.2), enable_flag 必须先为 1.

finsh 命令 (开发调试用):
```
mc_calibrate       # 触发标定
mc_cal_status      # 查询当前状态机阶段
mc_cal_dump        # 打印 256 点表
mc_cal_erase       # 擦除 FLASH 标定区
```

#### 4.7.9 验证标准 (Stage 4b 验收点)

- 标定完成后 ``cal_max_residual`` < ±1.0° (峰峰 2°)
- 重复 3 次标定, 两两表的最大差 < ±0.2°
- 标定后再用电流环测试 (Stage 5), 转矩纹波应降低 50% 以上 (示波器看 iq 波形脉动)
## 5. CAN 协议

### 5.1 帧格式 (标准 11-bit ID, 500 kbps)

节点 ID 范围: 0x00 ~ 0x7F (本设计默认 0x01, 可通过宏 `MOTOR_NODE_ID` 改).

CAN ID 编码:
`CAN_ID = (function_code << 7) | node_id`

| function_code | 类型 | 用途 |
| --- | --- | --- |
| 0x02 | 主机 -> 节点 | 控制帧 |
| 0x03 | 节点 -> 主机 | 状态帧 (100 Hz) |
| 0x05 | 节点 -> 主机 | 扩展状态帧 (50 Hz) |

例如节点 ID = 0x01, 则:
- 控制帧 CAN_ID = `(0x02 << 7) | 0x01 = 0x101`
- 状态帧 CAN_ID = `(0x03 << 7) | 0x01 = 0x181`
- 扩展状态帧 CAN_ID = `(0x05 << 7) | 0x01 = 0x281`

### 5.2 控制帧 (CAN ID 0x101, DLC = 8)

| Byte | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | enable_flag | u8 | 0=disable, 1=enable |
| 1 | mode | u8 | 0=open_loop, 1=current, 2=speed, 3=position, 4=align, 5=calibrate |
| 2 | fault_clear | u8 | 1=clear fault (单次脉冲, 节点接收后回零) |
| 3 | reserved | u8 | 0 |
| 4-7 | target_value | i32 LE | 含义随 mode: 0=duty×10000, 1=mA, 2=rpm, 3=mdeg |

### 5.3 状态帧 (CAN ID 0x181, DLC = 8, 100 Hz 自动上报)

| Byte | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | state | u8 | 0=disable, 1=enable, 2=fault |
| 1 | mode | u8 | echo 当前控制模式 |
| 2-3 | fault_flags | u16 LE | bitmap (见 5.5) |
| 4-5 | vbus | u16 LE | 母线电压, 单位 0.1 V (1 LSB = 100 mV) |
| 6-7 | iq_actual | i16 LE | 实际 q 轴电流, 单位 mA |

### 5.4 扩展状态帧 (CAN ID 0x281, DLC = 8, 50 Hz 自动上报)

| Byte | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0-3 | rpm | i32 LE | 实际速度 |
| 4-7 | angle | i32 LE | 实际位置, 单位 mdeg (0.001°) |

### 5.5 故障位图 (16-bit bitmap)

| Bit | 名称 | 触发源 | 是否可清 |
| --- | --- | --- | --- |
| 0 | FAULT_DRIVER | MP6540H nFAULT (PB2) | 硬件恢复后可清 |
| 1 | FAULT_OVERCURRENT | FOC ISR Iq/Id 软件阈值 | 可清 |
| 2 | FAULT_SENSOR | MA600A SPI 超时或 CRC 错 | 可清 |
| 3 | FAULT_UNDERVOLTAGE | 1 kHz Vbus < 阈值 | 自动清 (Vbus 恢复) |
| 4 | FAULT_OVERVOLTAGE | 1 kHz Vbus > 阈值 | 自动清 |
| 5 | FAULT_CAN_TIMEOUT | 500 ms 未收 CAN 控制帧 (仅 CAN 模式下) | 自动清 (收到新帧) |
| 6 | FAULT_CAL_INVALID | 标定数据 CRC 错或 magic 不匹配 | 可清 (重新标定后自动清) |
| 7-15 | reserved | - | - |

### 5.6 安全策略

- 节点上电默认 `state = DISABLED`, `mode = OPEN_LOOP`, PWM 关
- 收到 enable=1 + mode != 当前模式 时, 先切模式 (清电流环积分) 再使能
- 任意时刻 `fault_flags != 0`, `state = FAULT`, PWM 强制关, 控制帧仅响应 fault_clear
- CAN 控制超时: 在 `enable == 1` 且模式 != OPEN_LOOP 时, 若 500 ms 无控制帧 -> `FAULT_CAN_TIMEOUT` -> 软停
- OPEN_LOOP 模式不做 CAN 超时检查 (用于台架手动调试)

### 5.7 主机端示例

```python
# Python + python-can 示例 (作品提交时附在源码包)
import can
bus = can.Bus(channel='can0', bustype='socketcan', bitrate=500000)

# Enable + 设置速度 1000 rpm
msg = can.Message(arbitration_id=0x101, data=[1, 2, 0, 0,
                                              0xE8, 0x03, 0, 0],
                  is_extended_id=False)
bus.send(msg)
```

## 6. 测试策略

### 6.1 主机端单元测试 (gcc / arm-none-eabi-gcc -fsyntax-only)

| 测试套件 | 范围 | 验证手段 |
| --- | --- | --- |
| `test_motor_control_state.c` | 状态机转换 | assert 主机执行 (已有) |
| `test_foc_clarke_park.c` | Clarke/Park/IPark 数表对拍 | 与 MATLAB 计算结果对比 |
| `test_svpwm.c` | SVPWM 6 扇区中心点 + 限幅 | 解析解对拍 |
| `test_pid.c` | PID 输出 + 抗 windup | 阶跃响应 |
| `test_can_protocol.c` | 帧编解码 | 字节序 / 字段提取 |
| `test_ma600a_static.py` | MA600A 协议层 | (已有, 8 个用例已过) |

### 6.2 板端验证

每个阶段必须完成的台架验收点:

| 阶段 | 验收方式 | 通过标准 |
| --- | --- | --- |
| Stage 1 PWM | 示波器看 PWMA/B/C | 16 kHz, 中心对齐, 50% 占空比, 三相对齐 |
| Stage 2 开环 | MP6540H_EN=高, 限流电源, 看三相电流 | 三相正弦波, 平衡度 < 5%, 电流 < 设定值 |
| Stage 3 电流采样 | 注 ADC 转换值与已知电流对比 | 误差 < 5%, 零偏漂移 < 0.1 A |
| Stage 4 MA600A | 手转电机轴, 看角度连续性 | 0-360° 无跳变, 方向与硬件文档一致 |
| Stage 4b 标定 | 触发 mc_calibrate, 等待 25 s | cal_max_residual < 1°, 重复一致性 < 0.2°, 转矩纹波下降 50%+ |
| Stage 5 电流环 | iq_ref 阶跃, 看 iq_actual | 上升时间 < 1 ms, 稳态误差 < 5% |
| Stage 6 速度环 | rpm_ref 阶跃, 看 rpm_actual | 上升时间 < 50 ms, 超调 < 20%, 稳态误差 < 5% |
| Stage 7 位置环 | mdeg_ref 阶跃, 看 angle_actual | 稳态误差 < 0.5°, 无持续抖动 |
| Stage 8 CAN | python-can 发控制帧 + 读状态帧 | 收发正常, 超时保护生效 |
| Stage 9 保护 | 模拟各故障源 | 全部进入 FAULT 态, fault_clear 后可恢复 |

### 6.3 测试与记录的连接

每次台架测试必须按 `doc/FOC控制器开发记录.md §6` 的调试记录模板填一条, 并附波形截图.

参数标定每次按 §7 参数调试记录模板填.


## 7. 阶段任务清单 (落实到 CLAUDE.md / FOC控制器开发记录.md 的 Stage 0~9)

### Stage 0: 基线整理与目录重构 (1-2 天)

- 拷贝 LV demo 的 ``libraries/dsp/`` 目录到本工程 (CMSIS-DSP FastMath, 用于 ``arm_sin_cos_f32``)
- 建立 `application/motor_control/` 下新文件骨架 (空函数 + .h 接口声明)
- 建立 `platform/at32m412/` 下新文件骨架
- 建立 `communication/` 目录
- 更新 `CMakeLists.txt` 把新文件纳入构建, 验证 WSL `cmake --build` 通过 (空函数即可)
- `project/src/at32m412_416_wk_config.c` 中所有 `wk_*_init()` 函数体清空 (保留函数名以兼容 `wk_config.h` 声明)
- `project/src/main.c` 改为只调用 `motor_app_init() / motor_app_run()`
- 验证: WSL 构建通过, FLASH 占用基线记录

### Stage 1: 硬件 Bring-up (2-3 天)

- 实现 `clock_at32m412.c` 系统时钟 (沿用 wk_system_clock_config, 180 MHz; spec §1.1 原标 96 MHz 已更正为 180 MHz)
- 实现 `motor_pwm_at32m412.c` 最小集: TMR1 中心对齐 + 16 kHz + 3 路 PWM 输出 + 安全关断
- 实现 `board_motor_pins.h` 集中定义所有引脚常量 + `board_init_at32m412.c` (外设时钟/GPIO/NVIC)
- 主程序: 上电 -> 时钟 -> board_init (时钟/GPIO/NVIC) -> PWM 50% 输出 -> MP6540H_EN 保持低
- 验收: 示波器看 PA8/PA9/PA10 = 16 kHz 中心对齐 50%, 三相对齐

### Stage 2: PWM 与开环控制 (2-3 天)

- 实现 `foc_core.c` SVPWM (无电流环)
- 实现 `motor_control_isr.c` ISR 骨架, 但只跑 OPEN_LOOP 模式
- 实现 `motor_pwm_at32m412_enable_output()` (拉高 MP6540H_EN)
- 主程序: 手动设置 OPEN_LOOP 模式 + 固定占空比 + 固定电角度递增
- 验收: 限流电源 12 V, MP6540H_EN=高, 看三相电流波形为正弦 (电机不一定转, 因为没接编码器同步)

### Stage 3: ADC 同步采样与电流反馈 (2-3 天)

- 实现 `current_sense_at32m412.c` ADC2 注入序列 + TMR1_CH4 触发 + 零偏标定
- 在 ISR 内读 ADC, 通过全局变量暴露 ia/ib/ic 给调试观察
- 实现三相一致性校验
- 验收: 注已知电流 (限流电源给 0.5 A), 读 ADC 反算电流, 误差 < 5%

### Stage 4: MA600A 有感角度闭环准备 (1-2 天)

- 实现 `motor_encoder_at32m412.c` 封装现有 MA600A 驱动
- 在 ISR 内调用 `motor_encoder_read_angle_speed()`, 加 SPI 超时保护
- 实现机械角到电角度转换 (考虑极对数 + 零点偏移)
- 在 OPEN_LOOP 模式下, 输出电角度 = MA600A 读出电角度, 验证开环跟踪
- 验收: 手转电机, 看波形 ia 与电角度同步变化


### Stage 4b: 旁轴非线性标定 (新增, 1-2 天)

接续 Stage 4 MA600A 角度读取与零点对齐, 在闭环之前完成旁轴误差补偿.

- 实现 `motor_calibration.[ch]` 标定状态机 (CAL_IDLE → CAL_ZERO_ALIGN → CAL_SPIN_FWD → CAL_SPIN_REV → CAL_COMPUTE → CAL_WRITE_FLASH → CAL_DONE)
- 实现 256 段直方图采集器 (在 FOC ISR 内累加)
- 实现 FLASH 标定区读写 (`flash_calibration.[ch]`, 调用 AT32 flash 驱动擦除末页 + 写 528 字节)
- 实现 CRC32 校验工具函数 (AT32M412 有硬件 CRC, 直接调用)
- 实现 finsh 命令: `mc_calibrate / mc_cal_status / mc_cal_dump / mc_cal_erase`
- 在 `motor_app_init()` 中加入开机加载逻辑, 失败时设置 FAULT_CAL_INVALID
- 在 `motor_encoder_at32m412_read_angle()` 内加入查表校正
- 修改 `AT32M412xB_FLASH.ld`: FLASH `LENGTH = 127K`, 末 1 KB 给标定区
- 验收 (台架):
  1. 触发标定: `mc_calibrate`, 电机自动正反拖动 5+5 圈, 约 25 s 完成
  2. `cal_max_residual` < 1° 峰峰
  3. 重复 3 次标定, 两两表差 < 0.2°
  4. 与未校正情况比较, 转矩纹波下降 50% 以上 (示波器看 iq)
  5. 断电重启, 校正自动加载, fault_flags 中 FAULT_CAL_INVALID 不置位
  6. 故意把 FLASH 末页擦除, 重启, FAULT_CAL_INVALID 置位, 标定可重新执行
### Stage 5: 电流环控制 (3-4 天)

- 实现 `current_loop.c` Id/Iq PI (调用 `MC_PIDExec`)
- 在 ISR 内启用 CURRENT 模式分支
- 标定 Kp/Ki 初值, 阶跃响应测试
- 验收: iq_ref 阶跃 0.5A->1A, iq_actual 上升时间 < 1 ms, 稳态误差 < 5%

### Stage 6: 速度环控制 (2-3 天)

- 实现 `speed_loop.c` 速度 PI + 一阶低通
- 在 ISR 内分频 1 kHz 启用 SPEED 模式分支
- 验收: rpm_ref 阶跃, 上升时间 < 50 ms, 超调 < 20%

### Stage 7: 位置环 (2-3 天)

- 实现 `position_loop.c` 位置 P/PI + 角度跨零处理
- 在 ISR 内分频 200 Hz 启用 POSITION 模式分支
- 验收: 阶跃 0 -> 90°, 稳态误差 < 0.5°

### Stage 8: CAN 指令控制 (3-4 天)

- 实现 `can_at32m412.c` CAN1 收发 + RX FIFO 中断
- 实现 `can_protocol.c` 帧编解码 + 协议解析
- 实现 RT-Thread 任务: CAN 控制帧解析 / 状态帧定时发送 / 超时检测
- 验收: python-can + USB-CAN 工具收发, 控制 + 状态上报正常, 超时保护生效

### Stage 9: 保护 / 参数 / 发布 (2-3 天)

- 实现 `motor_protect_at32m412.c` nFAULT EXINT + Vbus ADC 监测
- 实现 `fault_manager.c` 故障锁存与清除
- 整理参数表, `motor_params.h` 集中所有可调参数
- 编写编译说明 / 烧录说明 / CAN 协议文档
- 验收: 模拟各故障源, 全部进入 FAULT 态并可清除

### 总工期估算

| 阶段 | 工期 (天) |
| --- | --- |
| Stage 0 基线整理 | 1-2 |
| Stage 1 硬件 Bring-up | 2-3 |
| Stage 2 PWM 开环 | 2-3 |
| Stage 3 ADC 电流 | 2-3 |
| Stage 4 MA600A | 1-2 |
| Stage 5 电流环 | 3-4 |
| Stage 6 速度环 | 2-3 |
| Stage 7 位置环 | 2-3 |
| Stage 8 CAN | 3-4 |
| Stage 9 保护与发布 | 2-3 |
| 合计 | 20-30 天 |

距比赛截止 (2026-07-31) 约 40 天, 工期可控, 留约 8 天给作品帖 / 视频 / 修正.

## 8. 风险与缓解

| 风险 | 概率 | 影响 | 缓解 |
| --- | --- | --- | --- |
| MP6540H 电流采样 SNR 不足 | 中 | 电流环不稳 | 已采用数据手册推荐拓扑 (4.7k/4.7k 分压); Stage 3 若 SNR < 30 dB 再加软件 IIR 滤波 |
| SVPWM 扇区判断 bug | 低 | 三相不平衡 | 主机端 6 扇区单元测试 + 示波器三相对齐验证 |
| MA600A SPI 偶发超时 | 低 | FOC 周期抖动 | SPI 超时保护 + 故障锁存; 若高频出现切 DMA 异步 |
| RT-Thread 临界区阻塞 FOC ISR | 低 | 控制环抖动 | 本工程 libcpu 用 PRIMASK (非 BASEPRI), 短暂全局关中断; FOC ISR 不调 RT-Thread API; Stage 2/3 实测抖动超阈值再改 BASEPRI 方案 |
| CAN 总线高负载延迟 | 低 | 控制响应慢 | 状态上报降级到 50 Hz / 25 Hz 可配置 |
| Workbench 函数残留依赖 | 中 | Stage 0 编译错 | 全文搜索 `wk_.*_init` 调用点, 主程序入口完全重写 |


## 9. 未决问题 (本设计文档外, 工程推进时确认)

1. ~~USART1 引脚冲突~~ — 已解决: USART1 改到 PB6/PB7, SPI2 改到 PB3/PB4/PB5.
2. ~~MP6540H CS_gain~~ — 已解决: 基于硬件 4.7k/4.7k 分压网络 + 数据手册公式, typ 标度 ≈ 3.16 mA/LSB, 单板 gain 由 Stage 3 标定补偿 ±15% 镜像比例容差 (见 §4.3).
3. ~~MA600A 极对数与零点~~ — 已确认: 极对数默认 7 (调试时可调), 零点作为显式 ALIGN 功能模式 (见 §4.5).
4. ~~VBUS 分压比~~ — 已确认: 1/6, 满量程 19.8 V, 1 LSB = 4.834 mV (见 §4.3.6).
5. ~~CAN 节点 ID 与终端电阻~~ — 已确认: 默认 ID = 0x01, 板载 120 Ω 终端默认焊接 (见 §5.1).
6. ~~mclib license~~ — 已确认: AT mclib 开源部分仅滤波器/定点PID, 核心 .lib 不开源不可复用; 改为全自研, 三角函数复用 CMSIS-DSP (Apache-2.0). 无 license 风险.

**当前无遗留未决项**. Stage 3 与 Stage 4 的台架标定动作 (电流 gain 单板标定 / 极对数实测 / MA600A 零点) 在阶段任务清单中已规划, 不属于设计阶段未决问题.

## 10. 实现优先级与下一步

下一步 (用户 review 通过本设计文档之后):
1. 调用 `writing-plans` skill 生成详细实现计划
2. 按 Stage 0 -> Stage 9 顺序执行 (Stage 4 → 4b 是子序列, 不可跳过), 每个阶段独立 PR/commit
3. 每个阶段完成后必须更新 `doc/FOC控制器开发记录.md` 的 §5 开发进度表

每个阶段都遵守 `CLAUDE.md` 中已有规则:
- 不回退用户已有改动
- 不在 `wk_config.c` 等已废弃文件写业务逻辑
- 电机控制相关改动必须先确认安全默认态
- 每完成可验证节点追加 "调试记录"
- 所有硬件现象以实测记录为准, 不能用推测替代结论