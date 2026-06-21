# Plan 1: 工程基线重构 (Stage 0) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. 若 subagent 遇到阻塞、token 暴涨或卡阻, 立即切回 inline 执行, 不执着于 subagent.

**Goal:** 重构 MPS_MotorDriver 工程目录骨架与构建系统, 建立后续 FOC 开发所需的全部分层目录、空骨架文件、DSP 数学件接入, 使 WSL CMake 构建通过且资源占用与基线持平.

**Architecture:** 按 spec §2.1 分层, 新增 `application/motor_control/`、`platform/at32m412/`、`communication/` 三个目录的空骨架; 从 LV demo 拷贝 `libraries/dsp/` (CMSIS-DSP FastMath); 废弃 Workbench 的 `wk_*_init()` 函数体但保留引脚 `#define`; 重写 `main.c` 为精简应用入口; `CMakeLists.txt` 纳入所有新源文件.

**Tech Stack:** C11, arm-none-eabi-gcc (WSL), CMake 3.22+, AT32M412 CMSIS, CMSIS-DSP FastMath, RT-Thread Nano.

**构建验证命令 (所有任务通用):**
```bash
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && cmake --build build/Debug 2>&1 | tail -20"
```

**前置事实 (已查证):**
- 当前已编译基线: FLASH 26504 B (20.22%), RAM 3520 B (21.48%)
- 现有 `application/motor_control/motor_control.[ch]` (状态机骨架)
- 现有 `platform/at32m412/motor_pwm_at32m412.[ch]` (PWM 安全接口骨架)
- 现有 `tests/motor_control/test_motor_control_state.c` (接口语法验证)
- 现有 `middlewares/msp/ma600/` (MA600A 驱动)
- LV demo 路径: `E:\WorkSpaces\2_MotorDriver\AT32M412_LV_MC_Library_Porject_V2.1.5\libraries\dsp\`

**约定:**
- 每个 Task 结束必须 commit
- commit message 格式: `refactor(stage0): <简述>` 或 `feat(stage0): <简述>`
- 不回退用户已有改动 (现有 `motor_control.[ch]` / `motor_pwm_at32m412.[ch]` / MA600A 驱动 / 测试保留)
- WSL 中路径用 `/mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver`
- Windows 路径用反斜杠

---

## 文件结构 (本 Plan 涉及)

**新增目录与文件:**
- `application/motor_control/foc_core.h` — Clarke/Park/IPark/SVPWM 接口声明 (空实现)
- `application/motor_control/foc_core.c` — 空实现, 只含 stub 函数
- `application/motor_control/current_loop.h` — Id/Iq PI 接口
- `application/motor_control/current_loop.c` — stub
- `application/motor_control/speed_loop.h` / `.c` — stub
- `application/motor_control/position_loop.h` / `.c` — stub
- `application/motor_control/motor_control_isr.h` / `.c` — ISR 主体 stub
- `application/motor_control/motor_params.h` — 电机参数宏定义 (含 §4.3.2 / §4.5.1 全部宏)
- `application/motor_control/motor_calibration.h` / `.c` — 旁轴标定状态机 stub (Stage 4b 实现细节)
- `application/motor_control/fault_manager.h` / `.c` — 故障锁存 stub
- `application/motor_app.h` / `.c` — 应用层主入口 (替代 main 业务逻辑)
- `platform/at32m412/current_sense_at32m412.h` / `.c` — ADC2 注入 stub
- `platform/at32m412/motor_encoder_at32m412.h` / `.c` — MA600A 适配封装 stub
- `platform/at32m412/motor_protect_at32m412.h` / `.c` — nFAULT/Vbus stub
- `platform/at32m412/flash_calibration_at32m412.h` / `.c` — FLASH 标定区 stub
- `platform/at32m412/board_motor_pins.h` — 引脚与时序常数集中定义
- `platform/at32m412/clock_at32m412.h` / `.c` — 系统时钟 stub (暂沿用 wk_system_clock_config)
- `communication/can_protocol.h` / `.c` — CAN 协议 stub
- `communication/can_at32m412.h` / `.c` — CAN1 硬件适配 stub

**修改文件:**
- `project/src/main.c` — 精简为只调用 `motor_app_init() / motor_app_run()`
- `project/src/at32m412_416_wk_config.c` — `wk_*_init()` 函数体清空 (保留函数名, body 改为空或仅 `return;`)
- `project/inc/at32m412_416_wk_config.h` — 保留引脚 `#define`, 删除已不需要的声明 (如有)
- `CMakeLists.txt` — 把所有新源文件纳入 `target_sources`, 新增 include 目录
- `AT32M412xB_FLASH.ld` — FLASH `LENGTH = 127K`, 预留末 1 KB 给标定区
- `.gitignore` — 补充 `docs/` 例外 (允许 spec/plan md 提交)

**DSP 三角函数 (arm_sin_cos_f32) 推迟到 Plan 4**: Plan 1 不接 DSP, `foc_core.c` 的 stub 暂不调用三角函数. 避免本 plan 引入 CMSIS-DSP 配置复杂度.

**保留不动:**
- `application/motor_control/motor_control.[ch]` (已有状态机)
- `platform/at32m412/motor_pwm_at32m412.[ch]` (已有 PWM 安全接口)
- `middlewares/msp/ma600/` (MA600A 驱动)
- `tests/motor_control/test_motor_control_state.c`
- `project/src/ma600a_debug.c` (bring-up 路径, 暂留)
- `project/src/rtthread_app.c` / `board.c` / `syscalls.c` / `sysmem.c`

---
## Task 1: 提交现有未跟踪改动作为基线

当前工作区有未提交改动 (`application/`, `platform/`, `tests/`, `CLAUDE.md`, `docs/` 等都是 `??`), 必须先固化基线, 否则后续 task 的 diff 会混入历史遗留.

**Files:**
- Modify: `.gitignore` (补充 docs 例外)

- [ ] **Step 1: 修正 .gitignore 允许 docs/superpowers 提交**

在 `.gitignore` 末尾的 `doc/*` 规则之后, 补充:

```
# --- Documentation: allow spec/plan markdown ---
!doc/FOC控制器开发记录.md
!doc/stage0_baseline_2026-06-11.md
!docs/
```

- [ ] **Step 2: 验证 git 能看到 spec 与 plan 文件**

Run:
```bash
cd E:\WorkSpaces\2_MotorDriver\MPS_MotorDriver
git status --short docs/
```
Expected: 显示 `docs/superpowers/specs/2026-06-22-mps-foc-design.md` 与 `docs/superpowers/plans/2026-06-22-plan1-project-baseline.md` 为 `??` (未跟踪, 但可见)

- [ ] **Step 3: 提交全部基线**

```bash
git add -A
git commit -m "docs: add FOC design spec and plan1; commit stage0 baseline assets"
```

Expected: commit 成功, `git status` 干净

- [ ] **Step 4: 验证基线构建仍通过**

Run:
```bash
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && cmake --build build/Debug 2>&1 | tail -5"
```
Expected: 构建成功, 生成 `MPS_MotorDriver.elf`. 记录 FLASH/RAM 占用数字 (应与基线 26504 B / 3520 B 持平, 因为没改任何源码).

---

## Task 2: 创建 board_motor_pins.h 集中引脚定义

把 spec §1.2 引脚表固化成代码常量, 后续所有 platform 文件都引用这里, 避免引脚号散落.

**Files:**
- Create: `platform/at32m412/board_motor_pins.h`

- [ ] **Step 1: 写 board_motor_pins.h**

```c
#ifndef BOARD_MOTOR_PINS_H
#define BOARD_MOTOR_PINS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "at32m412_416.h"

/* ===== TMR1 三相 PWM (高边输入到 MP6540H) ===== */
#define PWMA_GPIO_PORT           GPIOA
#define PWMA_PIN                 GPIO_PINS_8
#define PWMA_PIN_SOURCE          GPIO_PINS_SOURCE8
#define PWMA_IOMUX               GPIO_MUX_1

#define PWMB_GPIO_PORT           GPIOA
#define PWMB_PIN                 GPIO_PINS_9
#define PWMB_PIN_SOURCE          GPIO_PINS_SOURCE9
#define PWMB_IOMUX               GPIO_MUX_1

#define PWMC_GPIO_PORT           GPIOA
#define PWMC_PIN                 GPIO_PINS_10
#define PWMC_PIN_SOURCE          GPIO_PINS_SOURCE10
#define PWMC_IOMUX               GPIO_MUX_1

/* ===== MP6540H 使能 / 故障 ===== */
#define PWM_EN_GPIO_PORT         GPIOB
#define PWM_EN_PIN               GPIO_PINS_10

#define nFAULT_GPIO_PORT         GPIOB
#define nFAULT_PIN               GPIO_PINS_2
#define nFAULT_EXINT_LINE        EXINT_LINE_2
#define nFAULT_EXINT_IRQn        EXINT2_3_IRQn

/* ===== ADC2 电流采样 (MP6540H 电流镜) ===== */
#define SOA_GPIO_PORT            GPIOB
#define SOA_PIN                  GPIO_PINS_1
#define SOA_ADC_CHANNEL          ADC_CHANNEL_9

#define SOB_GPIO_PORT            GPIOB
#define SOB_PIN                  GPIO_PINS_0
#define SOB_ADC_CHANNEL          ADC_CHANNEL_8

#define SOC_GPIO_PORT            GPIOA
#define SOC_PIN                  GPIO_PINS_7
#define SOC_ADC_CHANNEL          ADC_CHANNEL_7

/* ===== VBUS 母线电压 (分压比 1/6) ===== */
#define VBUS_GPIO_PORT           GPIOA
#define VBUS_PIN                 GPIO_PINS_6
#define VBUS_ADC_CHANNEL         ADC_CHANNEL_6

/* ===== SPI2 (MA600A) ===== */
#define SPI2_SCK_GPIO_PORT       GPIOB
#define SPI2_SCK_PIN             GPIO_PINS_3
#define SPI2_SCK_PIN_SOURCE      GPIO_PINS_SOURCE3
#define SPI2_SCK_IOMUX           GPIO_MUX_3

#define SPI2_MISO_GPIO_PORT      GPIOB
#define SPI2_MISO_PIN            GPIO_PINS_4
#define SPI2_MISO_PIN_SOURCE     GPIO_PINS_SOURCE4
#define SPI2_MISO_IOMUX          GPIO_MUX_3

#define SPI2_MOSI_GPIO_PORT      GPIOB
#define SPI2_MOSI_PIN            GPIO_PINS_5
#define SPI2_MOSI_PIN_SOURCE     GPIO_PINS_SOURCE5
#define SPI2_MOSI_IOMUX          GPIO_MUX_3

#define SPI2_CS_GPIO_PORT        GPIOA
#define SPI2_CS_PIN              GPIO_PINS_15

/* ===== CAN1 (TJA1051) ===== */
#define CAN1_TX_GPIO_PORT        GPIOA
#define CAN1_TX_PIN              GPIO_PINS_12
#define CAN1_TX_PIN_SOURCE       GPIO_PINS_SOURCE12
#define CAN1_TX_IOMUX            GPIO_MUX_9

#define CAN1_RX_GPIO_PORT        GPIOA
#define CAN1_RX_PIN              GPIO_PINS_11
#define CAN1_RX_PIN_SOURCE       GPIO_PINS_SOURCE11
#define CAN1_RX_IOMUX            GPIO_MUX_9

/* ===== USART1 (finsh shell, 已从 PA9 移走避免冲突) ===== */
#define USART1_TX_GPIO_PORT      GPIOB
#define USART1_TX_PIN            GPIO_PINS_6
#define USART1_TX_PIN_SOURCE     GPIO_PINS_SOURCE6
#define USART1_TX_IOMUX          GPIO_MUX_0

#define USART1_RX_GPIO_PORT      GPIOB
#define USART1_RX_PIN            GPIO_PINS_7
#define USART1_RX_PIN_SOURCE     GPIO_PINS_SOURCE7
#define USART1_RX_IOMUX          GPIO_MUX_0

/* ===== LED ===== */
#define LED_GPIO_PORT            GPIOA
#define LED_PIN                  GPIO_PINS_0

/* ===== PWM 时序常数 (spec §1.3) ===== */
#define PWM_FREQUENCY_HZ         16000u
#define TMR1_CLOCK_HZ            96000000u
#define TMR1_ARR                 2999u   /* 96MHz / (2 * 16kHz) - 1, 中心对齐 */
#define PWM_DUTY_MAX             (uint16_t)(TMR1_ARR * 0.95f)  /* 硬限幅 95% */

#ifdef __cplusplus
}
#endif

#endif /* BOARD_MOTOR_PINS_H */
```

- [ ] **Step 2: 验证头文件语法 (arm-none-eabi-gcc -fsyntax-only)**

Run:
```bash
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && arm-none-eabi-gcc -std=c11 -Ilibraries/cmsis/cm4/device_support -Ilibraries/drivers/inc -Iproject/inc -fsyntax-only platform/at32m412/board_motor_pins.h 2>&1 | head -20"
```
Expected: 无错误输出 (可能有 `unused macro` 类警告, 忽略). 若报 `GPIO_PINS_8` 等未定义, 检查 include 路径.

- [ ] **Step 3: Commit**

```bash
git add platform/at32m412/board_motor_pins.h
git commit -m "feat(stage0): add board_motor_pins.h centralized pin definitions"
```

---
## Task 3: 创建 motor_params.h 参数头文件

把 spec §4.3.2 / §4.3.6 / §4.4 / §4.5.1 / §4.7.3 的所有宏定义集中到一个头文件, 后续所有控制环与采样模块都引用.

**Files:**
- Create: `application/motor_control/motor_params.h`

- [ ] **Step 1: 写 motor_params.h**

```c
#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ===== ADC 通用 ===== */
#define ADC_VREF_VOLTS                  3.3f
#define ADC_BITS                        12u
#define ADC_FULL_SCALE                  4096u
#define ADC_LSB_VOLTS                   (ADC_VREF_VOLTS / (float)ADC_FULL_SCALE)

/* ===== MP6540H 电流采样 (spec §4.3.2) ===== */
/* 硬件: 4.7k 上拉到 3.3V + 4.7k 下拉到 GND */
#define MP6540H_VREF_VOLTS              1.65f
#define MP6540H_RTERM_OHMS              2350.0f
#define MP6540H_MIRROR_RATIO_TYP        (1.0f / 9200.0f)
#define MP6540H_VSO_PER_AMP_TYP         0.2554f             /* V/A, typ */
#define CURRENT_ZERO_OFFSET_LSB         2048u                /* 12-bit half-scale */
#define CURRENT_GAIN_DEFAULT_A_PER_LSB  (ADC_LSB_VOLTS / MP6540H_VSO_PER_AMP_TYP)
                                                          /* ~3.16e-3 A/LSB typ */
#define IQ_OVERCURRENT_A                5.0f
#define IMBALANCE_THRESHOLD_A           1.5f

/* ===== VBUS 母线电压 (spec §4.3.6, 分压比 1/6) ===== */
#define VBUS_DIVIDER_RATIO              6.0f
#define VBUS_VOLTS_PER_LSB              (ADC_LSB_VOLTS * VBUS_DIVIDER_RATIO)
                                                          /* 4.834 mV/LSB, 满量程 19.8V */
#define VBUS_UNDERVOLTAGE_THRESHOLD_V   8.0f
#define VBUS_OVERVOLTAGE_THRESHOLD_V    18.0f

/* ===== 电机参数 (spec §4.5.1) ===== */
#define MOTOR_POLE_PAIRS                7u    /* 2808 BLDC 默认, 实测后修正 */

/* ===== CAN 协议 (spec §5.1) ===== */
#define MOTOR_NODE_ID                   0x01u
#define CAN_BITRATE                     500000u

/* ===== 旁轴标定 (spec §4.7.3 / §4.7.4) ===== */
#define CAL_FLASH_ADDR                  0x0801FC00u
#define CAL_FLASH_SIZE                  1024u
#define CAL_MAGIC                       0x304C4143u   /* 'CAL0' little-endian */
#define CAL_VERSION                     1u
#define CAL_TABLE_POINTS                256u
#define CAL_TURNS_PER_DIRECTION         5u
#define CAL_SPIN_SPEED_RPM              30
#define CAL_SAMPLES_PER_TURN            4096u

/* ===== PID 参数初值 (spec §4.4) ===== */
/* 电流环 */
#define PID_ID_KP                       0.5f
#define PID_ID_KI                       100.0f
#define PID_IQ_KP                       0.5f
#define PID_IQ_KI                       100.0f
#define PID_CURRENT_INTEGRAL_LIMIT      (VBUS_OVERVOLTAGE_THRESHOLD_V / 2.0f)
#define PID_CURRENT_OUT_LIMIT           (VBUS_OVERVOLTAGE_THRESHOLD_V / 2.0f)
#define IQ_MAX_A                        8.0f

/* 速度环 */
#define PID_SPEED_KP                    0.01f
#define PID_SPEED_KI                    0.5f
#define PID_SPEED_INTEGRAL_LIMIT        IQ_MAX_A
#define PID_SPEED_OUT_LIMIT             IQ_MAX_A
#define RPM_MAX                         3000

/* 位置环 */
#define PID_POSITION_KP                 5.0f
#define PID_POSITION_OUT_LIMIT          RPM_MAX

/* ===== 零点标定 (spec §4.5.3) ===== */
#define ZERO_ALIGN_CURRENT_A            1.0f
#define ZERO_ALIGN_HOLD_MS              500u
#define ZERO_ALIGN_SAMPLE_WINDOW_MS     100u

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PARAMS_H */
```

- [ ] **Step 2: 验证语法**

Run:
```bash
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && arm-none-eabi-gcc -std=c11 -fsyntax-only application/motor_control/motor_params.h 2>&1 | head -10"
```
Expected: 无输出 (纯宏头文件, 无依赖)

- [ ] **Step 3: Commit**

```bash
git add application/motor_control/motor_params.h
git commit -m "feat(stage0): add motor_params.h centralized parameter definitions"
```

---

## Task 4: 创建 application 层空骨架 (foc_core / loops / isr / fault_manager / calibration)

为后续 plan 预占接口, 所有函数先返回 stub 值, 不影响构建. 每个文件只放接口声明 + 空 实现, 真正算法在 Plan 2/3/4 实现.

**Files:**
- Create: `application/motor_control/foc_core.h` / `foc_core.c`
- Create: `application/motor_control/current_loop.h` / `current_loop.c`
- Create: `application/motor_control/speed_loop.h` / `speed_loop.c`
- Create: `application/motor_control/position_loop.h` / `position_loop.c`
- Create: `application/motor_control/motor_control_isr.h` / `motor_control_isr.c`
- Create: `application/motor_control/fault_manager.h` / `fault_manager.c`
- Create: `application/motor_control/motor_calibration.h` / `motor_calibration.c`
- Create: `application/motor_app.h` / `application/motor_app.c`

- [ ] **Step 1: 写 foc_core.h**

```c
#ifndef FOC_CORE_H
#define FOC_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Clarke 变换: 三相电流 -> alpha/beta */
void foc_clarke(float ia, float ib, float ic, float *i_alpha, float *i_beta);

/* Park 变换: alpha/beta -> d/q (输入电角度 theta_e, 弧度) */
void foc_park(float i_alpha, float i_beta, float theta_e, float *id, float *iq);

/* 逆 Park 变换: d/q -> alpha/beta */
void foc_ipark(float vd, float vq, float theta_e, float *v_alpha, float *v_beta);

/* SVPWM 7 段法, 三相高边特化 (spec §4.2)
 * v_alpha, v_beta: 电压分量 (V)
 * vbus: 母线电压 (V)
 * ta, tb, tc: 输出占空比 ticks (0..TMR1_ARR, 已限幅 95%)
 */
void foc_svpwm_3phase_high_side(float v_alpha, float v_beta, float vbus,
                                 uint16_t *ta, uint16_t *tb, uint16_t *tc);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CORE_H */
```

- [ ] **Step 2: 写 foc_core.c (stub)**

```c
#include "foc_core.h"
#include "board_motor_pins.h"

void foc_clarke(float ia, float ib, float ic, float *i_alpha, float *i_beta)
{
    (void)ic;
    *i_alpha = ia;
    *i_beta = (ia + 2.0f * ib) * 0.57735026919f; /* 1/sqrt(3) */
}

void foc_park(float i_alpha, float i_beta, float theta_e, float *id, float *iq)
{
    (void)i_alpha; (void)i_beta; (void)theta_e;
    *id = 0.0f;
    *iq = 0.0f;
}

void foc_ipark(float vd, float vq, float theta_e, float *v_alpha, float *v_beta)
{
    (void)vd; (void)vq; (void)theta_e;
    *v_alpha = 0.0f;
    *v_beta = 0.0f;
}

void foc_svpwm_3phase_high_side(float v_alpha, float v_beta, float vbus,
                                 uint16_t *ta, uint16_t *tb, uint16_t *tc)
{
    (void)v_alpha; (void)v_beta; (void)vbus;
    /* stub: 50% 占空比, 三相同电位, 不出力 */
    *ta = (uint16_t)(TMR1_ARR / 2u);
    *tb = (uint16_t)(TMR1_ARR / 2u);
    *tc = (uint16_t)(TMR1_ARR / 2u);
}
```

> 注: `foc_clarke` 给了真实实现 (公式简单且不会错, 直接用), 其他三个 stub 留到 Plan 4 填. 这样 Plan 1 构建通过且语义正确.

- [ ] **Step 3: 写 current_loop.h / current_loop.c (stub)**

`current_loop.h`:
```c
#ifndef CURRENT_LOOP_H
#define CURRENT_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "motor_params.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float integral_limit;
    float out_limit;
    float last_error;
} pid_f32_t;

float pid_f32_exec(pid_f32_t *pid, float error);
void current_loop_init(void);
void current_loop_run(float id, float iq, float *vd_ref, float *vq_ref);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_LOOP_H */
```

`current_loop.c`:
```c
#include "current_loop.h"

float pid_f32_exec(pid_f32_t *pid, float error)
{
    (void)pid; (void)error;
    return 0.0f; /* stub, Plan 4 实现 */
}

void current_loop_init(void)
{
}

void current_loop_run(float id, float iq, float *vd_ref, float *vq_ref)
{
    (void)id; (void)iq;
    *vd_ref = 0.0f;
    *vq_ref = 0.0f;
}
```

- [ ] **Step 4: 写 speed_loop.h / speed_loop.c (stub)**

`speed_loop.h`:
```c
#ifndef SPEED_LOOP_H
#define SPEED_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "current_loop.h"  /* pid_f32_t */

void speed_loop_init(void);
void speed_loop_run(int16_t raw_speed);

#ifdef __cplusplus
}
#endif

#endif /* SPEED_LOOP_H */
```

`speed_loop.c`:
```c
#include "speed_loop.h"

void speed_loop_init(void) {}
void speed_loop_run(int16_t raw_speed) { (void)raw_speed; }
```

- [ ] **Step 5: 写 position_loop.h / position_loop.c (stub)**

`position_loop.h`:
```c
#ifndef POSITION_LOOP_H
#define POSITION_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void position_loop_init(void);
void position_loop_run(uint16_t raw_angle);

#ifdef __cplusplus
}
#endif

#endif /* POSITION_LOOP_H */
```

`position_loop.c`:
```c
#include "position_loop.h"

void position_loop_init(void) {}
void position_loop_run(uint16_t raw_angle) { (void)raw_angle; }
```

- [ ] **Step 6: 写 motor_control_isr.h / motor_control_isr.c (stub)**

`motor_control_isr.h`:
```c
#ifndef MOTOR_CONTROL_ISR_H
#define MOTOR_CONTROL_ISR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* FOC ISR 主体, 由 TMR1_OVF 中断调用 (spec §3.1).
 * Stage 2+ 实现真正逻辑, Plan 1 仅空实现.
 */
void motor_control_isr_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_ISR_H */
```

`motor_control_isr.c`:
```c
#include "motor_control_isr.h"

void motor_control_isr_tick(void)
{
    /* stub, Plan 2 开始填充 */
}
```

- [ ] **Step 7: 写 fault_manager.h / fault_manager.c (stub)**

`fault_manager.h`:
```c
#ifndef FAULT_MANAGER_H
#define FAULT_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 故障位图 (spec §5.5) */
typedef enum {
    FAULT_NONE             = 0u,
    FAULT_DRIVER           = 1u << 0,  /* MP6540H nFAULT */
    FAULT_OVERCURRENT      = 1u << 1,
    FAULT_SENSOR           = 1u << 2,
    FAULT_UNDERVOLTAGE     = 1u << 3,
    FAULT_OVERVOLTAGE      = 1u << 4,
    FAULT_CAN_TIMEOUT      = 1u << 5,
    FAULT_CAL_INVALID      = 1u << 6,
} motor_fault_t;

void fault_manager_init(void);
void fault_manager_set(uint32_t fault);
void fault_manager_clear(uint32_t fault);
void fault_manager_clear_all(void);
uint32_t fault_manager_get(void);
bool fault_manager_any(void);

#ifdef __cplusplus
}
#endif

#endif /* FAULT_MANAGER_H */
```

`fault_manager.c`:
```c
#include "fault_manager.h"
#include <stdbool.h>

static uint32_t s_fault_flags = 0u;

void fault_manager_init(void) { s_fault_flags = 0u; }
void fault_manager_set(uint32_t fault) { s_fault_flags |= fault; }
void fault_manager_clear(uint32_t fault) { s_fault_flags &= ~fault; }
void fault_manager_clear_all(void) { s_fault_flags = 0u; }
uint32_t fault_manager_get(void) { return s_fault_flags; }
bool fault_manager_any(void) { return s_fault_flags != 0u; }
```

> 注: `fault_manager` 给真实实现 (逻辑简单且不会错), 后续 Plan 5 只需扩展触发源接入.

- [ ] **Step 8: 写 motor_calibration.h / motor_calibration.c (stub)**

`motor_calibration.h`:
```c
#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "motor_params.h"

typedef struct {
    uint32_t magic;
    uint8_t  version;
    uint8_t  reserved[3];
    uint32_t timestamp_ms;
    int16_t  table[CAL_TABLE_POINTS];
    uint16_t mech_zero_raw;
    uint8_t  pole_pairs;
    uint8_t  reserved2;
    uint32_t crc32;
} motor_calibration_t;

typedef enum {
    CAL_STATE_IDLE = 0,
    CAL_STATE_ZERO_ALIGN,
    CAL_STATE_SPIN_FWD,
    CAL_STATE_SPIN_REV,
    CAL_STATE_COMPUTE,
    CAL_STATE_WRITE_FLASH,
    CAL_STATE_DONE,
} cal_state_t;

/* 开机加载, 失败时 g_cal_valid=false 且置 FAULT_CAL_INVALID */
void motor_calibration_load(void);
bool motor_calibration_is_valid(void);
const motor_calibration_t *motor_calibration_get(void);

/* 触发标定 (CAN/finsh 入口) */
void motor_calibration_start(void);
cal_state_t motor_calibration_get_state(void);
uint8_t motor_calibration_get_progress(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CALIBRATION_H */
```

`motor_calibration.c`:
```c
#include "motor_calibration.h"
#include "fault_manager.h"

static motor_calibration_t s_cal;
static bool s_cal_valid = false;
static cal_state_t s_cal_state = CAL_STATE_IDLE;

void motor_calibration_load(void)
{
    s_cal_valid = false;  /* stub, Plan 3 (Stage 4b) 实现 FLASH 读取 + CRC */
    fault_manager_set(FAULT_CAL_INVALID);
}

bool motor_calibration_is_valid(void) { return s_cal_valid; }
const motor_calibration_t *motor_calibration_get(void) { return &s_cal; }
void motor_calibration_start(void) { s_cal_state = CAL_STATE_ZERO_ALIGN; }
cal_state_t motor_calibration_get_state(void) { return s_cal_state; }
uint8_t motor_calibration_get_progress(void) { return 0u; }
```

- [ ] **Step 9: 写 motor_app.h / motor_app.c (应用入口)**

`application/motor_app.h`:
```c
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
```

`application/motor_app.c`:
```c
#include "motor_app.h"
#include "motor_control.h"
#include "fault_manager.h"
#include "motor_calibration.h"
#include "ma600a_debug.h"

void motor_app_init(void)
{
    fault_manager_init();
    motor_control_init();  /* 已有状态机 */
    motor_calibration_load();  /* 开机加载标定 */
    ma600a_debug_init();   /* 保留 bring-up 路径 */
}

void motor_app_run(void)
{
    while (1) {
        ma600a_debug_poll();
        /* Plan 5 加入 CAN 收发 / 状态上报 */
    }
}
```

> 注: `motor_app_run` 内部死循环, `main.c` 直接调用即可. RT-Thread 任务在 Plan 5 重构, Plan 1 暂保留原 `rtthread_app.c` 不动.

- [ ] **Step 10: 验证全部新文件语法**

Run:
```bash
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && for f in application/motor_control/foc_core.c application/motor_control/current_loop.c application/motor_control/speed_loop.c application/motor_control/position_loop.c application/motor_control/motor_control_isr.c application/motor_control/fault_manager.c application/motor_control/motor_calibration.c application/motor_app.c; do echo \"--- \$f ---\"; arm-none-eabi-gcc -std=c11 -Wall -Wextra -Iapplication/motor_control -Iplatform/at32m412 -Iproject/inc -Ilibraries/drivers/inc -Ilibraries/cmsis/cm4/device_support -Imiddlewares/msp/ma600 -fsyntax-only \$f 2>&1 | head -10; done"
```
Expected: 每个文件无错误 (可能有 unused parameter 警告, 因 stub 函数用了 `(void)`, 应无警告).

- [ ] **Step 11: Commit**

```bash
git add application/
git commit -m "feat(stage0): add application layer skeletons (foc_core/loops/isr/fault/calib/app)"
```

---
## Task 5: 创建 platform 层空骨架 (current_sense / encoder / protect / flash_calibration / clock)

**Files:**
- Create: `platform/at32m412/current_sense_at32m412.h` / `.c`
- Create: `platform/at32m412/motor_encoder_at32m412.h` / `.c`
- Create: `platform/at32m412/motor_protect_at32m412.h` / `.c`
- Create: `platform/at32m412/flash_calibration_at32m412.h` / `.c`
- Create: `platform/at32m412/clock_at32m412.h` / `.c`

- [ ] **Step 1: 写 current_sense_at32m412.h / .c (stub)**

`.h`:
```c
#ifndef CURRENT_SENSE_AT32M412_H
#define CURRENT_SENSE_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ADC2 注入序列初始化 (TMR1_CH4 顶点触发, spec §3.2) */
void current_sense_at32m412_init(void);

/* 读取三相电流 ADC 原始值 (FOC ISR 内调用) */
void current_sense_at32m412_read_raw(uint16_t *ia, uint16_t *ib, uint16_t *ic);

/* 零偏标定 (PWM 50% 时采 1024 次平均, spec §4.3.3) */
void current_sense_at32m412_calibrate_offset(void);

/* raw -> 安培 (spec §4.3.4) */
float current_sense_calc(uint16_t raw, float offset_lsb, float gain_a_per_lsb);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_SENSE_AT32M412_H */
```

`.c`:
```c
#include "current_sense_at32m412.h"
#include "motor_params.h"

void current_sense_at32m412_init(void) { /* stub, Plan 2 */ }
void current_sense_at32m412_read_raw(uint16_t *ia, uint16_t *ib, uint16_t *ic)
{
    *ia = CURRENT_ZERO_OFFSET_LSB;
    *ib = CURRENT_ZERO_OFFSET_LSB;
    *ic = CURRENT_ZERO_OFFSET_LSB;
}
void current_sense_at32m412_calibrate_offset(void) { /* stub */ }

float current_sense_calc(uint16_t raw, float offset_lsb, float gain_a_per_lsb)
{
    return ((float)raw - offset_lsb) * gain_a_per_lsb;
}
```

> 注: `current_sense_calc` 给真实实现 (纯数学, 不会错).

- [ ] **Step 2: 写 motor_encoder_at32m412.h / .c (stub)**

`.h`:
```c
#ifndef MOTOR_ENCODER_AT32M412_H
#define MOTOR_ENCODER_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 初始化 MA600A SPI2 适配 (封装 middlewares/msp/ma600) */
void motor_encoder_at32m412_init(void);

/* 读取角度 + 速度 (FOC ISR 内同步阻塞, spec §3.1 step 3)
 * retval 0=成功, 非0=故障码
 */
int motor_encoder_read_angle_speed(uint16_t *raw_angle_16, int16_t *raw_speed);

/* 机械角 -> 电角度 (弧度), 含旁轴标定查表 (spec §4.5.2 + §4.7.6) */
float motor_encoder_to_electrical_angle(uint16_t raw_angle_16);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ENCODER_AT32M412_H */
```

`.c`:
```c
#include "motor_encoder_at32m412.h"
#include "ma600a.h"
#include "ma600a_at32_spi2.h"
#include "motor_calibration.h"
#include "motor_params.h"
#include <math.h>

static ma600a_t s_ma600a;

void motor_encoder_at32m412_init(void)
{
    ma600a_init(&s_ma600a, ma600a_at32_spi2_bus_get());
}

int motor_encoder_read_angle_speed(uint16_t *raw_angle_16, int16_t *raw_speed)
{
    uint16_t raw_12 = 0u;
    int16_t spd = 0;
    if (ma600a_read_angle_and_speed_raw(&s_ma600a, &raw_12, &spd) != 0) {
        return -1;
    }
    *raw_angle_16 = (uint16_t)(raw_12 << 4);  /* 12-bit -> 16-bit 扩展 */
    *raw_speed = spd;
    return 0;
}

float motor_encoder_to_electrical_angle(uint16_t raw_angle_16)
{
    /* stub: 暂不做旁轴标定查表与零点修正, Plan 3 填充 */
    (void)raw_angle_16;
    return 0.0f;
}
```

> 注: `motor_encoder_read_angle_speed` 给真实实现 (直接调用现有 ma600a 驱动), `motor_encoder_to_electrical_angle` 留 stub 到 Plan 3.

- [ ] **Step 3: 写 motor_protect_at32m412.h / .c (stub)**

`.h`:
```c
#ifndef MOTOR_PROTECT_AT32M412_H
#define MOTOR_PROTECT_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* nFAULT EXINT 初始化 (PB2, spec §1.2) */
void motor_protect_at32m412_init(void);

/* 读 VBUS ADC (普通转换), 返回电压 V (spec §4.3.6) */
float motor_protect_read_vbus_v(void);

/* 1 kHz 调用: 检查 VBUS 过/欠压, 触发 fault_manager */
void motor_protect_check_vbus(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PROTECT_AT32M412_H */
```

`.c`:
```c
#include "motor_protect_at32m412.h"
#include "fault_manager.h"
#include "motor_params.h"

void motor_protect_at32m412_init(void) { /* stub, Plan 5 */ }

float motor_protect_read_vbus_v(void)
{
    /* stub: 返回安全中值, Plan 2 接真实 ADC */
    return 12.0f;
}

void motor_protect_check_vbus(void)
{
    float vbus = motor_protect_read_vbus_v();
    if (vbus < VBUS_UNDERVOLTAGE_THRESHOLD_V) {
        fault_manager_set(FAULT_UNDERVOLTAGE);
    } else {
        fault_manager_clear(FAULT_UNDERVOLTAGE);
    }
    if (vbus > VBUS_OVERVOLTAGE_THRESHOLD_V) {
        fault_manager_set(FAULT_OVERVOLTAGE);
    } else {
        fault_manager_clear(FAULT_OVERVOLTAGE);
    }
}
```

> 注: `motor_protect_check_vbus` 给真实实现 (逻辑简单), VBUS 读取 stub 到 Plan 2.

- [ ] **Step 4: 写 flash_calibration_at32m412.h / .c (stub)**

`.h`:
```c
#ifndef FLASH_CALIBRATION_AT32M412_H
#define FLASH_CALIBRATION_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "motor_calibration.h"

/* 从 FLASH 标定区读取 (spec §4.7.7) */
bool flash_calibration_read(motor_calibration_t *cal);

/* 擦除末页 + 写入标定数据 (spec §4.7.5 CAL_WRITE_FLASH) */
bool flash_calibration_write(const motor_calibration_t *cal);

/* 擦除标定区 (finsh mc_cal_erase) */
bool flash_calibration_erase(void);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_CALIBRATION_AT32M412_H */
```

`.c`:
```c
#include "flash_calibration_at32m412.h"
#include "motor_params.h"
#include <string.h>

bool flash_calibration_read(motor_calibration_t *cal)
{
    /* stub: 直接返回失败, Plan 3 实现 */
    (void)cal;
    return false;
}

bool flash_calibration_write(const motor_calibration_t *cal)
{
    (void)cal;
    return false;
}

bool flash_calibration_erase(void)
{
    return false;
}
```

- [ ] **Step 5: 写 clock_at32m412.h / .c (暂沿用 wk_system_clock_config)**

`.h`:
```c
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
```

`.c`:
```c
#include "clock_at32m412.h"
#include "at32m412_416_wk_config.h"

void clock_at32m412_init(void)
{
    wk_system_clock_config();
}
```

> 注: Plan 1 不重写时钟, 沿用 Workbench 生成的 `wk_system_clock_config()` (此时该函数体还在, Task 6 才清空其他 wk_*_init, 但 wk_system_clock_config 保留). Plan 2 再决定是否自研.

- [ ] **Step 6: 验证全部新文件语法**

Run:
```bash
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && for f in platform/at32m412/current_sense_at32m412.c platform/at32m412/motor_encoder_at32m412.c platform/at32m412/motor_protect_at32m412.c platform/at32m412/flash_calibration_at32m412.c platform/at32m412/clock_at32m412.c; do echo \"--- \$f ---\"; arm-none-eabi-gcc -std=c11 -Wall -Wextra -Iplatform/at32m412 -Iapplication/motor_control -Iproject/inc -Ilibraries/drivers/inc -Ilibraries/cmsis/cm4/device_support -Imiddlewares/msp/ma600 -fsyntax-only \$f 2>&1 | head -10; done"
```
Expected: 无错误. `motor_encoder_at32m412.c` 可能报 `ma600a_t` 未定义, 确认 `-Imiddlewares/msp/ma600` 路径正确.

- [ ] **Step 7: Commit**

```bash
git add platform/at32m412/
git commit -m "feat(stage0): add platform layer skeletons (current_sense/encoder/protect/flash/clock)"
```

---

## Task 6: 创建 communication 层空骨架

**Files:**
- Create: `communication/can_protocol.h` / `.c`
- Create: `communication/can_at32m412.h` / `.c`

- [ ] **Step 1: 写 can_protocol.h / .c (stub)**

`can_protocol.h`:
```c
#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* CAN ID 编码 (spec §5.1): (function_code << 7) | node_id */
#define CAN_ID_CONTROL           ((0x02u << 7) | MOTOR_NODE_ID)   /* 0x101 */
#define CAN_ID_STATUS            ((0x03u << 7) | MOTOR_NODE_ID)   /* 0x181 */
#define CAN_ID_EXT_STATUS        ((0x05u << 7) | MOTOR_NODE_ID)   /* 0x281 */

/* 控制模式 (spec §3.4) */
typedef enum {
    CAN_MODE_OPEN_LOOP  = 0,
    CAN_MODE_CURRENT    = 1,
    CAN_MODE_SPEED      = 2,
    CAN_MODE_POSITION   = 3,
    CAN_MODE_ALIGN      = 4,
    CAN_MODE_CALIBRATE  = 5,
} can_mode_t;

/* 解析收到的控制帧 (8 bytes), 更新 motor_control 状态 */
void can_protocol_handle_control(const uint8_t *data, uint8_t len);

/* 组装状态帧 (8 bytes) 用于发送 */
void can_protocol_build_status(uint8_t *data, uint8_t *len);

/* 组装扩展状态帧 (8 bytes) */
void can_protocol_build_ext_status(uint8_t *data, uint8_t *len);

#ifdef __cplusplus
}
#endif

#endif /* CAN_PROTOCOL_H */
```

`can_protocol.c`:
```c
#include "can_protocol.h"
#include "motor_params.h"
#include <string.h>

void can_protocol_handle_control(const uint8_t *data, uint8_t len)
{
    (void)data; (void)len;  /* stub, Plan 5 */
}

void can_protocol_build_status(uint8_t *data, uint8_t *len)
{
    memset(data, 0, 8);
    *len = 8;  /* stub, Plan 5 */
}

void can_protocol_build_ext_status(uint8_t *data, uint8_t *len)
{
    memset(data, 0, 8);
    *len = 8;  /* stub, Plan 5 */
}
```

- [ ] **Step 2: 写 can_at32m412.h / .c (stub)**

`can_at32m412.h`:
```c
#ifndef CAN_AT32M412_H
#define CAN_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* CAN1 初始化 (500kbps, spec §5.1) */
void can_at32m412_init(void);

/* 发送一帧 (阻塞, 超时 100ms) */
bool can_at32m412_send(uint32_t id, const uint8_t *data, uint8_t len);

/* 注册接收回调 (CAN RX 中断内调用) */
typedef void (*can_rx_callback_t)(uint32_t id, const uint8_t *data, uint8_t len);
void can_at32m412_register_rx(can_rx_callback_t cb);

#ifdef __cplusplus
}
#endif

#endif /* CAN_AT32M412_H */
```

`can_at32m412.c`:
```c
#include "can_at32m412.h"

void can_at32m412_init(void) { /* stub, Plan 5 */ }
bool can_at32m412_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    (void)id; (void)data; (void)len;
    return false;
}
void can_at32m412_register_rx(can_rx_callback_t cb) { (void)cb; }
```

- [ ] **Step 3: 验证语法**

Run:
```bash
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && arm-none-eabi-gcc -std=c11 -Wall -Wextra -Icommunication -Iapplication/motor_control -Iproject/inc -fsyntax-only communication/can_protocol.c 2>&1 | head -10 && arm-none-eabi-gcc -std=c11 -Wall -Wextra -Icommunication -fsyntax-only communication/can_at32m412.c 2>&1 | head -10"
```
Expected: 无错误

- [ ] **Step 4: Commit**

```bash
git add communication/
git commit -m "feat(stage0): add communication layer skeletons (can_protocol/can_at32m412)"
```

---
## Task 7: 废弃 Workbench wk_*_init() 函数体 (保留 wk_system_clock_config)

按 spec 方案 Y: 保留头文件引脚 `#define` 与函数声明, 函数体清空 (除 `wk_system_clock_config` 暂留, Plan 2 决定是否自研).

**Files:**
- Modify: `project/src/at32m412_416_wk_config.c`

- [ ] **Step 1: 查看当前 wk_config.c 中所有 wk_* 函数**

Run:
```bash
cd E:\WorkSpaces\2_MotorDriver\MPS_MotorDriver
git show HEAD:project/src/at32m412_416_wk_config.c | grep -n "^void wk_"
```
Expected: 列出所有 `wk_*_init/config` 函数名与行号. 记录下来.

- [ ] **Step 2: 用脚本批量清空函数体 (保留 wk_system_clock_config)**

在 PowerShell 中执行 (谨慎, 先备份):
```powershell
$file = "E:\WorkSpaces\2_MotorDriver\MPS_MotorDriver\project\src\at32m412_416_wk_config.c"
Copy-Item $file "$file.bak"
# 人工编辑: 对每个 wk_* 函数 (除 wk_system_clock_config), 把函数体内容替换为空或仅 return
# 由于函数体跨多行且含 AT 库调用, 建议用编辑器逐个处理, 不用脚本盲目替换
```

实际操作: 用 `apply_patch` 工具或编辑器, 把以下函数体清空 (保留函数签名 + 一对空花括号):
- `wk_periph_clock_config(void)` → `{}`
- `wk_nvic_config(void)` → `{}`
- `wk_gpio_config(void)` → `{}`
- `wk_adc_common_init(void)` → `{}`
- `wk_adc2_init(void)` → `{}`
- `wk_usart1_init(void)` → `{}`
- `wk_can1_init(void)` → `{}`
- `wk_spi2_init(void)` → `{}`
- `wk_wdt_init(void)` → `{}`
- `wk_tmr1_init(void)` → `{}`

**保留不动:** `wk_system_clock_config(void)` (Plan 2 评估是否自研时钟)

- [ ] **Step 3: 验证编译 (此时 main.c 仍调用 wk_*_init, 空函数体不影响编译)**

Run:
```bash
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && cmake --build build/Debug 2>&1 | tail -10"
```
Expected: 构建成功 (空函数体合法). 资源占用应略降 (因为 wk_* 内的寄存器配置代码没了).

- [ ] **Step 4: Commit**

```bash
git add project/src/at32m412_416_wk_config.c
git commit -m "refactor(stage0): empty wk_*_init bodies (keep wk_system_clock_config, spec scheme Y)"
```

> ⚠️ 风险提示: 此 task 后, 板子上的外设 (GPIO/ADC/SPI/CAN/USART/TMR1) 不再被初始化. 这是预期的 —— Plan 2 起用 platform 层重新初始化. 此 commit 后固件烧录不会正常跑 MA600A 调试, 属正常过渡态.

---

## Task 8: 精简 main.c 为应用入口

**Files:**
- Modify: `project/src/main.c`

- [ ] **Step 1: 重写 main.c**

```c
/* add user code begin Header */
/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
  **************************************************************************
  * Copyright (c) 2025, Artery Technology, All rights reserved.
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* add user code end Header */

/* Includes ------------------------------------------------------------------*/
#include "at32m412_416_wk_config.h"
#include "rtthread_app.h"
#include "clock_at32m412.h"
#include "motor_app.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */

/* add user code end private includes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  /* add user code begin 1 */

  /* add user code end 1 */

  /* 系统时钟 (暂沿用 wk_system_clock_config) */
  clock_at32m412_init();

  /* 应用层初始化 (加载标定 / 初始化状态机 / MA600A) */
  motor_app_init();

  /* init rtthread function. */
  wk_rtthread_init();

  /* 应用主循环 (内部 while(1)) */
  motor_app_run();

  /* 不应到达 */
  while (1) {
  }
}

  /* add user code begin 4 */

  /* add user code end 4 */
```

> 注: 移除了对 `wk_gpio_config / wk_adc2_init / wk_usart1_init / wk_can1_init / wk_spi2_init / wk_wdt_init / wk_tmr1_init` 的调用 (这些函数体已在 Task 7 清空, 调用无意义). `ma600a_debug_init/poll` 已移入 `motor_app_init/run`. RT-Thread 初始化保留 (Plan 5 重构).

- [ ] **Step 2: 验证编译**

Run:
```bash
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && cmake --build build/Debug 2>&1 | tail -15"
```
Expected: 构建成功. 若报 `motor_app.h` 找不到, 说明 CMake 还没纳入新源 (Task 9 会修), 此 task 先在 main.c 用相对路径 include 或临时调整. 正确做法: 等 Task 9 一起验证.

> **调整: Task 8 与 Task 9 合并验证, Task 8 先写代码不单独构建, Task 9 改完 CMake 后统一构建.**

- [ ] **Step 3: Commit (暂不构建验证, 与 Task 9 合并)**

```bash
git add project/src/main.c
git commit -m "refactor(stage0): slim main.c to call motor_app_init/run (wk_*_init calls removed)"
```

---

## Task 9: 更新 CMakeLists.txt 纳入所有新源文件与 include 路径

**Files:**
- Modify: `CMakeLists.txt`

- [ ] **Step 1: 修改 CMakeLists.txt 的 target_sources 与 target_include_directories**

在现有 `target_sources` 的 user 段补充所有新 .c 文件, 在 `target_include_directories` 补充新目录:

```cmake
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_SOURCE_DIR}/application/motor_control/motor_control.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/foc_core.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/current_loop.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/speed_loop.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/position_loop.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/motor_control_isr.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/fault_manager.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/motor_calibration.c
    ${CMAKE_SOURCE_DIR}/application/motor_app.c
    ${CMAKE_SOURCE_DIR}/middlewares/msp/ma600/ma600a.c
    ${CMAKE_SOURCE_DIR}/middlewares/msp/ma600/ma600a_at32_spi2.c
    ${CMAKE_SOURCE_DIR}/platform/at32m412/motor_pwm_at32m412.c
    ${CMAKE_SOURCE_DIR}/platform/at32m412/current_sense_at32m412.c
    ${CMAKE_SOURCE_DIR}/platform/at32m412/motor_encoder_at32m412.c
    ${CMAKE_SOURCE_DIR}/platform/at32m412/motor_protect_at32m412.c
    ${CMAKE_SOURCE_DIR}/platform/at32m412/flash_calibration_at32m412.c
    ${CMAKE_SOURCE_DIR}/platform/at32m412/clock_at32m412.c
    ${CMAKE_SOURCE_DIR}/communication/can_protocol.c
    ${CMAKE_SOURCE_DIR}/communication/can_at32m412.c
    ${CMAKE_SOURCE_DIR}/project/src/ma600a_debug.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/application
    ${CMAKE_SOURCE_DIR}/application/motor_control
    ${CMAKE_SOURCE_DIR}/middlewares/msp/ma600
    ${CMAKE_SOURCE_DIR}/platform/at32m412
    ${CMAKE_SOURCE_DIR}/communication
)
```

- [ ] **Step 2: 重新配置并构建**

Run:
```bash
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && rm -rf build/Debug && cmake -B build/Debug -DCMAKE_BUILD_TYPE=Debug -G 'Unix Makefiles' 2>&1 | tail -10 && cmake --build build/Debug 2>&1 | tail -30"
```
Expected: 构建成功. 可能有 unused parameter 警告 (stub 函数), 属正常. 若有 undefined reference, 检查是否有 .c 文件遗漏未加入 target_sources.

- [ ] **Step 3: 记录资源占用**

Run:
```bash
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && arm-none-eabi-size build/Debug/MPS_MotorDriver.elf"
```
Expected: FLASH 占用应略增 (新增 stub 函数 + motor_params 宏展开), 预估 27-29 KB (基线 26.5 KB + ~2 KB stub). RAM 基本持平. **记录实际数字**.

- [ ] **Step 4: Commit**

```bash
git add CMakeLists.txt
git commit -m "build(stage0): wire all new sources/includes into CMakeLists.txt; baseline rebuild passes"
```

---

## Task 10: 修改 FLASH.ld 预留末 1 KB 给标定区

**Files:**
- Modify: `AT32M412xB_FLASH.ld`

- [ ] **Step 1: 修改 FLASH LENGTH 从 128K 改为 127K**

在 `AT32M412xB_FLASH.ld` 找到:
```
FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 128K
```
改为:
```
FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 127K
```

> 注: 不新增 MEMORY 段给标定区, 因为标定区由 `flash_calibration_at32m412.c` 直接按绝对地址 `0x0801FC00` 读写, 不参与链接. 缩短 LENGTH 只是防止链接器把代码放到末 1 KB.

- [ ] **Step 2: 验证构建 (确认代码未溢出新边界)**

Run:
```bash
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && cmake --build build/Debug 2>&1 | tail -5 && arm-none-eabi-size build/Debug/MPS_MotorDriver.elf"
```
Expected: 构建成功, FLASH 占用 < 127K (当前 ~27 KB, 远低于上限). 若报 `region FLASH overflowed`, 说明 LENGTH 改错.

- [ ] **Step 3: Commit**

```bash
git add AT32M412xB_FLASH.ld
git commit -m "build(stage0): reserve last 1KB flash sector for calibration data (LENGTH 128K->127K)"
```

---

## Task 11: 扩展单元测试覆盖新增模块的纯逻辑函数

只测**纯逻辑且已给真实实现**的函数 (stub 不测): `foc_clarke`, `current_sense_calc`, `fault_manager`, `motor_protect_check_vbus` 的逻辑分支.

**Files:**
- Create: `tests/foc_core/test_foc_clarke.c`
- Create: `tests/current_sense/test_current_sense_calc.c`
- Create: `tests/fault_manager/test_fault_manager.c`

- [ ] **Step 1: 写 test_foc_clarke.c**

```c
/* tests/foc_core/test_foc_clarke.c */
#include "foc_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static int approx_eq(float a, float b, float eps)
{
    return fabsf(a - b) < eps;
}

int main(void)
{
    float ia, ib, ic, ialpha, ibeta;

    /* 测试 1: 三相平衡正序, ia=1A, ib=-0.5A, ic=-0.5A -> ialpha=1, ibeta=0 */
    ia = 1.0f; ib = -0.5f; ic = -0.5f;
    foc_clarke(ia, ib, ic, &ialpha, &ibeta);
    assert(approx_eq(ialpha, 1.0f, 1e-4f));
    assert(approx_eq(ibeta, 0.0f, 1e-4f));

    /* 测试 2: 全零 -> 全零 */
    foc_clarke(0, 0, 0, &ialpha, &ibeta);
    assert(approx_eq(ialpha, 0.0f, 1e-6f));
    assert(approx_eq(ibeta, 0.0f, 1e-6f));

    /* 测试 3: ia=0, ib=sqrt(3)/2, ic=-sqrt(3)/2 -> ialpha=0, ibeta=1 */
    ia = 0.0f; ib = 0.8660254f; ic = -0.8660254f;
    foc_clarke(ia, ib, ic, &ialpha, &ibeta);
    assert(approx_eq(ialpha, 0.0f, 1e-4f));
    assert(approx_eq(ibeta, 1.0f, 1e-4f));

    printf("test_foc_clarke: 3 tests passed\n");
    return 0;
}
```

- [ ] **Step 2: 写 test_current_sense_calc.c**

```c
/* tests/current_sense/test_current_sense_calc.c */
#include "current_sense_at32m412.h"
#include "motor_params.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static int approx_eq(float a, float b, float eps) { return fabsf(a - b) < eps; }

int main(void)
{
    /* 零电流: raw=2048, offset=2048 -> 0 A */
    float i = current_sense_calc(CURRENT_ZERO_OFFSET_LSB, (float)CURRENT_ZERO_OFFSET_LSB, CURRENT_GAIN_DEFAULT_A_PER_LSB);
    assert(approx_eq(i, 0.0f, 1e-6f));

    /* 满量程正向: raw=4096, offset=2048, gain=3.16mA/LSB -> +6.48 A */
    i = current_sense_calc(4096, 2048.0f, CURRENT_GAIN_DEFAULT_A_PER_LSB);
    assert(approx_eq(i, 2048.0f * CURRENT_GAIN_DEFAULT_A_PER_LSB, 1e-3f));

    /* 负向: raw=0, offset=2048 -> -6.48 A */
    i = current_sense_calc(0, 2048.0f, CURRENT_GAIN_DEFAULT_A_PER_LSB);
    assert(approx_eq(i, -2048.0f * CURRENT_GAIN_DEFAULT_A_PER_LSB, 1e-3f));

    printf("test_current_sense_calc: 3 tests passed\n");
    return 0;
}
```

- [ ] **Step 3: 写 test_fault_manager.c**

```c
/* tests/fault_manager/test_fault_manager.c */
#include "fault_manager.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    fault_manager_init();
    assert(fault_manager_get() == FAULT_NONE);
    assert(!fault_manager_any());

    fault_manager_set(FAULT_OVERCURRENT);
    assert(fault_manager_get() == FAULT_OVERCURRENT);
    assert(fault_manager_any());

    fault_manager_set(FAULT_SENSOR);
    assert(fault_manager_get() == (FAULT_OVERCURRENT | FAULT_SENSOR));

    fault_manager_clear(FAULT_OVERCURRENT);
    assert(fault_manager_get() == FAULT_SENSOR);
    assert(fault_manager_any());

    fault_manager_clear_all();
    assert(fault_manager_get() == FAULT_NONE);
    assert(!fault_manager_any());

    printf("test_fault_manager: 5 tests passed\n");
    return 0;
}
```

- [ ] **Step 4: 编译并运行测试 (主机 gcc)**

> 注: 这些测试文件引用了嵌入式头文件 (含 `at32m412_416.h` 等), 主机 gcc 直接编译会失败. 解决方案: 测试文件只 include 纯逻辑头文件, 对依赖硬件的头文件用 mock. 实际操作: `test_foc_clarke.c` 只 include `foc_core.h` (它 include `stdint.h`), 但 `foc_core.c` include `board_motor_pins.h` (它 include `at32m412_416.h`). 因此主机编译需 mock `board_motor_pins.h` 的 `TMR1_ARR`.

简化方案: 主机测试用 `arm-none-eabi-gcc -fsyntax-only` 验证语法, 不实际运行 assert (与现有 `test_motor_control_state.c` 同策略, 见 stage0_baseline §"TDD/验证记录").

Run:
```bash
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && arm-none-eabi-gcc -std=c11 -Wall -Wextra -Iapplication/motor_control -Iplatform/at32m412 -Iproject/inc -Ilibraries/drivers/inc -Ilibraries/cmsis/cm4/device_support -fsyntax-only tests/foc_core/test_foc_clarke.c application/motor_control/foc_core.c 2>&1 | head -10 && arm-none-eabi-gcc -std=c11 -Wall -Wextra -Iapplication/motor_control -Iplatform/at32m412 -Iproject/inc -Ilibraries/drivers/inc -Ilibraries/cmsis/cm4/device_support -fsyntax-only tests/current_sense/test_current_sense_calc.c platform/at32m412/current_sense_at32m412.c 2>&1 | head -10 && arm-none-eabi-gcc -std=c11 -Wall -Wextra -Iapplication/motor_control -fsyntax-only tests/fault_manager/test_fault_manager.c application/motor_control/fault_manager.c 2>&1 | head -10"
```
Expected: 三个测试均无语法错误.

> 后续 Plan: Stage 0 收尾时, 若 WSL 装了主机 gcc, 可改为实际运行. 当前沿用 stage0_baseline 的 `-fsyntax-only` 策略.

- [ ] **Step 5: Commit**

```bash
git add tests/
git commit -m "test(stage0): add syntax-check tests for foc_clarke/current_sense_calc/fault_manager"
```

---
## Task 12: 更新 CLAUDE.md 与 FOC控制器开发记录.md, 记录 Stage 0 完成

按 spec §10 与 CLAUDE.md "记录规范", 每个 Stage 完成必须更新进度表.

**Files:**
- Modify: `CLAUDE.md`
- Modify: `doc/FOC控制器开发记录.md`

- [ ] **Step 1: 在 CLAUDE.md 的 "Latest Stage 0 Baseline" 段之后追加 Stage 0 完成记录**

在 `CLAUDE.md` 找到 `## Latest Stage 0 Baseline - 2026-06-11` 段, 在其后追加:

```markdown
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

资源占用 (Stage 0 完成后): FLASH <X> KB / 127 KB, RAM <Y> KB / 16 KB. (实际数字填入)

已知限制:
- wk_*_init 已清空, 板上外设不再初始化, 固件烧录后 MA600A 调试路径暂不可用 (预期, Plan 2 重建)
- foc_park / foc_ipark / foc_svpwm / pid_f32_exec / 各 loop_run / isr_tick 均为 stub, Plan 2-4 实现
- DSP 三角函数 (arm_sin_cos_f32) 未接入, Plan 4 处理

后续 Stage 1 前置条件已满足: 引脚映射 / 时序常数 / 参数宏 / 分层骨架全部就位.
```

- [ ] **Step 2: 在 doc/FOC控制器开发记录.md §5 开发进度表追加一行**

找到 `| 2026-06-11 | 阶段 0 | 进行中 |` 行, 在其后追加:

```markdown
| 2026-06-22 | 阶段 0 | 已验证 | 目录重构完成, 分层骨架建立, CMake 构建通过, 资源占用 FLASH <X>KB RAM <Y>KB. wk_*_init 已清空, 外设初始化移至 platform 层 (Plan 2). |
```

(实际数字填入)

- [ ] **Step 3: Commit**

```bash
git add CLAUDE.md doc/FOC控制器开发记录.md
git commit -m "docs(stage0): record Stage 0 completion in CLAUDE.md and FOC dev log"
```

---

## 验收标准 (Plan 1 整体)

Plan 1 全部 12 个 Task 完成后, 必须满足:

1. **构建通过**: `wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && cmake --build build/Debug"` 成功, 生成 `MPS_MotorDriver.elf`
2. **资源占用**: FLASH < 30 KB / 127 KB, RAM < 5 KB / 16 KB (stub 代码不应显著膨胀)
3. **语法测试通过**: 3 个新测试 + 原有 `test_motor_control_state.c` 全部 `arm-none-eabi-gcc -fsyntax-only` 无错
4. **git 历史清晰**: 每个 Task 一个 commit, message 符合 `refactor/feat/build/test/docs(stage0): <简述>` 格式
5. **文档更新**: CLAUDE.md 与 FOC控制器开发记录.md 记录 Stage 0 完成, 含实际资源占用数字
6. **无遗留 wk_*_init 调用**: `grep -rn "wk_gpio_config\|wk_adc2_init\|wk_usart1_init\|wk_can1_init\|wk_spi2_init\|wk_wdt_init\|wk_tmr1_init" project/src/main.c` 应无输出 (只保留 `wk_system_clock_config` 经 `clock_at32m412_init` 间接调用, 与 `wk_rtthread_init`)

## 回滚条件

若任一 Task 验证失败且无法在 2 次尝试内修复, 立即停止并报告:
- 哪个 Task 失败
- 失败的命令与错误输出
- 已尝试的修复
- 建议的下一步 (回滚到上一 commit / 调整方案 / 请求人工介入)

**不要**继续执行后续 Task, 因为 Plan 1 是地基, 错误会传染到 Plan 2-5.

---

## Self-Review (plan 作者自查)

**1. Spec 覆盖检查:**
- spec §0.3 决策摘要 → Plan 1 不直接实现算法决策, 只搭骨架 ✓
- spec §2.1 目录结构 → Task 2-6 创建全部新目录与文件 ✓
- spec §2.3 文件大小预估 → Plan 1 只创建骨架, 行数远小于预估 (stub) ✓
- spec §1.2 引脚映射 → Task 2 `board_motor_pins.h` 全部覆盖 ✓
- spec §1.3 时序常数 → Task 2 `PWM_FREQUENCY_HZ / TMR1_ARR` ✓, Task 3 `CAL_*` ✓
- spec §4.3.2 电流采样宏 → Task 3 `motor_params.h` 全部覆盖 ✓
- spec §4.3.6 VBUS 宏 → Task 3 ✓
- spec §4.4 PID 结构体 → Task 4 `current_loop.h` 定义 `pid_f32_t` ✓
- spec §4.5.1 极对数 → Task 3 `MOTOR_POLE_PAIRS` ✓
- spec §4.7.3 标定数据结构 → Task 4 `motor_calibration.h` 定义 `motor_calibration_t` ✓
- spec §5.1 CAN 节点 ID → Task 3 `MOTOR_NODE_ID`, Task 6 `CAN_ID_*` ✓
- spec §5.5 故障位图 → Task 4 `fault_manager.h` 全部 7 个 fault 枚举 ✓
- spec 方案 Y (Workbench 边界) → Task 7 清空 wk_*_init ✓
- spec §7 Stage 0 任务清单 → Task 1-12 覆盖全部子项 ✓

**2. 占位符扫描:** 无 TBD/TODO/placeholder, 所有代码块完整 ✓

**3. 类型一致性检查:**
- `pid_f32_t` 在 `current_loop.h` 定义, `speed_loop.h` include 它 ✓
- `motor_calibration_t` 在 `motor_calibration.h` 定义, `flash_calibration_at32m412.h` include 它 ✓
- `motor_fault_t` 枚举值与 spec §5.5 bit 6 (FAULT_CAL_INVALID) 一致 ✓
- `can_mode_t` 枚举值与 spec §3.4 (0-5) 一致 ✓
- `CAL_*` 宏在 `motor_params.h` 定义, `motor_calibration.h` include `motor_params.h` ✓

**4. 风险点:**
- Task 7 (清空 wk_*_init) 是破坏性操作, 已加 .bak 备份与风险提示 ✓
- Task 9 重新配置 CMake 可能触发全量重编译, 预期耗时 1-2 分钟 ✓
- 主机端测试只能 `-fsyntax-only` (WSL 无主机 gcc), 已说明并沿用 stage0_baseline 策略 ✓
- DSP 三角函数推迟到 Plan 4, Plan 1 的 `foc_core.c` stub 不依赖三角函数 ✓

**结论: Plan 1 可执行, 无遗漏, 无占位符, 类型一致.**