# Full-Quadrant Current Sampling Reconstruction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不改变硬件的前提下，用固定顶部采样、逐相有效性判定和单相重构，为正反向电流环提供可靠反馈，并把电机命令限制为 `±1.5 A`。

**Architecture:** `current_reconstruction` 是不依赖芯片寄存器的纯 C 模块，负责 PWM 样本周期配对、低侧窗口判定、两相选择、第三相重构和连续帧保护状态。AT32 PWM 驱动维护该模块的 tracker，并在 TMR1 更新中断进入控制 ISR 前锁存与 ADC 样本对应的占空比；控制 ISR 只把有效/重构后的电流送入 Clarke/Park，无效帧保持上一组 d/q 电压并冻结闭环更新。

**Tech Stack:** C11、AT32M412 标准外设库、TMR1 中心对齐 PWM、ADC2 注入序列、主机 GCC 单元测试、Python 静态测试、CMake/arm-none-eabi-gcc。

## Global Constraints

- 不修改功率板、电流检测电路或 MP6540H 外围。
- 采用固定顶部单次采样；首版不实现动态 CH4 或双触发 DMA。
- `TMR1_ARR = 5624`，初始 `sample_tick = 5264`，即顶部保护时间 360 ticks。
- 低侧建立/消隐门槛为 180 ticks。
- 用户和速度环电流命令限制为 `-1.5 A` 至 `+1.5 A`。
- 软件相电流过流阈值为 `2.0 A`，连续 4 个有效帧后锁存。
- 少于两相有效时保持上一有效 d/q 电压；连续 8 个无效帧后锁存采样故障。
- 电流 PI 输出和积分限幅保持 `±2 V`，本计划不重新整定 PI。
- 原始 `Ia + Ib + Ic` 不平衡保护只在三相原始样本全部有效时运行。
- FOC ISR 不新增 RT-Thread API、动态内存或 VBUS 普通 ADC 转换。
- 先通过 `±50/100/200/500 mA` 全象限验证，再人工逐级验证 `±0.75/1.0/1.25/1.5 A`。
- 保留用户现有的 `debug.lksscope` 未提交改动；每个提交只暂存本任务列出的文件。

## File Structure

| 文件 | 动作 | 单一职责 |
|---|---|---|
| `application/motor_control/current_reconstruction.h` | 新建 | 定义采样计划、周期 tracker、重构结果和连续帧 guard 接口 |
| `application/motor_control/current_reconstruction.c` | 新建 | 实现纯采样窗口、重构、周期配对和去抖状态机 |
| `tests/current_sense/test_current_reconstruction.c` | 新建 | 主机端验证六种占空比排序、三种重构、周期对齐和保护计数 |
| `platform/at32m412/motor_pwm_at32m412.[ch]` | 修改 | 配置 CH4=5264，维护 tracker，并在 TMR1 更新边界锁存 sampled plan |
| `application/motor_control/motor_params.h` | 修改 | 保存 180/8/2.0A/1.5A 等产品策略常量 |
| `application/motor_control/fault_manager.h` | 修改 | 增加致命采样窗口故障位 |
| `application/motor_control/motor_control_isr.[ch]` | 修改 | 消费重构帧、执行保护、冻结 PI、保持输出并发布诊断快照 |
| `application/motor_app.c` | 修改 | 在 PWM/电流环初始化后初始化采样 guard 运行状态 |
| `application/motor_shell.c` | 修改 | 显示 raw/corrected 电流、窗口、占空比、重构和故障计数 |
| `CMakeLists.txt` | 修改 | 将纯重构模块加入固件构建 |
| `tests/motor_control/test_current_sampling_static.py` | 新建 | 防止原始电流绕过重构，核对阈值、故障位、PWM 时序和诊断接口 |
| `tests/stage5_bench.py` | 修改 | 把自动台架矩阵限制在 `±500 mA`，按新诊断量判定采样质量 |
| `doc/调试记录.md` | 修改 | 记录示波器门禁、低电流结果和是否允许扩大电流 |

---

### Task 1: Pure reconstruction, PWM-cycle tracker, and guard logic

**Files:**
- Create: `application/motor_control/current_reconstruction.h`
- Create: `application/motor_control/current_reconstruction.c`
- Create: `tests/current_sense/test_current_reconstruction.c`
- Modify: `CMakeLists.txt:42-72`

**Interfaces:**
- Consumes: 三相 ADC 换算后的安培值、与样本配对的三相 duty、`sample_tick`、180-tick blanking。
- Produces: `current_sample_tracker_*()`、`current_reconstruction_run()`、`current_sample_guard_*()`；Task 2 和 Task 3 直接使用这些签名。

- [ ] **Step 1: Write the failing host test**

Create `tests/current_sense/test_current_reconstruction.c` with these concrete cases:

```c
#include "current_reconstruction.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 1.0e-6f);
}

static current_sample_plan_t plan(uint16_t a, uint16_t b, uint16_t c)
{
    current_sample_plan_t p;
    p.duty_a = a;
    p.duty_b = b;
    p.duty_c = c;
    p.sample_tick = 5264u;
    return p;
}

static void test_tracker_pairs_previous_active_cycle(void)
{
    current_sample_tracker_t t;
    current_sample_plan_t sampled;

    current_sample_tracker_init(&t, 2812u, 5264u);
    current_sample_tracker_stage_duty(&t, 3000u, 3100u, 3200u);
    current_sample_tracker_latch_update(&t);
    current_sample_tracker_get_sampled(&t, &sampled);
    assert(sampled.duty_a == 2812u);
    assert(sampled.duty_b == 2812u);
    assert(sampled.duty_c == 2812u);

    current_sample_tracker_stage_duty(&t, 3300u, 3400u, 3500u);
    current_sample_tracker_latch_update(&t);
    current_sample_tracker_get_sampled(&t, &sampled);
    assert(sampled.duty_a == 3000u);
    assert(sampled.duty_b == 3100u);
    assert(sampled.duty_c == 3200u);

    current_sample_tracker_stage_trigger(&t, 5100u);
    current_sample_tracker_latch_update(&t);
    current_sample_tracker_get_sampled(&t, &sampled);
    assert(sampled.sample_tick == 5264u);
    current_sample_tracker_latch_update(&t);
    current_sample_tracker_get_sampled(&t, &sampled);
    assert(sampled.sample_tick == 5100u);
}

static void test_rearm_uses_values_that_loaded_while_irq_was_off(void)
{
    current_sample_tracker_t t;
    current_sample_plan_t sampled;
    current_sample_tracker_init(&t, 2812u, 5264u);
    current_sample_tracker_stage_duty(&t, 2000u, 2100u, 2200u);
    current_sample_tracker_rearm_from_next(&t);
    current_sample_tracker_get_sampled(&t, &sampled);
    assert(sampled.duty_a == 2000u);
    assert(sampled.duty_b == 2100u);
    assert(sampled.duty_c == 2200u);
}

static void test_exactly_two_valid_reconstructs_each_missing_phase(void)
{
    current_reconstruction_result_t r;

    current_reconstruction_run(&(current_sample_plan_t){5200u, 3000u, 3200u, 5264u},
                               99.0f, 0.20f, -0.05f, 180u, &r);
    assert(r.frame_valid && r.valid_mask == (CURRENT_PHASE_B_MASK | CURRENT_PHASE_C_MASK));
    assert(r.reconstructed_phase == CURRENT_RECON_PHASE_A);
    assert_close(r.ia, -0.15f);

    current_reconstruction_run(&(current_sample_plan_t){3000u, 5200u, 3200u, 5264u},
                               0.20f, 99.0f, -0.05f, 180u, &r);
    assert(r.reconstructed_phase == CURRENT_RECON_PHASE_B);
    assert_close(r.ib, -0.15f);

    current_reconstruction_run(&(current_sample_plan_t){3000u, 3200u, 5200u, 5264u},
                               0.20f, -0.05f, 99.0f, 180u, &r);
    assert(r.reconstructed_phase == CURRENT_RECON_PHASE_C);
    assert_close(r.ic, -0.15f);
}

static void test_all_valid_drops_smallest_margin_for_all_six_orders(void)
{
    static const uint16_t duty[6][3] = {
        {3000u, 3200u, 3400u}, {3000u, 3400u, 3200u},
        {3200u, 3000u, 3400u}, {3400u, 3000u, 3200u},
        {3200u, 3400u, 3000u}, {3400u, 3200u, 3000u}
    };
    static const current_reconstructed_phase_t expected[6] = {
        CURRENT_RECON_PHASE_C, CURRENT_RECON_PHASE_B,
        CURRENT_RECON_PHASE_C, CURRENT_RECON_PHASE_A,
        CURRENT_RECON_PHASE_B, CURRENT_RECON_PHASE_A
    };
    current_reconstruction_result_t r;
    unsigned i;

    for (i = 0u; i < 6u; ++i) {
        current_sample_plan_t p = plan(duty[i][0], duty[i][1], duty[i][2]);
        current_reconstruction_run(&p, 0.20f, -0.05f, 17.0f, 180u, &r);
        assert(r.frame_valid);
        assert(r.valid_mask == CURRENT_PHASE_ALL_MASK);
        assert(r.reconstructed_phase == expected[i]);
        assert_close(r.ia + r.ib + r.ic, 0.0f);
    }
}

static void test_fewer_than_two_valid_is_invalid(void)
{
    current_reconstruction_result_t r;
    current_sample_plan_t p = plan(3000u, 5200u, 5200u);
    current_reconstruction_run(&p, 0.2f, 0.3f, 0.4f, 180u, &r);
    assert(!r.frame_valid);
    assert(r.reconstructed_phase == CURRENT_RECON_PHASE_NONE);
}

static void test_guard_debounce_and_reset(void)
{
    current_sample_guard_t g;
    current_sample_action_t action;
    unsigned i;

    current_sample_guard_init(&g);
    for (i = 0u; i < 7u; ++i) {
        action = current_sample_guard_step(&g, false, false, 4u, 8u);
        assert(action == CURRENT_SAMPLE_ACTION_HOLD);
    }
    action = current_sample_guard_step(&g, false, false, 4u, 8u);
    assert(action == CURRENT_SAMPLE_ACTION_TRIP_INVALID);
    assert(g.invalid_total == 8u && g.invalid_consecutive == 8u);

    current_sample_guard_reset_consecutive(&g);
    for (i = 0u; i < 3u; ++i) {
        assert(current_sample_guard_step(&g, true, true, 4u, 8u) == CURRENT_SAMPLE_ACTION_USE);
    }
    assert(current_sample_guard_step(&g, true, true, 4u, 8u) ==
           CURRENT_SAMPLE_ACTION_TRIP_OVERCURRENT);
    assert(g.overcurrent_consecutive == 4u);

    assert(current_sample_guard_step(&g, true, false, 4u, 8u) == CURRENT_SAMPLE_ACTION_USE);
    assert(g.overcurrent_consecutive == 0u && g.invalid_consecutive == 0u);
}

int main(void)
{
    test_tracker_pairs_previous_active_cycle();
    test_rearm_uses_values_that_loaded_while_irq_was_off();
    test_exactly_two_valid_reconstructs_each_missing_phase();
    test_all_valid_drops_smallest_margin_for_all_six_orders();
    test_fewer_than_two_valid_is_invalid();
    test_guard_debounce_and_reset();
    puts("current reconstruction: all tests passed");
    return 0;
}
```

- [ ] **Step 2: Run the new test and verify the red state**

Run:

```powershell
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control tests/current_sense/test_current_reconstruction.c application/motor_control/current_reconstruction.c -lm -o /tmp/test_current_reconstruction && /tmp/test_current_reconstruction"
```

Expected: FAIL because `current_reconstruction.h` and `.c` do not exist.

- [ ] **Step 3: Define the complete public interface**

Create `application/motor_control/current_reconstruction.h`:

```c
#ifndef CURRENT_RECONSTRUCTION_H
#define CURRENT_RECONSTRUCTION_H

#include <stdbool.h>
#include <stdint.h>

#define CURRENT_PHASE_A_MASK   (1u << 0)
#define CURRENT_PHASE_B_MASK   (1u << 1)
#define CURRENT_PHASE_C_MASK   (1u << 2)
#define CURRENT_PHASE_ALL_MASK (CURRENT_PHASE_A_MASK | CURRENT_PHASE_B_MASK | CURRENT_PHASE_C_MASK)

typedef enum {
    CURRENT_RECON_PHASE_NONE = 0u,
    CURRENT_RECON_PHASE_A = 1u,
    CURRENT_RECON_PHASE_B = 2u,
    CURRENT_RECON_PHASE_C = 3u
} current_reconstructed_phase_t;

typedef struct {
    uint16_t duty_a;
    uint16_t duty_b;
    uint16_t duty_c;
    uint16_t sample_tick;
} current_sample_plan_t;

typedef struct {
    current_sample_plan_t sampled;
    current_sample_plan_t active;
    current_sample_plan_t next;
} current_sample_tracker_t;

typedef struct {
    float raw_ia;
    float raw_ib;
    float raw_ic;
    float ia;
    float ib;
    float ic;
    uint16_t margin_a;
    uint16_t margin_b;
    uint16_t margin_c;
    uint8_t valid_mask;
    current_reconstructed_phase_t reconstructed_phase;
    bool frame_valid;
} current_reconstruction_result_t;

typedef enum {
    CURRENT_SAMPLE_ACTION_USE = 0,
    CURRENT_SAMPLE_ACTION_HOLD,
    CURRENT_SAMPLE_ACTION_TRIP_OVERCURRENT,
    CURRENT_SAMPLE_ACTION_TRIP_INVALID
} current_sample_action_t;

typedef struct {
    uint16_t overcurrent_consecutive;
    uint16_t invalid_consecutive;
    uint32_t invalid_total;
} current_sample_guard_t;

void current_sample_tracker_init(current_sample_tracker_t *tracker,
                                 uint16_t initial_duty,
                                 uint16_t initial_sample_tick);
void current_sample_tracker_stage_duty(current_sample_tracker_t *tracker,
                                       uint16_t duty_a, uint16_t duty_b, uint16_t duty_c);
void current_sample_tracker_stage_trigger(current_sample_tracker_t *tracker,
                                          uint16_t sample_tick);
void current_sample_tracker_latch_update(current_sample_tracker_t *tracker);
void current_sample_tracker_rearm_from_next(current_sample_tracker_t *tracker);
void current_sample_tracker_get_sampled(const current_sample_tracker_t *tracker,
                                        current_sample_plan_t *out);

void current_reconstruction_run(const current_sample_plan_t *plan,
                                float raw_ia, float raw_ib, float raw_ic,
                                uint16_t blanking_ticks,
                                current_reconstruction_result_t *out);

void current_sample_guard_init(current_sample_guard_t *guard);
void current_sample_guard_reset_consecutive(current_sample_guard_t *guard);
current_sample_action_t current_sample_guard_step(current_sample_guard_t *guard,
                                                  bool frame_valid,
                                                  bool overcurrent,
                                                  uint16_t overcurrent_limit,
                                                  uint16_t invalid_limit);

#endif
```

- [ ] **Step 4: Implement the minimal pure logic**

Create `application/motor_control/current_reconstruction.c`. Implement these exact rules:

```c
#include "current_reconstruction.h"
#include <string.h>

static uint16_t margin_ticks(uint16_t sample_tick, uint16_t duty)
{
    return sample_tick > duty ? (uint16_t)(sample_tick - duty) : 0u;
}

static bool margin_valid(uint16_t margin, uint16_t blanking_ticks)
{
    return margin >= blanking_ticks;
}

void current_sample_tracker_init(current_sample_tracker_t *t,
                                 uint16_t initial_duty,
                                 uint16_t initial_sample_tick)
{
    current_sample_plan_t p;
    p.duty_a = initial_duty;
    p.duty_b = initial_duty;
    p.duty_c = initial_duty;
    p.sample_tick = initial_sample_tick;
    t->sampled = p;
    t->active = p;
    t->next = p;
}

void current_sample_tracker_stage_duty(current_sample_tracker_t *t,
                                       uint16_t a, uint16_t b, uint16_t c)
{
    t->next.duty_a = a;
    t->next.duty_b = b;
    t->next.duty_c = c;
}

void current_sample_tracker_stage_trigger(current_sample_tracker_t *t, uint16_t tick)
{
    t->next.sample_tick = tick;
}

void current_sample_tracker_latch_update(current_sample_tracker_t *t)
{
    t->sampled = t->active;
    t->active = t->next;
}

void current_sample_tracker_rearm_from_next(current_sample_tracker_t *t)
{
    t->sampled = t->next;
    t->active = t->next;
}

void current_sample_tracker_get_sampled(const current_sample_tracker_t *t,
                                        current_sample_plan_t *out)
{
    *out = t->sampled;
}
```

Implement `current_reconstruction_run()` exactly as follows. Strict `<` comparisons retain C as the deterministic reconstructed phase when margins tie, then prefer B and A only when their margin is strictly smaller:

```c
void current_reconstruction_run(const current_sample_plan_t *plan,
                                float raw_ia, float raw_ib, float raw_ic,
                                uint16_t blanking_ticks,
                                current_reconstruction_result_t *out)
{
    uint8_t valid_count;
    uint16_t smallest_margin;

    memset(out, 0, sizeof(*out));
    out->raw_ia = raw_ia;
    out->raw_ib = raw_ib;
    out->raw_ic = raw_ic;
    out->margin_a = margin_ticks(plan->sample_tick, plan->duty_a);
    out->margin_b = margin_ticks(plan->sample_tick, plan->duty_b);
    out->margin_c = margin_ticks(plan->sample_tick, plan->duty_c);

    if (margin_valid(out->margin_a, blanking_ticks)) out->valid_mask |= CURRENT_PHASE_A_MASK;
    if (margin_valid(out->margin_b, blanking_ticks)) out->valid_mask |= CURRENT_PHASE_B_MASK;
    if (margin_valid(out->margin_c, blanking_ticks)) out->valid_mask |= CURRENT_PHASE_C_MASK;

    valid_count = 0u;
    if ((out->valid_mask & CURRENT_PHASE_A_MASK) != 0u) valid_count++;
    if ((out->valid_mask & CURRENT_PHASE_B_MASK) != 0u) valid_count++;
    if ((out->valid_mask & CURRENT_PHASE_C_MASK) != 0u) valid_count++;
    if (valid_count < 2u) return;

    if (valid_count == 2u) {
        if ((out->valid_mask & CURRENT_PHASE_A_MASK) == 0u) {
            out->reconstructed_phase = CURRENT_RECON_PHASE_A;
        } else if ((out->valid_mask & CURRENT_PHASE_B_MASK) == 0u) {
            out->reconstructed_phase = CURRENT_RECON_PHASE_B;
        } else {
            out->reconstructed_phase = CURRENT_RECON_PHASE_C;
        }
    } else {
        out->reconstructed_phase = CURRENT_RECON_PHASE_C;
        smallest_margin = out->margin_c;
        if (out->margin_b < smallest_margin) {
            smallest_margin = out->margin_b;
            out->reconstructed_phase = CURRENT_RECON_PHASE_B;
        }
        if (out->margin_a < smallest_margin) {
            out->reconstructed_phase = CURRENT_RECON_PHASE_A;
        }
    }

out->ia = raw_ia;
out->ib = raw_ib;
out->ic = raw_ic;
switch (out->reconstructed_phase) {
case CURRENT_RECON_PHASE_A:
    out->ia = -(out->ib + out->ic);
    break;
case CURRENT_RECON_PHASE_B:
    out->ib = -(out->ia + out->ic);
    break;
case CURRENT_RECON_PHASE_C:
    out->ic = -(out->ia + out->ib);
    break;
default:
    return;
}
out->frame_valid = true;
}
```

Implement the guard as:

```c
void current_sample_guard_init(current_sample_guard_t *g)
{
    memset(g, 0, sizeof(*g));
}

void current_sample_guard_reset_consecutive(current_sample_guard_t *g)
{
    g->overcurrent_consecutive = 0u;
    g->invalid_consecutive = 0u;
}

current_sample_action_t current_sample_guard_step(current_sample_guard_t *g,
                                                  bool frame_valid,
                                                  bool overcurrent,
                                                  uint16_t oc_limit,
                                                  uint16_t invalid_limit)
{
    if (!frame_valid) {
        g->overcurrent_consecutive = 0u;
        g->invalid_total++;
        if (g->invalid_consecutive < UINT16_MAX) g->invalid_consecutive++;
        if (g->invalid_consecutive >= invalid_limit) {
            return CURRENT_SAMPLE_ACTION_TRIP_INVALID;
        }
        return CURRENT_SAMPLE_ACTION_HOLD;
    }

    g->invalid_consecutive = 0u;
    if (!overcurrent) {
        g->overcurrent_consecutive = 0u;
        return CURRENT_SAMPLE_ACTION_USE;
    }
    if (g->overcurrent_consecutive < UINT16_MAX) g->overcurrent_consecutive++;
    if (g->overcurrent_consecutive >= oc_limit) {
        return CURRENT_SAMPLE_ACTION_TRIP_OVERCURRENT;
    }
    return CURRENT_SAMPLE_ACTION_USE;
}
```

- [ ] **Step 5: Run the host test and verify green**

Run the Step 2 command again.

Expected: exit 0 and `current reconstruction: all tests passed`.

- [ ] **Step 6: Add the source to the firmware build and compile**

Add this line beside `current_loop.c` in `CMakeLists.txt`:

```cmake
${CMAKE_SOURCE_DIR}/application/motor_control/current_reconstruction.c
```

Run:

```powershell
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && cmake --build build/Debug"
```

Expected: exit 0 and `MPS_MotorDriver.elf` links successfully.

- [ ] **Step 7: Commit Task 1**

```powershell
git add -- application/motor_control/current_reconstruction.h application/motor_control/current_reconstruction.c tests/current_sense/test_current_reconstruction.c CMakeLists.txt
git diff --cached --check
git commit -m "feat: add current sample reconstruction core"
```

---

### Task 2: Pair ADC samples with the active PWM cycle

**Files:**
- Modify: `platform/at32m412/motor_pwm_at32m412.h:8-34`
- Modify: `platform/at32m412/motor_pwm_at32m412.c:1-155`
- Create: `tests/motor_control/test_current_sampling_static.py`

**Interfaces:**
- Consumes: Task 1 `current_sample_tracker_t` and tracker functions.
- Produces: `motor_pwm_at32m412_get_sample_plan(current_sample_plan_t *out)` for Task 3; TMR1 ISR latches the plan before calling `motor_control_isr_tick()`.

- [ ] **Step 1: Write the failing PWM/static test**

Create `tests/motor_control/test_current_sampling_static.py` with the first test set:

```python
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
PWM_C = ROOT / "platform" / "at32m412" / "motor_pwm_at32m412.c"
PWM_H = ROOT / "platform" / "at32m412" / "motor_pwm_at32m412.h"
ISR_C = ROOT / "application" / "motor_control" / "motor_control_isr.c"
ISR_H = ROOT / "application" / "motor_control" / "motor_control_isr.h"
PARAMS = ROOT / "application" / "motor_control" / "motor_params.h"
FAULT_H = ROOT / "application" / "motor_control" / "fault_manager.h"
SHELL = ROOT / "application" / "motor_shell.c"
CMAKE = ROOT / "CMakeLists.txt"

def read(path):
    return path.read_text(encoding="utf-8")

def test_pwm_sample_tick_and_cycle_pairing():
    c = read(PWM_C)
    h = read(PWM_H)
    assert re.search(r"#define\s+PWM_ADC_TRIGGER_TICKS\s+\(TMR1_ARR\s*-\s*360u\)", c)
    assert "current_sample_tracker_stage_duty" in c
    assert "current_sample_tracker_stage_trigger" in c
    assert "current_sample_tracker_rearm_from_next" in c
    assert "current_sample_tracker_latch_update" in c
    assert c.index("current_sample_tracker_latch_update") < c.index("motor_control_isr_tick();")
    assert "motor_pwm_at32m412_get_sample_plan" in h
    assert "motor_pwm_at32m412_get_sample_plan" in c

if __name__ == "__main__":
    test_pwm_sample_tick_and_cycle_pairing()
    print("current sampling static tests passed")
```

- [ ] **Step 2: Run the static test and verify it fails**

Run:

```powershell
python tests/motor_control/test_current_sampling_static.py
```

Expected: FAIL because the trigger remains 2500 and no tracker calls exist.

- [ ] **Step 3: Add the getter interface**

In `motor_pwm_at32m412.h`, include the pure type and add:

```c
#include "current_reconstruction.h"

void motor_pwm_at32m412_get_sample_plan(current_sample_plan_t *out);
```

- [ ] **Step 4: Integrate tracker state in the PWM driver**

In `motor_pwm_at32m412.c`:

```c
#define PWM_ADC_TRIGGER_TICKS (TMR1_ARR - 360u)

static current_sample_tracker_t s_sample_tracker;
```

After writing the initial 50% CCRs and CH4 in `motor_pwm_at32m412_init()` initialize the tracker:

```c
current_sample_tracker_init(&s_sample_tracker,
                            TMR1_ARR / 2u,
                            PWM_ADC_TRIGGER_TICKS);
```

Replace the duty setter body with clamped local values so hardware and tracker receive identical values:

```c
uint16_t a;
uint16_t b;
uint16_t c;
a = pwm_clamp_duty(phase_u);
b = pwm_clamp_duty(phase_v);
c = pwm_clamp_duty(phase_w);
tmr_channel_value_set(TMR1, PWM_PHASE_U_TMR_CHANNEL, a);
tmr_channel_value_set(TMR1, PWM_PHASE_V_TMR_CHANNEL, b);
tmr_channel_value_set(TMR1, PWM_PHASE_W_TMR_CHANNEL, c);
current_sample_tracker_stage_duty(&s_sample_tracker, a, b, c);
```

After clamping and writing CH4 in `motor_pwm_at32m412_set_adc_trigger_ticks()` add:

```c
current_sample_tracker_stage_trigger(&s_sample_tracker, ticks);
```

Before enabling the update interrupt in `motor_pwm_at32m412_enable_ovf_irq()` add:

```c
current_sample_tracker_rearm_from_next(&s_sample_tracker);
```

At the update ISR, lock the sample association before control reads ADC:

```c
if (tmr_flag_get(TMR1, TMR_OVF_FLAG) != RESET) {
    tmr_flag_clear(TMR1, TMR_OVF_FLAG);
    current_sample_tracker_latch_update(&s_sample_tracker);
    motor_control_isr_tick();
}
```

Implement the getter without reading CCR registers:

```c
void motor_pwm_at32m412_get_sample_plan(current_sample_plan_t *out)
{
    current_sample_tracker_get_sampled(&s_sample_tracker, out);
}
```

- [ ] **Step 5: Update the old trigger expectation**

In `tests/motor_control/test_current_loop_tuning_static.py`, replace the 2500-tick assertion with:

```python
assert "#define PWM_ADC_TRIGGER_TICKS (TMR1_ARR - 360u)" in pwm_c
```

Keep the test that confirms shell trigger sweeping remains present.

- [ ] **Step 6: Run Task 1 and Task 2 tests plus firmware build**

Run:

```powershell
python tests/motor_control/test_current_sampling_static.py
python tests/motor_control/test_current_loop_tuning_static.py
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control tests/current_sense/test_current_reconstruction.c application/motor_control/current_reconstruction.c -lm -o /tmp/test_current_reconstruction && /tmp/test_current_reconstruction && cmake --build build/Debug"
```

Expected: both Python scripts pass, the host C test prints its pass line, and the firmware build exits 0.

- [ ] **Step 7: Commit Task 2**

```powershell
git add -- platform/at32m412/motor_pwm_at32m412.h platform/at32m412/motor_pwm_at32m412.c tests/motor_control/test_current_sampling_static.py tests/motor_control/test_current_loop_tuning_static.py
git diff --cached --check
git commit -m "feat: align current samples with active pwm"
```

---

### Task 3: Consume reconstructed current and enforce safe protection

**Files:**
- Modify: `application/motor_control/motor_params.h:16-29,87-104`
- Modify: `application/motor_control/fault_manager.h:11-26`
- Modify: `application/motor_control/motor_control_isr.c:15-27,74-125,155-429,431-739`
- Modify: `application/motor_control/motor_control_isr.h:96-147`
- Modify: `application/motor_app.c:28-52`
- Modify: `tests/motor_control/test_current_sampling_static.py`
- Modify: `tests/motor_control/test_current_fault_debounce_static.py`
- Modify: `tests/motor_control/test_speed_loop.c:17-19,111`

**Interfaces:**
- Consumes: `motor_pwm_at32m412_get_sample_plan()` and all Task 1 reconstruction/guard interfaces.
- Produces: corrected `ia/ib/ic`, sample validity diagnostics, `FAULT_CURRENT_SAMPLE`, and held d/q output behavior used by Task 4 shell output.

- [ ] **Step 1: Extend the static test with policy and data-path assertions**

Append these functions to `tests/motor_control/test_current_sampling_static.py` and call them in `__main__`:

```python
def test_limits_and_fatal_sample_fault():
    params = read(PARAMS)
    fault = read(FAULT_H)
    assert re.search(r"#define\s+CURRENT_SAMPLE_BLANKING_TICKS\s+180u", params)
    assert re.search(r"#define\s+CURRENT_SAMPLE_INVALID_LIMIT\s+8u", params)
    assert re.search(r"#define\s+IQ_OVERCURRENT_A\s+2\.0f", params)
    assert re.search(r"#define\s+IQ_MAX_A\s+1\.5f", params)
    assert re.search(r"#define\s+IQ_MAX_MA\s+1500\b", params)
    assert "FAULT_CURRENT_SAMPLE" in fault
    fatal = re.search(r"#define\s+FAULT_FATAL_MASK[\s\S]*?\n\n", fault).group(0)
    assert "FAULT_CURRENT_SAMPLE" in fatal

def test_isr_uses_reconstruction_before_clarke_and_freezes_invalid_frames():
    source = read(ISR_C)
    assert "motor_pwm_at32m412_get_sample_plan" in source
    assert "current_reconstruction_run" in source
    assert "current_sample_guard_step" in source
    assert source.index("current_reconstruction_run") < source.index("foc_clarke(")
    assert "sample.frame_valid" in source
    assert "s_held_vd_ref" in source and "s_held_vq_ref" in source
    assert "s_dbg_pi_freeze_count" in source
    assert "fault_manager_set(FAULT_CURRENT_SAMPLE)" in source
    assert "sample.valid_mask == CURRENT_PHASE_ALL_MASK" in source

def test_raw_currents_cannot_bypass_reconstruction():
    source = read(ISR_C)
    assert "foc_clarke(sample.ia, sample.ib, sample.ic" in source
    assert "foc_clarke(ia, ib, ic" not in source
```

Expected red state: limits remain 4.5/5.0 A, fault bit is absent, and ISR calls Clarke with raw converted currents.

- [ ] **Step 2: Run the expanded test and verify it fails**

Run `python tests/motor_control/test_current_sampling_static.py`.

Expected: FAIL in the new policy/data-path tests.

- [ ] **Step 3: Apply exact product limits and fault bit**

In `motor_params.h`, set:

```c
#define IQ_OVERCURRENT_A                  2.0f
#define OVERCURRENT_DEBOUNCE_TICKS        4u
#define CURRENT_SAMPLE_BLANKING_TICKS     180u
#define CURRENT_SAMPLE_INVALID_LIMIT      8u
#define IQ_MAX_A                          1.5f
#define IQ_MAX_MA                         1500
```

Keep `PID_CURRENT_INTEGRAL_LIMIT` and `PID_CURRENT_OUT_LIMIT` at `2.0f`.

In `fault_manager.h`, add a unique bit and make it fatal:

```c
FAULT_CURRENT_SAMPLE   = 1u << 7,
```

```c
#define FAULT_FATAL_MASK  (FAULT_DRIVER | FAULT_OVERCURRENT | FAULT_SENSOR | \
                           FAULT_UNDERVOLTAGE | FAULT_OVERVOLTAGE | \
                           FAULT_CURRENT_SAMPLE)
```

Update `tests/motor_control/test_speed_loop.c` so its local `IQ_MAX_A` is `1.5f`; the existing saturation assertion must still pass.

- [ ] **Step 4: Add ISR runtime state and diagnostics fields**

Include `current_reconstruction.h` in `motor_control_isr.c`. Replace `s_oc_consec` with:

```c
static current_sample_guard_t s_sample_guard;
static volatile uint32_t s_dbg_pi_freeze_count;
static volatile uint8_t s_dbg_sample_valid_mask;
static volatile uint8_t s_dbg_reconstructed_phase;
static volatile uint16_t s_dbg_margin_a;
static volatile uint16_t s_dbg_margin_b;
static volatile uint16_t s_dbg_margin_c;
static volatile uint16_t s_dbg_sample_duty_a;
static volatile uint16_t s_dbg_sample_duty_b;
static volatile uint16_t s_dbg_sample_duty_c;
static volatile uint16_t s_dbg_sample_tick;
static volatile int32_t s_dbg_raw_ia_ma;
static volatile int32_t s_dbg_raw_ib_ma;
static volatile int32_t s_dbg_raw_ic_ma;
static float s_held_vd_ref;
static float s_held_vq_ref;
```

Add matching fixed-width fields to `motor_control_isr_debug_t`:

```c
int32_t raw_ia_ma;
int32_t raw_ib_ma;
int32_t raw_ic_ma;
uint8_t sample_valid_mask;
uint8_t reconstructed_phase;
uint16_t sample_margin_a;
uint16_t sample_margin_b;
uint16_t sample_margin_c;
uint16_t sample_duty_a;
uint16_t sample_duty_b;
uint16_t sample_duty_c;
uint16_t sample_tick;
uint32_t sample_invalid_total;
uint16_t sample_invalid_consecutive;
uint32_t pi_freeze_count;
```

Copy every field in `motor_control_isr_get_debug()`; do not expose pointers to ISR-owned state.

- [ ] **Step 5: Replace unconditional raw-current protection with reconstruction**

At the beginning of `motor_control_isr_tick()`, declare `current_sample_plan_t plan`, `current_reconstruction_result_t sample`, `current_sample_action_t sample_action`, and `bool phase_overcurrent` with the other C90-compatible declarations.

After raw-to-amp conversion:

```c
motor_pwm_at32m412_get_sample_plan(&plan);
current_reconstruction_run(&plan, ia, ib, ic,
                           CURRENT_SAMPLE_BLANKING_TICKS, &sample);

s_dbg_raw_ia_ma = (int32_t)(sample.raw_ia * 1000.0f);
s_dbg_raw_ib_ma = (int32_t)(sample.raw_ib * 1000.0f);
s_dbg_raw_ic_ma = (int32_t)(sample.raw_ic * 1000.0f);
s_dbg_sample_valid_mask = sample.valid_mask;
s_dbg_reconstructed_phase = (uint8_t)sample.reconstructed_phase;
s_dbg_margin_a = sample.margin_a;
s_dbg_margin_b = sample.margin_b;
s_dbg_margin_c = sample.margin_c;
s_dbg_sample_duty_a = plan.duty_a;
s_dbg_sample_duty_b = plan.duty_b;
s_dbg_sample_duty_c = plan.duty_c;
s_dbg_sample_tick = plan.sample_tick;

if (sample.frame_valid) {
    s_dbg_ia_ma = (int32_t)(sample.ia * 1000.0f);
    s_dbg_ib_ma = (int32_t)(sample.ib * 1000.0f);
    s_dbg_ic_ma = (int32_t)(sample.ic * 1000.0f);
}
```

Only advance guard/protection while `mc->state == MOTOR_CONTROL_STATE_ENABLED`:

```c
phase_overcurrent = sample.frame_valid &&
    (fabsf(sample.ia) > IQ_OVERCURRENT_A ||
     fabsf(sample.ib) > IQ_OVERCURRENT_A ||
     fabsf(sample.ic) > IQ_OVERCURRENT_A);

if (mc->state == MOTOR_CONTROL_STATE_ENABLED) {
    sample_action = current_sample_guard_step(&s_sample_guard,
                                              sample.frame_valid,
                                              phase_overcurrent,
                                              OVERCURRENT_DEBOUNCE_TICKS,
                                              CURRENT_SAMPLE_INVALID_LIMIT);
    if (phase_overcurrent) s_dbg_oc_hits++;
    if (sample_action == CURRENT_SAMPLE_ACTION_TRIP_OVERCURRENT) {
        fault_manager_set(FAULT_OVERCURRENT);
    } else if (sample_action == CURRENT_SAMPLE_ACTION_TRIP_INVALID) {
        fault_manager_set(FAULT_CURRENT_SAMPLE);
    }
} else {
    current_sample_guard_reset_consecutive(&s_sample_guard);
    sample_action = CURRENT_SAMPLE_ACTION_USE;
}
```

Run raw KCL imbalance only under this exact condition:

```c
if (mc->state == MOTOR_CONTROL_STATE_ENABLED &&
    sample.valid_mask == CURRENT_PHASE_ALL_MASK) {
    i_sum = sample.raw_ia + sample.raw_ib + sample.raw_ic;
    if (fabsf(i_sum) > IMBALANCE_THRESHOLD_A) {
        s_dbg_imbal_hits++;
        s_imbal_consec++;
        if (s_imbal_consec >= IMBALANCE_DEBOUNCE_TICKS) {
            fault_manager_set(FAULT_OVERCURRENT);
        }
    } else {
        s_imbal_consec = 0u;
    }
} else {
    s_imbal_consec = 0u;
}
```

- [ ] **Step 6: Freeze current and speed control on invalid frames**

In both CURRENT and SPEED branches, always compute/update `theta`, then branch on `sample.frame_valid`.

Valid CURRENT frame:

```c
foc_clarke(sample.ia, sample.ib, sample.ic, &i_alpha, &i_beta);
foc_park(i_alpha, i_beta, theta, &id, &iq);
current_loop_run(id, iq, &vd_ref, &vq_ref);
s_held_vd_ref = vd_ref;
s_held_vq_ref = vq_ref;
```

Invalid CURRENT frame:

```c
vd_ref = s_held_vd_ref;
vq_ref = s_held_vq_ref;
s_dbg_pi_freeze_count++;
```

Valid SPEED frame runs `speed_loop_run()`, sets current targets, performs Clarke/Park and runs the current loop, then stores held d/q voltage. Invalid SPEED frame skips both PI calls and uses the same two held values. This freezes the outer PI as well as the inner PI, preventing `iq_ref` from moving while current feedback is unavailable.

In both branches, continue with existing IPark/SVPWM using `vd_ref`/`vq_ref`. Do not reuse stale `id/iq` for average accumulation; update `id`, `iq` and the 256-sample average only in the valid branch.

- [ ] **Step 7: Reset transient sampling state on lifecycle transitions**

Add:

```c
static void current_sampling_runtime_reset(void)
{
    current_sample_guard_reset_consecutive(&s_sample_guard);
    s_held_vd_ref = 0.0f;
    s_held_vq_ref = 0.0f;
    s_imbal_consec = 0u;
}
```

Add this public initialization function and declaration:

```c
void motor_control_isr_sampling_init(void)
{
    current_sample_guard_init(&s_sample_guard);
    s_dbg_pi_freeze_count = 0u;
    current_sampling_runtime_reset();
}
```

Call `current_sampling_runtime_reset()` from every open-loop, align, current, and speed `*_start()`/`*_stop()` before output is enabled or after the IRQ is disabled. Call `motor_control_isr_sampling_init()` from `motor_app_init()` immediately after `current_loop_init()`.

Do not clear `invalid_total` or `s_dbg_pi_freeze_count` on mode switches; they are boot-lifetime diagnostics. Only consecutive counts and held outputs reset.

- [ ] **Step 8: Update existing debounce tests and run the test set**

Replace the test body in `test_current_fault_debounce_static.py` with:

```python
def test_phase_overcurrent_uses_valid_frame_guard_and_short_debounce():
    params = PARAMS_H.read_text(encoding="utf-8")
    source = ISR_C.read_text(encoding="utf-8")
    assert re.search(r"#define\s+OVERCURRENT_DEBOUNCE_TICKS\s+4u", params)
    assert "current_sample_guard_step" in source
    assert "sample.frame_valid" in source
    assert "fault_manager_set(FAULT_OVERCURRENT)" in source
```

Run:

```powershell
python tests/motor_control/test_current_sampling_static.py
python tests/motor_control/test_current_fault_debounce_static.py
wsl.exe -e bash -lc "cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control tests/current_sense/test_current_reconstruction.c application/motor_control/current_reconstruction.c -lm -o /tmp/test_current_reconstruction && /tmp/test_current_reconstruction && gcc -std=c11 -Wall -Wextra -Werror tests/motor_control/test_speed_loop.c -lm -o /tmp/test_speed_loop && /tmp/test_speed_loop && cmake --build build/Debug"
```

Expected: all Python/C tests pass and firmware build exits 0.

- [ ] **Step 9: Commit Task 3**

```powershell
git add -- application/motor_control/motor_params.h application/motor_control/fault_manager.h application/motor_control/motor_control_isr.c application/motor_control/motor_control_isr.h application/motor_app.c tests/motor_control/test_current_sampling_static.py tests/motor_control/test_current_fault_debounce_static.py tests/motor_control/test_speed_loop.c
git diff --cached --check
git commit -m "fix: reject invalid current feedback frames"
```

---

### Task 4: Expose diagnostics and make the bench test safely full-quadrant

**Files:**
- Modify: `application/motor_shell.c:195-267,427-455`
- Modify: `tests/motor_control/test_current_sampling_static.py`
- Modify: `tests/stage5_bench.py`

**Interfaces:**
- Consumes: Task 3 fields in `motor_control_isr_debug_t` and `FAULT_CURRENT_SAMPLE`.
- Produces: stable `sample`, `raw_current`, `current`, and `sample_count` shell lines parsed by the bench test.

- [ ] **Step 1: Add failing shell/bench assertions to the static test**

Append and call:

```python
def test_shell_reports_sample_quality_and_fault_name():
    shell = read(SHELL)
    for token in (
        "raw_current", "sample     :", "sample_duty", "sample_margin",
        "sample_count", "valid_mask", "recon", "pi_freeze",
        "CURRENT_SAMPLE",
    ):
        assert token in shell

def test_bench_matrix_is_full_quadrant_but_stays_below_500ma():
    bench = read(ROOT / "tests" / "stage5_bench.py")
    assert "CURRENT_TEST_POINTS_MA = (-50, 50, -100, 100, -200, 200, -500, 500)" in bench
    assert "max(20, abs(target_ma) * 0.10)" in bench
    assert 'abs(snapshot["id_avg"]) <= 100' in bench
    assert 'snapshot["invalid_consecutive"] == 0' in bench
    assert "750" not in re.search(r"CURRENT_TEST_POINTS_MA\s*=\s*\([^\)]*\)", bench).group(0)
```

Run the static test and verify it fails because neither diagnostics nor the matrix exist.

- [ ] **Step 2: Print deterministic diagnostics**

In both `mc_debug` and `mc_current`, retain existing lines and add these formats exactly so Python parsing remains stable:

```c
rt_kprintf("raw_current: ia=%ld ib=%ld ic=%ld mA\n",
           (long)dbg.raw_ia_ma, (long)dbg.raw_ib_ma, (long)dbg.raw_ic_ma);
rt_kprintf("sample     : tick=%u valid_mask=0x%02X recon=%u\n",
           dbg.sample_tick, dbg.sample_valid_mask, dbg.reconstructed_phase);
rt_kprintf("sample_duty: a=%u b=%u c=%u\n",
           dbg.sample_duty_a, dbg.sample_duty_b, dbg.sample_duty_c);
rt_kprintf("sample_margin: a=%u b=%u c=%u\n",
           dbg.sample_margin_a, dbg.sample_margin_b, dbg.sample_margin_c);
rt_kprintf("sample_count: invalid_total=%lu invalid_consecutive=%u pi_freeze=%lu\n",
           (unsigned long)dbg.sample_invalid_total,
           dbg.sample_invalid_consecutive,
           (unsigned long)dbg.pi_freeze_count);
```

In `fault`, add:

```c
if (f & FAULT_CURRENT_SAMPLE) rt_kprintf("  CURRENT_SAMPLE\n");
```

- [ ] **Step 3: Replace the old accuracy sections with the approved safe matrix**

At the top of `tests/stage5_bench.py` define:

```python
CURRENT_TEST_POINTS_MA = (-50, 50, -100, 100, -200, 200, -500, 500)
```

Add parsers for the existing `cur_avg` line and new `sample_count` line:

```python
def parse_current_snapshot(text):
    snapshot = {}
    avg = re.search(r"cur_avg\s*:\s*id=(-?\d+)mA iq=(-?\d+)mA", text)
    count = re.search(
        r"sample_count:\s*invalid_total=(\d+) invalid_consecutive=(\d+) pi_freeze=(\d+)",
        text,
    )
    sample = re.search(r"sample\s*:\s*tick=(\d+) valid_mask=0x([0-9A-Fa-f]+) recon=(\d+)", text)
    if not avg or not count or not sample:
        return None
    snapshot["id_avg"] = int(avg.group(1))
    snapshot["iq_avg"] = int(avg.group(2))
    snapshot["invalid_total"] = int(count.group(1))
    snapshot["invalid_consecutive"] = int(count.group(2))
    snapshot["pi_freeze"] = int(count.group(3))
    snapshot["sample_tick"] = int(sample.group(1))
    snapshot["valid_mask"] = int(sample.group(2), 16)
    snapshot["recon"] = int(sample.group(3))
    return snapshot
```

Add these complete helpers and replace the old automatic calibration/ramp/step sections with `section_full_quadrant_current()`:

```python
def read_current_snapshot(ser):
    text = send_cmd(ser, "mc_debug", wait_after=0.6)
    snapshot = parse_current_snapshot(text)
    assert snapshot is not None, f"missing current/sample diagnostics: {text}"
    return snapshot


def read_fault_value(ser):
    text = send_cmd(ser, "fault", wait_after=0.2)
    match = re.search(r"fault\s*=\s*0x([0-9A-Fa-f]+)", text)
    assert match, f"missing fault value: {text}"
    return int(match.group(1), 16)


def section_full_quadrant_current(ser, log):
    status = send_cmd(ser, "enc_cal_status", wait_after=0.3)
    assert "DONE" in status, "encoder calibration invalid; complete calibration before this test"
    send_cmd(ser, "fault_clear", wait_after=0.2)

    for target_ma in CURRENT_TEST_POINTS_MA:
        before = read_current_snapshot(ser)
        try:
            start = send_cmd(ser, f"mc_cur {target_ma} enc", wait_after=0.5)
            assert "current loop" in start, f"start failed at {target_ma}mA: {start}"
            time.sleep(0.5)
            snapshots = [read_current_snapshot(ser) for _ in range(3)]
            snapshot = snapshots[-1]
            tolerance_ma = max(20, abs(target_ma) * 0.10)
            assert abs(snapshot["iq_avg"] - target_ma) <= tolerance_ma
            assert abs(snapshot["id_avg"]) <= 100
            assert snapshot["invalid_consecutive"] == 0
            assert snapshot["invalid_total"] == before["invalid_total"]
            assert snapshot["pi_freeze"] == before["pi_freeze"]
            fault_value = read_fault_value(ser)
            assert (fault_value & 0x9F) == 0
            log.append(
                f"PASS {target_ma:+d}mA: id={snapshot['id_avg']}mA "
                f"iq={snapshot['iq_avg']}mA mask=0x{snapshot['valid_mask']:02X} "
                f"recon={snapshot['recon']}"
            )
        finally:
            send_cmd(ser, "mc_stop", wait_after=0.3)
```

In `main()`, call `section_a()` followed only by `section_full_quadrant_current()`. Remove calls that erase calibration, start automatic calibration, or exercise the old ramp/step sections.

In `section_a()`, update the fatal-mask precondition to include the new bit:

```python
assert (fault_val & 0x9F) == 0, f"A: fatal fault active: {out.strip()}"
```

The script must not erase calibration, run automatic encoder calibration, or command any point above 500 mA. If calibration is invalid, exit with a clear instruction to complete the existing calibration workflow first.

- [ ] **Step 4: Run parser/static checks**

Run:

```powershell
python -m py_compile tests/stage5_bench.py
python tests/motor_control/test_current_sampling_static.py
```

Expected: syntax check exits 0 and static tests print `current sampling static tests passed`.

- [ ] **Step 5: Commit Task 4**

```powershell
git add -- application/motor_shell.c tests/motor_control/test_current_sampling_static.py tests/stage5_bench.py
git diff --cached --check
git commit -m "test: add current sampling diagnostics and bench matrix"
```

---

### Task 5: Full software verification and hardware-gated bring-up

**Files:**
- Modify: `doc/调试记录.md`

**Interfaces:**
- Consumes: all prior tasks and the physical oscilloscope/bench setup.
- Produces: build/test evidence and an explicit decision whether testing may advance beyond `±500 mA`.

- [ ] **Step 1: Run every relevant static test**

Run:

```powershell
$tests = Get-ChildItem tests\motor_control\test_*_static.py, tests\encoder_service\test_*_static.py
foreach ($test in $tests) { python $test.FullName; if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE } }
python tests\test_pwm_mapping_static.py
```

Expected: every script exits 0; no assertion traceback.

- [ ] **Step 2: Run all host C tests relevant to the changed loops**

Run:

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver; gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control tests/current_sense/test_current_reconstruction.c application/motor_control/current_reconstruction.c -lm -o /tmp/test_current_reconstruction; /tmp/test_current_reconstruction; gcc -std=c11 -Wall -Wextra -Werror tests/motor_control/test_current_loop.c -lm -o /tmp/test_current_loop; /tmp/test_current_loop; gcc -std=c11 -Wall -Wextra -Werror tests/motor_control/test_speed_loop.c -lm -o /tmp/test_speed_loop; /tmp/test_speed_loop"
```

Expected: all three executables exit 0 and print their pass summaries.

- [ ] **Step 3: Build the firmware from the existing Debug tree**

Run:

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver; cmake --build build/Debug; arm-none-eabi-size build/Debug/MPS_MotorDriver.elf"
```

Expected: build exits 0, ELF exists, and size output contains `text`, `data`, and `bss` columns.

- [ ] **Step 4: Perform the oscilloscope gate before closed-loop load**

With the driver disabled or current-limited supply set conservatively, observe one phase PWM, the CH4 trigger event, and its SOx signal. Record all of the following in `doc/调试记录.md`:

- CH4 effective sample tick is 5264.
- The three-channel ADC conversion window completes before the TMR1 top update.
- Whenever software marks a phase valid, that phase low side has been stable for at least 180 ticks.
- Software `sample_duty` corresponds to the PWM cycle immediately preceding the ISR, not the newly calculated duty.
- Sweep representative duty orderings and confirm `valid_mask`/`recon` match the traces.

If polarity, timer direction, or pairing is wrong, stop and correct Task 2; do not alter the 180-tick threshold to hide a mapping error.

- [ ] **Step 5: Run the automated low-current full-quadrant matrix**

After valid encoder calibration and with a current-limited supply:

```powershell
python tests/stage5_bench.py COM9
```

Expected for all eight points:

- `iq_avg` error is at most `max(20 mA, 10%)`.
- `|id_avg| <= 100 mA`.
- No fatal fault and no consecutive invalid frame.
- Neither `invalid_total` nor `pi_freeze` grows during a steady test point.

If any item fails, stop at `±500 mA` or below and append the command, raw/corrected currents, mask, margins, sampled duties and scope observation to `doc/调试记录.md`.

- [ ] **Step 6: Manually expand the current only after the gate passes**

Run one polarity and one magnitude at a time:

```text
mc_cur 750 enc
mc_cur -750 enc
mc_cur 1000 enc
mc_cur -1000 enc
mc_cur 1250 enc
mc_cur -1250 enc
mc_cur 1500 enc
mc_cur -1500 enc
```

After each command, inspect `mc_debug`, temperature, sound, supply current and voltage saturation, then issue `mc_stop`. Do not continue upward after abnormal temperature, abnormal noise, sustained `±2 V` saturation, any fatal fault, or increasing invalid/freeze counts.

- [ ] **Step 7: Verify speed mode only after current mode passes**

Start with low positive and negative electrical speed. Confirm speed PI output remains within `±1.5 A`, braking/reversal does not create invalid-frame streaks, and `iq` direction agrees with commanded torque. Do not tune speed PI in this task.

- [ ] **Step 8: Record evidence and commit the debug record**

Append a dated section to `doc/调试记录.md` containing:

- firmware commit hash;
- scope result and sample timing;
- all eight low-current points;
- invalid/freeze deltas;
- highest manually verified current;
- temperature/noise/saturation observations;
- explicit `允许继续速度环` or `禁止继续速度环` conclusion.

Then commit only the record:

```powershell
git add -- doc/调试记录.md
git diff --cached --check
git commit -m "docs: record full-quadrant current validation"
```

- [ ] **Step 9: Final repository integrity check**

Run:

```powershell
git status --short
git log -5 --oneline
git diff --check HEAD~5 HEAD
```

Expected: `debug.lksscope` remains the only pre-existing user modification; the five new commits are scoped to their task files; committed diffs have no whitespace errors.
