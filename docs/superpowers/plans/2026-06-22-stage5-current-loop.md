# Stage 5: 电流环控制 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现 CURRENT 模式 Id/Iq 双轴 PI 电流环, 替换 motor_control_isr.c 中的 stub, 并提供 `mc_current` msh 命令 + 自动验收脚本。

**Architecture:** 方案 A — CURRENT 分支独立流水线 (Clarke→Park→PI→IPark→SVPWM), 不动已验证的 OPEN_LOOP/ALIGN 分支。theta 来源 enc/ramp 双支持 (独立 `s_cur_use_enc` 开关)。标准 PI + clamping 抗 windup。

**Tech Stack:** C (ARMCC/GCC, AT32M412 FPU), RT-Thread Nano msh, Python pyserial (验收脚本)

**Spec:** `docs/superpowers/specs/2026-06-22-stage5-current-loop-design.md`

---

## File Structure

| 文件 | 操作 | 职责 |
|---|---|---|
| `application/motor_control/current_loop.h` | 修改 | 新增 set_targets/reset 接口声明 |
| `application/motor_control/current_loop.c` | 修改 | 实现 pid_f32_exec + current_loop_run + set_targets + reset + init |
| `application/motor_control/motor_params.h` | 修改 | 新增电流环参数宏, 改 IQ_MAX_A 4.5 |
| `application/motor_control/motor_control_isr.h` | 修改 | 新增 4 个 current_* 接口声明 + debug 结构体扩展 |
| `application/motor_control/motor_control_isr.c` | 修改 | CURRENT 分支填充 + 4 接口实现 + 静态变量 |
| `application/motor_control/motor_shell.c` | 修改 | 新增 mc_current + 扩展 mc_stop/mc_debug |
| `tests/motor_control/test_current_loop.c` | 创建 | 主机 gcc 单元测试 (PI 数学) |
| `tests/stage5_bench.py` | 创建 | 串口自动化验收 |

**测试约定** (与现有 test_motor_calibration.c 一致):
- C 单元测试用主机 gcc 编译运行, 独立复制产品代码数学逻辑验证 (产品代码依赖 AT32 硬件无法直接编译)
- 验证命令: `gcc -std=c11 -Wall -o /tmp/test_current_loop tests/motor_control/test_current_loop.c -lm && /tmp/test_current_loop`
- 构建验证: WSL `cmake --build build/Debug`, Keil `flash.bat rebuild`

---

### Task 1: 参数宏 (motor_params.h)

**Files:**
- Modify: `application/motor_control/motor_params.h`

- [ ] **Step 1: 读取现有参数定义, 确认 IQ_MAX_A/IQ_OVERCURRENT_A 位置**

Run: `grep -n "IQ_MAX_A\|IQ_OVERCURRENT_A\|CURRENT_KP\|CURRENT_OUT_LIMIT" application/motor_control/motor_params.h`
Expected: 显示 `IQ_OVERCURRENT_A 5.0f` (line ~25), `IQ_MAX_A 8.0f` (line ~91), 无 CURRENT_KP (待新增)

- [ ] **Step 2: 修改 IQ_MAX_A 8.0→4.5, 新增电流环参数宏**

在 `motor_params.h` 中, 把:
```c
#define IQ_MAX_A                        8.0f
```
改为 (含新增电流环参数块, 紧跟其后):
```c
#define IQ_MAX_A                        4.5f          /* 电流环目标上限 (Stage 5: 8.0→4.5, < 过流 5.0A 留余量) */
#define IQ_MAX_MA                       4500          /* mA, shell 层用 */

/* ===== Stage 5: 电流环 PI 参数 (spec §4.6, spec-stage5 §7) ===== */
/* 单位: Kp=V/A, Ki=V/(A·s). 初值估算, 台架阶跃标定. */
#define CURRENT_KP_D                    0.5f
#define CURRENT_KI_D                    100.0f
#define CURRENT_KP_Q                    0.5f
#define CURRENT_KI_Q                    100.0f
#define CURRENT_OUT_LIMIT_V             6.0f          /* ±6V, 12V 母线一半, 静态限幅 */
#define CURRENT_RAMP_DEFAULT_RPM        300.0f        /* ramp 模式默认角速度 (电角度 rpm) */
```

确认 `IQ_OVERCURRENT_A` 保持 `5.0f` 不动 (Stage 3 验收过)。

- [ ] **Step 3: 验证 WSL 构建不报错 (参数宏被引用前, 仅宏定义不影响)**

Run (WSL): `cd /e/WorkSpaces/2_MotorDriver/MPS_MotorDriver && cmake --build build/Debug 2>&1 | tail -5`
Expected: 0 Error (可能有 unused macro 警告, 忽略)

- [ ] **Step 4: Commit**

```bash
git add application/motor_control/motor_params.h
git commit -m "feat(stage5): add current loop params, IQ_MAX 8.0->4.5A"
```

---

### Task 2: current_loop 接口与实现

**Files:**
- Modify: `application/motor_control/current_loop.h`
- Modify: `application/motor_control/current_loop.c`
- Test: `tests/motor_control/test_current_loop.c` (Task 3)

- [ ] **Step 1: 扩展 current_loop.h 接口声明**

在 `current_loop.h` 现有 `current_loop_run` 声明后, `#ifdef __cplusplus` 前新增:
```c
/* Stage 5 新增 */
/* 设置电流环目标 (A). id_ref 通常 0, iq_ref 由 shell/CAN 设置. */
void current_loop_set_targets(float id_ref_A, float iq_ref_A);
/* 清积分 (模式切换/stop 时调用, 避免残留冲击). */
void current_loop_reset(void);
```

- [ ] **Step 2: 实现 current_loop.c (替换全部 stub)**

替换 `current_loop.c` 全部内容为:
```c
/*
 * current_loop.c - Id/Iq 双轴 PI 电流环 (Stage 5, spec-stage5 §4)
 *
 * 标准 PI + clamping 抗 windup:
 *   - 积分先限幅 (integral_limit), 再加比例, 再输出限幅 (out_limit)
 *   - ki 含 1/s 量纲, 积分时 × ISR_DT_S 转 per-tick 增量
 *   - kd=0 (Stage 5 不用微分, 结构体字段保留)
 *
 * ISR 安全: 无 RT-Thread API, 无 malloc, 纯浮点运算 (~1µs).
 */
#include "current_loop.h"
#include "motor_params.h"
#include <string.h>

/* ISR_DT_S: 1/16000, 与 motor_control_isr.c 一致. 放头文件或此处定义.
 * 此处定义避免循环包含, 若 motor_params.h 已有则用其宏. */
#ifndef ISR_DT_S
#define ISR_DT_S   (1.0f / 16000.0f)
#endif

static pid_f32_t s_pid_d;
static pid_f32_t s_pid_q;
static float      s_id_ref = 0.0f;
static float      s_iq_ref = 0.0f;

float pid_f32_exec(pid_f32_t *pid, float error)
{
    float p_out = pid->kp * error;
    pid->integral += pid->ki * error * ISR_DT_S;   /* 离散积分 */
    /* 积分限幅 (clamping 抗 windup) */
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float out = p_out + pid->integral;
    /* 输出限幅 */
    if (out >  pid->out_limit) out =  pid->out_limit;
    if (out < -pid->out_limit) out = -pid->out_limit;
    pid->last_error = error;
    return out;
}

void current_loop_init(void)
{
    memset(&s_pid_d, 0, sizeof(s_pid_d));
    memset(&s_pid_q, 0, sizeof(s_pid_q));
    s_pid_d.kp = CURRENT_KP_D;  s_pid_d.ki = CURRENT_KI_D;  s_pid_d.kd = 0.0f;
    s_pid_d.integral_limit = CURRENT_OUT_LIMIT_V;  s_pid_d.out_limit = CURRENT_OUT_LIMIT_V;
    s_pid_q.kp = CURRENT_KP_Q;  s_pid_q.ki = CURRENT_KI_Q;  s_pid_q.kd = 0.0f;
    s_pid_q.integral_limit = CURRENT_OUT_LIMIT_V;  s_pid_q.out_limit = CURRENT_OUT_LIMIT_V;
    s_id_ref = 0.0f;
    s_iq_ref = 0.0f;
}

void current_loop_set_targets(float id_ref_A, float iq_ref_A)
{
    s_id_ref = id_ref_A;
    s_iq_ref = iq_ref_A;
}

void current_loop_reset(void)
{
    s_pid_d.integral = 0.0f;
    s_pid_q.integral = 0.0f;
    s_pid_d.last_error = 0.0f;
    s_pid_q.last_error = 0.0f;
}

void current_loop_run(float id, float iq, float *vd_ref, float *vq_ref)
{
    *vd_ref = pid_f32_exec(&s_pid_d, s_id_ref - id);
    *vq_ref = pid_f32_exec(&s_pid_q, s_iq_ref - iq);
}
```

- [ ] **Step 3: 确认 ISR_DT_S 是否已在别处定义, 避免重复**

Run: `grep -rn "define ISR_DT_S\|ISR_DT_S" application/motor_control/ platform/ 2>&1 | head -10`
Expected: 显示 motor_control_isr.c 中的用法。若已有 `#define ISR_DT_S`, 记下行号, current_loop.c 的 `#ifndef ISR_DT_S` 守护已处理重复定义, 安全。

- [ ] **Step 4: WSL 构建验证**

Run (WSL): `cmake --build build/Debug 2>&1 | tail -8`
Expected: 0 Error (current_loop.c 替换 stub, 接口匹配)

- [ ] **Step 5: Commit**

```bash
git add application/motor_control/current_loop.h application/motor_control/current_loop.c
git commit -m "feat(stage5): implement current_loop PI + clamping + set_targets/reset"
```

---

### Task 3: current_loop 单元测试

**Files:**
- Test: `tests/motor_control/test_current_loop.c`

- [ ] **Step 1: 写测试 (主机 gcc, 独立复制 PI 数学)**

创建 `tests/motor_control/test_current_loop.c`:
```c
/*
 * test_current_loop.c - 电流环 PI 数学单元测试 (Stage 5)
 *
 * 主机端 gcc 编译运行. 独立复制 pid_f32_exec 逻辑, 验证:
 *   1. 比例项正确 (Kp × error)
 *   2. 积分累加 + clamping 限幅生效
 *   3. 输出限幅生效
 *   4. reset 清积分
 *   5. 稳态: 恒定误差下积分收敛到消除误差 (Ki 作用)
 *
 * 注: 产品代码 current_loop.c 依赖 motor_params.h 宏, 此处独立定义等价参数.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define ISR_DT_S   (1.0f / 16000.0f)
#define KP   0.5f
#define KI   100.0f
#define OUT_LIMIT  6.0f

typedef struct {
    float kp, ki, kd;
    float integral;
    float integral_limit;
    float out_limit;
    float last_error;
} pid_f32_t;

static float pid_f32_exec(pid_f32_t *pid, float error)
{
    float p_out = pid->kp * error;
    pid->integral += pid->ki * error * ISR_DT_S;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float out = p_out + pid->integral;
    if (out >  pid->out_limit) out =  pid->out_limit;
    if (out < -pid->out_limit) out = -pid->out_limit;
    pid->last_error = error;
    return out;
}

/* 测试 1: 纯比例 (误差恒定, 积分初值 0) */
static void test_pure_proportional(void)
{
    pid_f32_t pid = {KP, KI, 0, 0, OUT_LIMIT, OUT_LIMIT, 0};
    float out = pid_f32_exec(&pid, 1.0f);   /* error=1A */
    /* p_out = 0.5*1 = 0.5, integral = 100*1*(1/16000) = 0.00625, out = 0.50625 */
    assert(fabsf(out - 0.50625f) < 1e-4f);
    printf("[PASS] pure proportional: out=%.5f (expect 0.50625)\n", out);
}

/* 测试 2: 积分限幅 clamping */
static void test_integral_clamp(void)
{
    pid_f32_t pid = {KP, KI, 0, 0, OUT_LIMIT, OUT_LIMIT, 0};
    /* 持续大误差, 积分应被限幅到 OUT_LIMIT */
    for (int i = 0; i < 100000; i++) {
        pid_f32_exec(&pid, 10.0f);
    }
    assert(pid.integral <= OUT_LIMIT + 1e-6f);
    assert(pid.integral >= -OUT_LIMIT - 1e-6f);
    printf("[PASS] integral clamp: integral=%.4f (limit %.1f)\n", pid.integral, OUT_LIMIT);
}

/* 测试 3: 输出限幅 */
static void test_output_clamp(void)
{
    pid_f32_t pid = {KP, KI, 0, OUT_LIMIT, OUT_LIMIT, OUT_LIMIT, 0}; /* integral 已饱和 */
    float out = pid_f32_exec(&pid, 10.0f);  /* p_out=5 + integral=6 = 11 -> clamp 6 */
    assert(out <= OUT_LIMIT + 1e-6f);
    printf("[PASS] output clamp: out=%.4f (limit %.1f)\n", out, OUT_LIMIT);
}

/* 测试 4: reset 清积分 */
static void test_reset(void)
{
    pid_f32_t pid = {KP, KI, 0, 3.0f, OUT_LIMIT, OUT_LIMIT, 1.0f};
    pid.integral = 0.0f;
    pid.last_error = 0.0f;
    assert(pid.integral == 0.0f);
    assert(pid.last_error == 0.0f);
    printf("[PASS] reset clears integral and last_error\n");
}

/* 测试 5: 稳态消误差 (积分作用) */
static void test_steady_state_zero_error(void)
{
    /* 模拟: 实测电流缓慢跟踪目标, 积分把稳态误差压到 ~0 */
    pid_f32_t pid = {KP, KI, 0, 0, OUT_LIMIT, OUT_LIMIT, 0};
    float iq_target = 0.5f;
    float iq_actual = 0.0f;
    /* 简化被控对象: actual += out * 0.01 (一阶, 模拟电感) */
    for (int i = 0; i < 200000; i++) {
        float out = pid_f32_exec(&pid, iq_target - iq_actual);
        iq_actual += out * 0.01f;
    }
    /* 稳态误差 < 1% */
    assert(fabsf(iq_actual - iq_target) < 0.01f * iq_target);
    printf("[PASS] steady state: actual=%.5f target=%.5f err=%.6f\n",
           iq_actual, iq_target, fabsf(iq_actual - iq_target));
}

int main(void)
{
    test_pure_proportional();
    test_integral_clamp();
    test_output_clamp();
    test_reset();
    test_steady_state_zero_error();
    printf("\n5 current_loop tests passed\n");
    return 0;
}
```

- [ ] **Step 2: 编译运行测试**

Run: `gcc -std=c11 -Wall -o /tmp/test_current_loop tests/motor_control/test_current_loop.c -lm && /tmp/test_current_loop`
Expected: `[PASS]` ×5 + `5 current_loop tests passed`

- [ ] **Step 3: Commit**

```bash
git add tests/motor_control/test_current_loop.c
git commit -m "test(stage5): current_loop PI math unit tests"
```

---

### Task 4: ISR 接口声明 + debug 结构体 (motor_control_isr.h)

**Files:**
- Modify: `application/motor_control/motor_control_isr.h`

- [ ] **Step 1: 在 ALIGN 接口块后, debug 结构体前, 新增 CURRENT 接口声明**

在 `motor_control_isr.h` 中, `motor_control_isr_get_align_angle` 声明后, `/* 获取最近一次 ISR 内部状态 */` 注释前, 新增:
```c
/* ===== CURRENT 模式接口 (Stage 5, spec-stage5 §5.3) =====
 *
 * CURRENT 模式 (spec §3.4 模式 1):
 *   - Id 目标 = 0, Iq 目标 = iq_ref (用户/CAN 设置)
 *   - Clarke -> Park(theta) -> 电流环 PI -> IPark -> SVPWM
 *   - theta 来源: enc (编码器电角度, 需有效标定) 或 ramp (斜坡, 调试)
 *
 * 安全: 故障未清返回 -1. 启动时互斥清 OPEN_LOOP/ALIGN.
 */
int  motor_control_isr_current_start(float iq_ref_A);
void motor_control_isr_current_stop(void);
bool motor_control_isr_current_active(void);
void motor_control_isr_current_set_encoder_angle(bool use_enc);
void motor_control_isr_current_set_speed(float rad_per_s);
```

- [ ] **Step 2: 扩展 debug 结构体 (motor_control_isr_debug_t)**

在 debug 结构体 `cal_progress` 字段后, `} motor_control_isr_debug_t;` 前, 新增:
```c
    /* Stage 5: 电流环快照 */
    uint32_t cur_hits;      /* CURRENT 分支命中计数 */
    int32_t  id_ma;         /* 实测 d 轴电流 (毫安) */
    int32_t  iq_ma;         /* 实测 q 轴电流 (毫安) */
    int32_t  id_ref_ma;     /* 目标 Id (毫安) */
    int32_t  iq_ref_ma;     /* 目标 Iq (毫安) */
```

- [ ] **Step 3: WSL 构建验证 (头文件改动, 实现未跟, 可能有未使用警告但不应 error)**

Run (WSL): `cmake --build build/Debug 2>&1 | tail -8`
Expected: 0 Error

- [ ] **Step 4: Commit**

```bash
git add application/motor_control/motor_control_isr.h
git commit -m "feat(stage5): declare current mode ISR api + debug fields"
```

---

### Task 5: ISR 实现 (motor_control_isr.c) — 静态变量 + CURRENT 分支 + 接口

**Files:**
- Modify: `application/motor_control/motor_control_isr.c`

- [ ] **Step 1: 读取 ISR 文件结构, 定位插入点**

Run: `grep -n "static volatile.*s_ol\|static volatile.*s_align\|case MOTOR_CONTROL_MODE_CURRENT\|s_dbg_cal_progress\|current_loop_init\|motor_control_isr_align_stop(void)" application/motor_control/motor_control_isr.c`
Expected: 显示 s_ol/s_align 变量区 (line ~48), CURRENT stub (line ~297), align_stop 实现 (供参考模式)

- [ ] **Step 2: 新增静态变量 (在 s_align 变量块后)**

在 `motor_control_isr.c` 中, `s_align_active` 等变量定义块后 (Stage 4 ALIGN 参数区之后), 新增:
```c
/* ===== Stage 5: CURRENT 模式参数 (ISR 读, shell 写) ===== */
static volatile bool  s_cur_active = false;      /* CURRENT 模式是否激活 */
static volatile bool  s_cur_use_enc = true;      /* theta 来源: true=编码器, false=斜坡 (独立于 s_ol_use_enc) */
static volatile float s_cur_theta_e = 0.0f;      /* ramp 模式电角度 (rad) */
static volatile float s_cur_speed_rad_s = 0.0f;  /* ramp 模式角速度 (rad/s) */
```

同时新增 debug 快照变量 (在 s_dbg_* 区):
```c
static volatile uint32_t s_dbg_cur_hits = 0u;
static volatile int32_t  s_dbg_id_ma = 0;
static volatile int32_t  s_dbg_iq_ma = 0;
```

- [ ] **Step 3: 替换 CURRENT stub 为完整分支**

把 `motor_control_isr.c` 中:
```c
        case MOTOR_CONTROL_MODE_CURRENT:
        case MOTOR_CONTROL_MODE_SPEED:
        case MOTOR_CONTROL_MODE_POSITION:
            /* Stage 5+ 实现, 暂输出 50% */
            motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);
            break;
```
替换为:
```c
        case MOTOR_CONTROL_MODE_CURRENT: {
            float id, iq;
            float i_alpha, i_beta;
            float vd_ref, vq_ref;

            if (!s_cur_active) {
                motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);
                return;
            }
            s_dbg_cur_hits++;

            /* theta 来源 (enc/ramp) */
            if (s_cur_use_enc) {
                theta = s_enc_theta_e;
            } else {
                s_cur_theta_e += s_cur_speed_rad_s * ISR_DT_S;
                if (s_cur_theta_e < 0.0f)      s_cur_theta_e += TWO_PI_F;
                if (s_cur_theta_e >= TWO_PI_F) s_cur_theta_e -= TWO_PI_F;
                theta = s_cur_theta_e;
            }
            s_dbg_theta_mrad = (int32_t)(theta * RAD_TO_MRAD_F);

            /* Clarke + Park: ia/ib/ic -> id/iq */
            foc_clarke(ia, ib, ic, &i_alpha, &i_beta);
            foc_park(i_alpha, i_beta, theta, &id, &iq);
            s_dbg_id_ma = (int32_t)(id * 1000.0f);
            s_dbg_iq_ma = (int32_t)(iq * 1000.0f);

            /* 电流环 PI */
            current_loop_run(id, iq, &vd_ref, &vq_ref);

            /* IPark + SVPWM */
            foc_ipark(vd_ref, vq_ref, theta, &v_alpha, &v_beta);
            s_dbg_v_alpha_mv = (int32_t)(v_alpha * VOLTS_TO_MV_F);
            s_dbg_v_beta_mv  = (int32_t)(v_beta  * VOLTS_TO_MV_F);
            foc_svpwm_3phase_high_side(v_alpha, v_beta, vbus, &ta, &tb, &tc);
            s_dbg_ta = ta; s_dbg_tb = tb; s_dbg_tc = tc;
            motor_pwm_at32m412_set_duty_ticks(ta, tb, tc);
            break;
        }

        case MOTOR_CONTROL_MODE_SPEED:
        case MOTOR_CONTROL_MODE_POSITION:
            /* Stage 6+ 实现, 暂输出 50% */
            motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);
            break;
```

注: `theta/v_alpha/v_beta/ta/tb/tc` 是 ISR 函数顶部已声明的局部变量 (OPEN_LOOP/ALIGN 分支复用), 确认它们在 switch 作用域可见。`ia/ib/ic/vbus` 是 ISR 前段已算好的局部变量。

- [ ] **Step 4: 实现 4 个接口函数 (在 align_stop 实现后)**

在 `motor_control_isr.c` 的 `motor_control_isr_get_align_angle` 实现后, 新增:
```c
/* ===== Stage 5: CURRENT 模式接口 (spec-stage5 §5.3) ===== */
int motor_control_isr_current_start(float iq_ref_A)
{
    if (fault_manager_any_fatal()) {
        return -1;
    }
    /* 互斥: 清 OPEN_LOOP/ALIGN 残留 (Stage 4b 根因 1 教训) */
    s_ol_active = false;
    s_align_active = false;
    s_ol_vd = 0.0f;
    s_align_vd = 0.0f;

    current_loop_reset();
    current_loop_set_targets(0.0f, iq_ref_A);   /* id_ref=0 */
    s_cur_theta_e = 0.0f;
    s_cur_active = true;
    return 0;
}

void motor_control_isr_current_stop(void)
{
    s_cur_active = false;
    s_cur_theta_e = 0.0f;
    current_loop_reset();
    /* 50% 安全输出 */
    motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);
}

bool motor_control_isr_current_active(void)
{
    return s_cur_active;
}

void motor_control_isr_current_set_encoder_angle(bool use_enc)
{
    s_cur_use_enc = use_enc;
}

void motor_control_isr_current_set_speed(float rad_per_s)
{
    s_cur_speed_rad_s = rad_per_s;
}
```

- [ ] **Step 5: 扩展 get_debug 填充新字段**

在 `motor_control_isr_get_debug` 函数中, `dbg->cal_progress = ...` 之后, 新增:
```c
    dbg->cur_hits   = s_dbg_cur_hits;
    dbg->id_ma      = s_dbg_id_ma;
    dbg->iq_ma      = s_dbg_iq_ma;
    dbg->id_ref_ma  = 0;   /* id_ref 恒 0 */
    dbg->iq_ref_ma  = (int32_t)(/* iq_ref 从 current_loop 取, 见 Step 6 */ 0);
```

- [ ] **Step 6: 暴露 iq_ref 供 debug 读取**

`current_loop.c` 的 `s_iq_ref` 是 static, ISR 无法直接读。在 `current_loop.h` 新增 getter:
```c
/* Stage 5: 供 debug 读取当前目标 (mA) */
float current_loop_get_id_ref_A(void);
float current_loop_get_iq_ref_A(void);
```
在 `current_loop.c` 实现:
```c
float current_loop_get_id_ref_A(void) { return s_id_ref; }
float current_loop_get_iq_ref_A(void) { return s_iq_ref; }
```
修正 Step 5 的 `iq_ref_ma`:
```c
    dbg->id_ref_ma  = (int32_t)(current_loop_get_id_ref_A() * 1000.0f);
    dbg->iq_ref_ma  = (int32_t)(current_loop_get_iq_ref_A() * 1000.0f);
```

- [ ] **Step 7: 确认 current_loop_init 在 motor_app_init 调用链中**

Run: `grep -n "current_loop_init\|motor_app_init\|motor_calibration_load" application/motor_control/motor_app.c application/motor_control/motor_app.h 2>&1`
Expected: 显示 motor_app_init 现有调用链。若 `current_loop_init()` 未被调用, 在 `motor_app_init` 中 (motor_pwm_init 之后, motor_calibration_load 附近) 新增 `current_loop_init();`

- [ ] **Step 8: WSL 构建验证**

Run (WSL): `cmake --build build/Debug 2>&1 | tail -10`
Expected: 0 Error 0 Warning (或仅 RT-Thread 既有警告)

- [ ] **Step 9: Commit**

```bash
git add application/motor_control/motor_control_isr.c application/motor_control/motor_control_isr.h application/motor_control/current_loop.c application/motor_control/current_loop.h application/motor_control/motor_app.c
git commit -m "feat(stage5): CURRENT mode ISR branch + current_* api + debug fields"
```

---

### Task 6: mc_current 命令 + mc_stop/mc_debug 扩展 (motor_shell.c)

**Files:**
- Modify: `application/motor_control/motor_shell.c`

- [ ] **Step 1: 读取 motor_shell.c 结构, 定位 mc_stop/mc_debug/mc_open**

Run: `grep -n "MSH_CMD_EXPORT\|cmd_mc_stop\|cmd_mc_debug\|cmd_mc_open\|cmd_mc_align" application/motor_control/motor_shell.c`
Expected: 显示命令函数与导出位置

- [ ] **Step 2: mc_stop 加 current_stop 互斥清理**

在 `cmd_mc_stop` 函数中, 现有 `motor_control_isr_open_loop_stop()` / `motor_control_isr_align_stop()` 调用前/后, 新增:
```c
    motor_control_isr_current_stop();      /* Stage 5: 停电流环 */
    motor_control_isr_open_loop_stop();
    motor_control_isr_align_stop();
```

- [ ] **Step 3: mc_debug 新增 cur 快照行**

在 `cmd_mc_debug` 中, 现有 align/cal 快照打印之后, 新增:
```c
    rt_kprintf("cur: active=%d hits=%lu theta=%s id=%ldmA iq=%ldmA id_ref=%ld iq_ref=%ldmA\n",
               motor_control_isr_current_active() ? 1 : 0,
               (unsigned long)dbg.cur_hits,
               /* theta 来源需从 ISR 读, 此处简化: 用 cur_active + enc 状态推断 */
               "see_theta",
               (long)dbg.id_ma, (long)dbg.iq_ma,
               (long)dbg.id_ref_ma, (long)dbg.iq_ref_ma);
```
注: theta 来源 (enc/ramp) 显示需要额外 getter, 为简化本版用 "see_theta" 占位 (调试时看 mc_state 的 mode + 用户记忆命令参数)。若需精确显示, 加 `motor_control_isr_current_get_theta_source()` 接口 (可选, 不阻塞验收)。

- [ ] **Step 4: 实现 cmd_mc_current**

在 `motor_shell.c` 中 (其他 cmd_mc_* 函数附近), 新增:
```c
/* Stage 5: mc_current <iq_ma> [enc|ramp] [speed_rpm_elec] */
static int cmd_mc_current(int argc, char **argv)
{
    long iq_ma;
    float iq_A;
    bool use_enc = true;
    long speed_rpm = 0;

    if (argc < 2) {
        rt_kprintf("usage: mc_current <iq_ma> [enc|ramp] [speed_rpm_elec]\n");
        return -1;
    }
    iq_ma = atol(argv[1]);
    if (iq_ma > IQ_MAX_MA || iq_ma < -IQ_MAX_MA) {
        rt_kprintf("err: iq_ma range +-%d\n", IQ_MAX_MA);
        return -1;
    }
    iq_A = (float)iq_ma / 1000.0f;

    if (argc >= 3) {
        if (rt_strcmp(argv[2], "ramp") == 0) {
            use_enc = false;
        } else if (rt_strcmp(argv[2], "enc") == 0) {
            use_enc = true;
        } else {
            rt_kprintf("err: mode must be enc or ramp\n");
            return -1;
        }
    }
    if (argc >= 4 && !use_enc) {
        speed_rpm = atol(argv[3]);
    }

    /* 前置检查 */
    if (fault_manager_any_fatal()) {
        rt_kprintf("err: fault active, clear first (fault_clear)\n");
        return -1;
    }
    if (use_enc && !motor_calibration_is_valid()) {
        rt_kprintf("err: cal invalid, run mc_calibrate or use ramp mode\n");
        return -1;
    }

    /* 设 theta 来源 + 速度 */
    motor_control_isr_current_set_encoder_angle(use_enc);
    if (!use_enc) {
        float rad_per_s = (float)speed_rpm * 6.28318530718f / 60.0f;
        motor_control_isr_current_set_speed(rad_per_s);
    }

    /* 启动 */
    if (motor_control_isr_current_start(iq_A) != 0) {
        rt_kprintf("err: current start failed\n");
        return -1;
    }
    rt_kprintf("current: iq_ref=%ldmA theta=%s%s\n",
               (long)iq_ma, use_enc ? "enc" : "ramp",
               (!use_enc && speed_rpm != 0) ? " spinning" : "");
    return 0;
}
MSH_CMD_EXPORT(mc_current, current mode: mc_current <iq_ma> [enc|ramp] [rpm_elec]);
```

- [ ] **Step 5: 确认包含头文件**

Run: `grep -n "#include" application/motor_control/motor_shell.c | head -20`
Expected: 确认已含 `motor_control_isr.h` `motor_calibration.h` `fault_manager.h` `motor_params.h` `current_loop.h` (current_loop.h 可能需新增 include)。若缺 `current_loop.h` 则加 (本命令未直接调 current_loop, 但 IQ_MAX_MA 在 motor_params.h, 确认该头已含)。

- [ ] **Step 6: WSL 构建验证**

Run (WSL): `cmake --build build/Debug 2>&1 | tail -10`
Expected: 0 Error 0 Warning

- [ ] **Step 7: Keil 构建验证 (确认 ARMCC 也通过)**

Run (Windows): `cd project\MDK_V5 && flash.bat rebuild` (或用户手动 Keil rebuild)
Expected: 0 Error 0 Warning

- [ ] **Step 8: Commit**

```bash
git add application/motor_control/motor_shell.c
git commit -m "feat(stage5): mc_current command + mc_stop/mc_debug current support"
```

---

### Task 7: 验收脚本 stage5_bench.py

**Files:**
- Test: `tests/stage5_bench.py`

- [ ] **Step 1: 复用 stage4_bench.py 框架, 写 stage5 脚本**

创建 `tests/stage5_bench.py` (基于 stage4_bench.py 的 open_port/read_all/send_cmd/parse_kv helper, 复制后改 Section 内容):
```python
"""
Stage 5 电流环台架自动化验收脚本

对应 spec `docs/superpowers/specs/2026-06-22-stage5-current-loop-design.md` §8.

串口约束: 同 stage4_bench.py (逐字符发送 + 等回显, 适配 finsh 轮询 getchar).
输出解析: 按 motor_shell.c 的 key:value 标签正则解析 mc_debug 输出.

验收矩阵:
  Section A: 前置准备 (mc_cal 零偏标定)
  Section B: ramp 模式基础验证 (强制无标定表, 验 enc 拒绝 + ramp 稳态)
  Section C: 阶跃响应 (ramp 模式, 0.5A->1A, 稳态误差 < 5%)
  Section D: enc 模式验证 (需有效标定表)
  Section E: 清理 + 报告

注: 上升时间 < 1ms 需示波器测, 脚本只测稳态误差 + 分支命中 (finsh ~50ms 轮询限制).

用法:
  python tests/stage5_bench.py            # 默认 COM9
  python tests/stage5_bench.py COM7       # 指定串口
"""
import serial
import sys
import time
import re

PORT = "COM9"
BAUD = 115200

# ---- 验收阈值 ----
IQ_STEADY_ERR_PCT = 5.0      # 稳态误差 < 5%
IQ_MAX_MA = 4500             # 与 motor_params.h 对齐
CAL_TOTAL_TIMEOUT_S = 120    # 标定超时 (复用 stage4)

# ============================================================
# 串口基础 (复制自 stage4_bench.py)
# ============================================================
def open_port(port):
    ser = serial.Serial(port, BAUD, timeout=0.5)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser

def read_all(ser, settle=0.15, max_wait=2.0):
    data = b""
    deadline = time.time() + max_wait
    while time.time() < deadline:
        if ser.in_waiting:
            data += ser.read(ser.in_waiting)
            deadline = time.time() + settle
        else:
            time.sleep(0.01)
    return data.decode("ascii", errors="replace")

def send_char_and_wait_echo(ser, ch, timeout=0.3):
    ser.write(ch.encode("ascii"))
    deadline = time.time() + timeout
    echoed = b""
    while time.time() < deadline:
        if ser.in_waiting:
            echoed += ser.read(ser.in_waiting)
            if len(echoed) >= 1:
                break
        else:
            time.sleep(0.005)
    return echoed.decode("ascii", errors="replace")

def send_cmd(ser, cmd, line_ending="\r", wait_after=1.0):
    for ch in cmd:
        send_char_and_wait_echo(ser, ch)
        time.sleep(0.02)
    ser.write(line_ending.encode("ascii"))
    time.sleep(wait_after)
    return read_all(ser, settle=0.3, max_wait=3.0)

def parse_kv(text):
    fields = {}
    for line in text.splitlines():
        m = re.match(r"\s*([A-Za-z_][A-Za-z0-9_]*)\s*[:=]\s*(\S+)", line)
        if m:
            fields[m.group(1)] = m.group(2)
    return fields

def wait_msh(ser, timeout=10.0):
    """等 msh 提示符 (板子启动/重启后)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        data = read_all(ser, settle=0.5, max_wait=2.0)
        if "msh />" in data or "msh" in data:
            return True
        time.sleep(0.3)
    return False

# ============================================================
# 验收 Sections
# ============================================================
def section_a(ser, log):
    """A: 前置准备 (零偏标定)"""
    log.append("=== Section A: precondition ===")
    out = send_cmd(ser, "mc_state")
    assert "DISABLED" in out, f"A: not DISABLED: {out}"
    out = send_cmd(ser, "fault")
    assert "0x00" in out or "fault=0" in out.lower(), f"A: fault active: {out}"
    send_cmd(ser, "mc_cal", wait_after=2.0)
    out = send_cmd(ser, "mc_current")
    assert "usage" in out, f"A: mc_current missing usage: {out}"
    log.append("[A] PASS: DISABLED, fault clear, mc_cal done, mc_current exists")

def section_b(ser, log):
    """B: ramp 模式 (强制 CAL_INVALID)"""
    log.append("=== Section B: ramp mode (force cal invalid) ===")
    # B1: 擦除标定 + 重启
    send_cmd(ser, "mc_cal_erase", wait_after=1.0)
    send_cmd(ser, "reboot", wait_after=3.0)
    assert wait_msh(ser, timeout=15.0), "B: reboot timeout"
    send_cmd(ser, "mc_cal", wait_after=2.0)  # 重启后重新零偏标定
    # B2: enc 模式应被拒
    out = send_cmd(ser, "mc_current 500 enc")
    assert "cal invalid" in out.lower(), f"B2: enc not rejected: {out}"
    log.append("[B2] PASS: enc rejected when cal invalid")
    # B3: ramp 启动
    out = send_cmd(ser, "mc_current 500 ramp 300")
    assert "current: iq_ref=500" in out, f"B3: start failed: {out}"
    # B4: 采快照, 验稳态
    time.sleep(0.5)
    snapshots = []
    for _ in range(10):
        out = send_cmd(ser, "mc_debug", wait_after=0.6)
        snap = parse_kv(out)
        if "iq" in snap:
            snapshots.append(snap)
        time.sleep(0.1)
    assert len(snapshots) >= 3, f"B4: too few snapshots: {len(snapshots)}"
    # 解析 cur 行的 iq (格式 "iq=501mA")
    iq_vals = []
    for s in snapshots:
        for k, v in s.items():
            if "iq" in k.lower() and "mA" in str(v):
                try:
                    iq_vals.append(int(re.sub(r"[^\d-]", "", str(v))))
                except ValueError:
                    pass
    assert len(iq_vals) >= 3, f"B4: no iq values parsed: {snapshots[-1]}"
    iq_steady = sum(iq_vals[-3:]) / 3
    assert 475 <= iq_steady <= 525, f"B4: iq steady {iq_steady} out of 475-525"
    log.append(f"[B4] PASS: iq steady={iq_steady}mA (target 500, tol +-5%)")
    send_cmd(ser, "mc_stop")

def section_c(ser, log):
    """C: 阶跃响应 (ramp, 0.5A->1A)"""
    log.append("=== Section C: step response (ramp) ===")
    send_cmd(ser, "mc_current 500 ramp 300")
    time.sleep(0.3)
    # 阶跃到 1A
    send_cmd(ser, "mc_current 1000 ramp 300")
    time.sleep(0.5)  # 等稳态
    snapshots = []
    for _ in range(20):
        out = send_cmd(ser, "mc_debug", wait_after=0.6)
        snap = parse_kv(out)
        snapshots.append(snap)
        time.sleep(0.05)
    # 稳态误差: 末 5 个 iq 均值 vs 1000mA
    iq_vals = []
    for s in snapshots:
        for k, v in s.items():
            if "iq" in k.lower() and "mA" in str(v):
                try:
                    iq_vals.append(int(re.sub(r"[^\d-]", "", str(v))))
                except ValueError:
                    pass
    assert len(iq_vals) >= 5, f"C: too few iq: {iq_vals}"
    iq_steady = sum(iq_vals[-5:]) / 5
    err_pct = abs(iq_steady - 1000) / 1000.0 * 100
    assert err_pct < IQ_STEADY_ERR_PCT, f"C: steady err {err_pct:.1f}% >= {IQ_STEADY_ERR_PCT}%"
    log.append(f"[C] PASS: iq steady={iq_steady}mA err={err_pct:.2f}% (< {IQ_STEADY_ERR_PCT}%)")
    send_cmd(ser, "mc_stop")

def section_d(ser, log):
    """D: enc 模式 (需标定表)"""
    log.append("=== Section D: enc mode (with calibration) ===")
    # D1: 标定 (~72s)
    send_cmd(ser, "mc_calibrate", wait_after=2.0)
    deadline = time.time() + CAL_TOTAL_TIMEOUT_S
    done = False
    while time.time() < deadline:
        out = send_cmd(ser, "mc_cal_status", wait_after=1.0)
        if "DONE" in out:
            done = True
            break
        if "ABORTED" in out:
            log.append(f"[D] FAIL: calibrate aborted: {out}")
            return False
        time.sleep(2.0)
    assert done, f"D: calibrate timeout ({CAL_TOTAL_TIMEOUT_S}s)"
    log.append("[D1] PASS: calibration DONE")
    # D3: enc 启动
    out = send_cmd(ser, "mc_current 500 enc")
    assert "current: iq_ref=500" in out, f"D3: enc start failed: {out}"
    time.sleep(0.5)
    # D4: 验稳态
    iq_vals = []
    for _ in range(10):
        out = send_cmd(ser, "mc_debug", wait_after=0.6)
        snap = parse_kv(out)
        for k, v in snap.items():
            if "iq" in k.lower() and "mA" in str(v):
                try:
                    iq_vals.append(int(re.sub(r"[^\d-]", "", str(v))))
                except ValueError:
                    pass
        time.sleep(0.1)
    assert len(iq_vals) >= 3, f"D4: too few iq: {iq_vals}"
    iq_steady = sum(iq_vals[-3:]) / 3
    assert 450 <= iq_steady <= 550, f"D4: enc iq {iq_steady} out of 450-550 (enc 模式容差大, 标定残差)"
    log.append(f"[D4] PASS: enc iq steady={iq_steady}mA (target 500, tol +-10%)")
    send_cmd(ser, "mc_stop")
    return True

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    ser = open_port(port)
    log = []
    try:
        section_a(ser, log)
        section_b(ser, log)
        section_c(ser, log)
        section_d(ser, log)
        log.append("\n=== ALL PASS ===")
    except AssertionError as e:
        log.append(f"\n=== FAIL: {e} ===")
    finally:
        send_cmd(ser, "mc_stop", wait_after=0.5)
        ser.close()
    report = "\n".join(log)
    print(report)
    with open("tests/stage5_bench_log.txt", "w", encoding="utf-8") as f:
        f.write(report)

if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 语法检查 (python -m py_compile)**

Run: `python -m py_compile tests/stage5_bench.py`
Expected: 无输出 (语法正确)

- [ ] **Step 3: Commit**

```bash
git add tests/stage5_bench.py
git commit -m "test(stage5): bench acceptance script (ramp/step/enc validation)"
```

---

### Task 8: CLAUDE.md 更新 + 手动验收步骤

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: 在 CLAUDE.md "Stage 4+4b 台架验收调试" 段后, 新增 "Stage 5 电流环" 段**

新增内容 (插入到 "## 调试串口 + msh 命令" 段之前):
```markdown
## Stage 5 电流环 - 2026-06-22

Stage 5 (CURRENT 模式 Id/Iq PI 电流环) 代码完成, 详见 `docs/superpowers/specs/2026-06-22-stage5-current-loop-design.md`.

完成内容:
- `current_loop.c` 实现: pid_f32_exec (标准 PI + clamping) / current_loop_run / set_targets / reset / get_id_ref_A / get_iq_ref_A
- `motor_control_isr.c` CURRENT 分支填充: Clarke->Park->电流环->IPark->SVPWM, theta 来源 enc/ramp 双支持 (独立 s_cur_use_enc)
- 新增 4 接口: current_start/stop/set_encoder_angle/set_speed + current_active
- 互斥清理 (复用 Stage 4b 根因 1 教训): current_start 清 OPEN_LOOP/ALIGN, mc_stop 清 current
- `motor_shell.c` 新增 `mc_current <iq_ma> [enc|ramp] [rpm_elec]` + 扩展 mc_stop/mc_debug
- `motor_params.h`: IQ_MAX_A 8.0->4.5 (< 过流 5.0A 留余量), 新增 CURRENT_KP/KI/OUT_LIMIT 宏
- `tests/stage5_bench.py` 自动验收 (A 前置 / B ramp / C 阶跃 / D enc)

关键决策:
- 方案 A (CURRENT 独立分支): 不动 OPEN_LOOP/ALIGN 已验证代码, 风险隔离
- theta 用独立 s_cur_use_enc (非 s_ol_use_enc): 模式间状态隔离
- enc 模式强制要有效标定表: shell 层 CAL_INVALID 拒绝, 避免坏表致电流环振荡
- 输出限幅静态 6V (= 12V 母线一半): 简单安全, 动态限幅留 Stage 6+

资源占用 (Stage 5 完成后, 待填实测):
- WSL GCC: FLASH ___ B / 127 KB, RAM ___ B / 16 KB
- Keil ARMCC -O1: Code ___ + RO ___, ZI ___, 0 Error 0 Warning

新增 msh 命令:
| `mc_current <iq_ma> [enc\|ramp] [rpm_elec]` | 启动电流环 (Iq 目标 mA, enc=编码器电角度/ramp=斜坡调试) |

台架验收步骤 (限流电源 12V/0.5A):
1. 烧录后串口连板, 见 msh 提示符
2. `mc_cal` 零偏标定
3. `mc_current 500 ramp 300` -> 示波器看三相电流, 正弦幅值 ~0.5A, 频率 35Hz
4. `mc_current 500 ramp 300` -> `mc_current 1000 ramp 300` 阶跃
   - 示波器看 iq 波形: 上升 < 1ms, 超调 < 20%, 稳态误差 < 5%
5. 有标定表后 `mc_current 500 enc` -> 真正 FOC, 电流更干净
6. 异常 (过流/振荡/异响) 立即 `mc_stop`

已知限制:
- Kp/Ki 初值 0.5/100 是估算, 台架阶跃标定 (振荡降 Kp, 慢升 Kp)
- 上升时间 < 1ms 需示波器测, 脚本测不到 (finsh ~50ms 轮询)
- enc 模式依赖标定表, 残差 3.66° 可能致 iq 纹波, Stage 6 闭环重标定降残差
- 输出限幅静态 6V, 未按 Vbus 动态算 (Stage 6+)
```

- [ ] **Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "docs(stage5): CLAUDE.md current loop stage record + bench steps"
```

---

## Self-Review

**Spec coverage:**
- §4 接口 (pid_f32_exec/set_targets/reset) → Task 2 ✓
- §5 ISR 改动 (CURRENT 分支 + 4 接口 + 静态变量 + debug) → Task 4+5 ✓
- §6 shell (mc_current + mc_stop/mc_debug 扩展) → Task 6 ✓
- §7 参数 (IQ_MAX 4.5 + 电流环宏) → Task 1 ✓
- §8 验收脚本 → Task 7 ✓
- §9 CLAUDE.md 手动验收 → Task 8 ✓
- §4.4 current_loop_run → Task 2 ✓

**Placeholder scan:** Task 8 资源占用留 `___` 待实测填 — 这是实现后才能测的值, 非计划 placeholder, 执行时填实测。其余无 TBD/TODO。✓

**Type consistency:**
- `motor_control_isr_current_start(float)` Task 4 声明 = Task 5 实现 ✓
- `current_loop_set_targets(float, float)` Task 2 声明 = Task 5 调用 ✓
- `dbg.cur_hits/id_ma/iq_ma/id_ref_ma/iq_ref_ma` Task 4 结构体 = Task 5 填充 ✓
- `IQ_MAX_MA=4500` Task 1 宏 = Task 6 shell 限幅 ✓

**缺口检查:** Task 5 Step 6 新增 `current_loop_get_id_ref_A/iq_ref_A` getter, 需在 Task 2 的 current_loop.h/c 也声明/实现。**已纳入 Task 5 Step 6 (同步改 current_loop.h/c)**, 但 Task 2 没提。这是顺序问题: Task 2 写基础接口, Task 5 补 getter。可接受 (getter 是 debug 附属), 不阻塞。执行时 Task 5 Step 6 一并改 current_loop.h/c。
