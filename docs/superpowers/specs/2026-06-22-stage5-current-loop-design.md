# Stage 5: 电流环控制 — 设计规格

- **日期**: 2026-06-22
- **状态**: 设计已批准, 待写实现计划
- **关联**: `docs/superpowers/specs/2026-06-22-mps-foc-design.md` §7 Stage 5, §4.6 PID 参数, §3.1 主循环
- **前置**: Stage 4+4b 已完成 (台架标定残差 3.66°, 可作 Stage 5 开环过渡方案)

## 1. 目标

实现 CURRENT 模式 (spec §3.4 模式 ID 1): Id/Iq 双轴 PI 电流环, 输入 `iq_ref_mA`, Id 目标固定 0, 电流环 PI 输出 Vd/Vq, 经 IPark + SVPWM 驱动三相 PWM。

验收标准 (spec Stage 5):
- `iq_ref` 阶跃 0.5A→1A, 上升时间 < 1ms (示波器测, 脚本测不到)
- 稳态误差 < 5% (脚本可测)
- 电流环分支命中 (cur_hits 递增)

## 2. 范围边界

**包含**:
- `current_loop.c` 实现 (pid_f32_exec + current_loop_run + set_targets + reset)
- `motor_control_isr.c` CURRENT 分支填充 + 新增 4 个接口 + 静态变量
- `motor_shell.c` 新增 `mc_current` 命令 + 扩展 `mc_stop`/`mc_debug`
- `motor_params.h` 新增电流环参数 + 修改 IQ_MAX_A
- `tests/stage5_bench.py` 自动验收脚本

**不包含** (Stage 6+):
- 速度环 (SPEED 模式仍 stub)
- 位置环
- 运行时动态输出限幅 (按 Vbus 实时算, 当前用静态 6V)
- 电压前馈 / MTPA / 弱磁

**不动** (已台架验证):
- OPEN_LOOP 分支 (Stage 2)
- ALIGN 分支 (Stage 4)
- 编码器读取段 (Stage 4, line 201-230, 所有 ENABLED 模式共享)
- 电流采样段 (Stage 3, line 138-168)

## 3. 方案选择: CURRENT 分支独立流水线 (方案 A)

三方案对比后选 A:

| 方案 | 描述 | 决策 |
|---|---|---|
| A. CURRENT 独立分支 | CURRENT 自跑 Clarke→Park→PI→IPark→SVPWM, OPEN_LOOP/ALIGN 不动 | **选** |
| B. 统一流水线 | 提到 switch 外, OPEN_LOOP 也改走电流环 | 否 (改已验证行为, 风险高) |
| C. 提取共享 helper | 封装 foc_voltage_pipeline, 各模式逐步改用 | 否 (过度设计, 范围扩散) |

理由: 风险隔离 (不动 Stage 2-4 已验证代码), 范围聚焦 (只动 CURRENT + current_loop), ramp 调试模式代价小 (复制 5 行斜坡 theta)。

## 4. 接口设计

### 4.1 `current_loop.h` (扩展现有骨架)

```c
/* 现有 (不动) */
typedef struct {
    float kp, ki, kd;
    float integral;
    float integral_limit;
    float out_limit;
    float last_error;
} pid_f32_t;

float pid_f32_exec(pid_f32_t *pid, float error);
void current_loop_init(void);
void current_loop_run(float id, float iq, float *vd_ref, float *vq_ref);

/* Stage 5 新增 */
void current_loop_set_targets(float id_ref_A, float iq_ref_A);
void current_loop_reset(void);
```

### 4.2 `current_loop.c` 内部状态

```c
static pid_f32_t s_pid_d;
static pid_f32_t s_pid_q;
static float      s_id_ref = 0.0f;
static float      s_iq_ref = 0.0f;
```

### 4.3 `pid_f32_exec` 实现 (标准 PI + clamping)

```c
float pid_f32_exec(pid_f32_t *pid, float error)
{
    float p_out = pid->kp * error;
    pid->integral += pid->ki * error * ISR_DT_S;   /* 离散积分, ki 含 1/s 量纲 */
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float out = p_out + pid->integral;
    if (out >  pid->out_limit) out =  pid->out_limit;
    if (out < -pid->out_limit) out = -pid->out_limit;
    pid->last_error = error;
    return out;
}
```

- `ki` 单位 V/(A·s), `× ISR_DT_S` (1/16000) 转每 tick 增量
- clamping: 积分先限幅, 再加比例, 再输出限幅 (双重保护)
- `kd=0`, 字段保留

### 4.4 `current_loop_run` 实现

```c
void current_loop_run(float id, float iq, float *vd_ref, float *vq_ref)
{
    *vd_ref = pid_f32_exec(&s_pid_d, s_id_ref - id);
    *vq_ref = pid_f32_exec(&s_pid_q, s_iq_ref - iq);
}
```

## 5. ISR 改动 (`motor_control_isr.c`)

### 5.1 新增静态变量

```c
static volatile bool  s_cur_active = false;
static volatile bool  s_cur_use_enc = true;      /* 独立于 s_ol_use_enc, 模式隔离 */
static volatile float s_cur_theta_e = 0.0f;
static volatile float s_cur_speed_rad_s = 0.0f;
```

### 5.2 CURRENT 分支填充 (替换 line 297-302 stub)

```c
case MOTOR_CONTROL_MODE_CURRENT: {
    float id, iq;
    float i_alpha, i_beta;
    float vd_ref, vq_ref;

    if (!s_cur_active) {
        motor_pwm_at32m412_set_duty_ticks(TMR1_ARR/2u, TMR1_ARR/2u, TMR1_ARR/2u);
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

    /* Clarke + Park */
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
```

输入变量来源 (均已就绪, 不新算):
- `ia/ib/ic` (A) — Stage 3, line 163-165
- `vbus` — Stage 3, line 158
- `s_enc_theta_e` — Stage 4, line 208

**与 ALIGN 的互斥说明**: 编码器读取段 (line 201-230) 在 switch 之前, 所有 ENABLED 模式共享。其中 ALIGN 采样累加 (line 212-218) 受 `s_align_active` 保护, CURRENT 模式下 `s_align_active=false` (current_start 已清), 不会误采 ALIGN 数据。

### 5.3 新增接口函数

```c
int  motor_control_isr_current_start(float iq_ref_A);
void motor_control_isr_current_stop(void);
void motor_control_isr_current_set_encoder_angle(bool use_enc);
void motor_control_isr_current_set_speed(float rad_per_s);
```

`current_start`:
```c
int motor_control_isr_current_start(float iq_ref_A)
{
    if (fault_manager_any_fatal()) return -1;
    /* 互斥: 清 OPEN_LOOP/ALIGN 残留 (Stage 4b 根因 1 教训) */
    s_ol_active = false;
    s_align_active = false;
    s_ol_vd = 0.0f;
    s_align_vd = 0.0f;

    current_loop_reset();
    current_loop_set_targets(0.0f, iq_ref_A);  /* id_ref=0 */
    s_cur_theta_e = 0.0f;
    s_cur_active = true;
    return 0;
}
```

`current_stop`:
```c
void motor_control_isr_current_stop(void)
{
    s_cur_active = false;
    s_cur_theta_e = 0.0f;
    current_loop_reset();
    motor_pwm_at32m412_set_duty_ticks(TMR1_ARR/2u, TMR1_ARR/2u, TMR1_ARR/2u);
}
```

### 5.4 debug 结构体扩展

新增字段: `cur_hits`, `id_ma`, `iq_ma`, `id_ref_ma`, `iq_ref_ma`

`mc_debug` 输出行:
```
cur: active=1 hits=12345 theta=enc id=12mA iq=501mA id_ref=0 iq_ref=500mA
```

### 5.5 mc_stop 互斥清理

`motor_shell.c` 的 `mc_stop` 命令加 `motor_control_isr_current_stop()`:
```c
motor_control_isr_current_stop();   /* 新增 */
motor_control_isr_open_loop_stop();
motor_control_isr_align_stop();
```

## 6. shell 命令

### 6.1 新增 `mc_current`

```
mc_current <iq_ma> [enc|ramp] [speed_rpm_elec]
```

- 启动 CURRENT 模式, Iq 目标 = `iq_ma` mA, Id 目标 = 0
- 第二参数: `enc` (默认, 编码器电角度) / `ramp` (斜坡, 调试)
- 第三参数: 仅 ramp 有效, 斜坡角速度 (电角度 rpm), 默认 0

**前置检查** (shell 层):
- `fault_manager_any_fatal()` → 拒绝
- `ramp` 模式: 直接允许
- `enc` 模式: `!motor_calibration_is_valid()` → 拒绝, 提示 "cal invalid, run mc_calibrate or use ramp mode"

**参数限幅**: `iq_ma` 范围 ±`IQ_MAX_MA` (±4500)

### 6.2 使用场景

| 场景 | 命令 | 用途 |
|---|---|---|
| 无标定表验证电流环 | `mc_current 500 ramp 300` | 0.5A, 斜坡 300rpm, 验 PI 响应 |
| 有标定表真实 FOC | `mc_current 500 enc` | 0.5A, 编码器电角度 |
| 阶跃测试 | `mc_current 500 enc` → `mc_current 1000 enc` | 0.5A→1A 阶跃 |
| 停止 | `mc_stop` | 停电流环 + 关 MP6540H |

## 7. 参数 (`motor_params.h`)

```c
/* ===== Stage 5: 电流环参数 (spec §4.6) ===== */
#define CURRENT_KP_D                0.5f          /* V/A */
#define CURRENT_KI_D                100.0f        /* V/(A·s) */
#define CURRENT_KP_Q                0.5f
#define CURRENT_KI_Q                100.0f
#define CURRENT_OUT_LIMIT_V         6.0f          /* ±6V, 12V 母线一半, 静态限幅 */

#define IQ_MAX_A                    4.5f          /* 电流环目标上限 (改: 8.0→4.5) */
#define IQ_MAX_MA                   4500          /* mA, shell 层用 */
#define IQ_OVERCURRENT_A            5.0f          /* 过流保护 (不动, Stage 3 验收过) */

#define CURRENT_RAMP_DEFAULT_RPM    300.0f        /* ramp 模式默认角速度 */
```

**阈值关系**: `IQ_MAX (4.5A, shell 限幅) < IQ_OVERCURRENT (5.0A, ISR 保护)`, 留 0.5A 动态余量, 电流环超调不误触发过流。

`PID_SPEED_INTEGRAL_LIMIT` / `PID_SPEED_OUT_LIMIT` 引用 `IQ_MAX_A`, 现变 4.5A, 不影响 Stage 5 (速度环 Stage 6 才用), Stage 6 时 4.5A 对 2808 电机合理。

### `current_loop_init` 实现

```c
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
```

## 8. 验收脚本 `tests/stage5_bench.py`

复用 `stage4_bench.py` 框架 (串口自动化, 逐字符发送+等回显)。

### 流程

```
Section A: 前置准备
  A1. 连串口, 等 msh 提示符
  A2. mc_state → DISABLED, fault=0
  A3. mc_cal → 零偏标定
  A4. mc_current (无参) → 确认 usage 提示

Section B: ramp 模式基础验证 (强制无标定表)
  B1. mc_cal_erase + reboot → CAL_INVALID
  B2. mc_current 500 enc → 断言被拒 (cal invalid)
  B3. mc_current 500 ramp 300 → 启动
  B4. 采 10 个 mc_debug → 断言: cur_hits 递增, iq_ma 475-525mA, id_ma ±50mA
  B5. mc_stop

Section C: 阶跃响应 (ramp 模式)
  C1. mc_current 500 ramp 300 → 稳态
  C2. 等 200ms
  C3. mc_current 1000 ramp 300 → 阶跃
  C4. 采 20 个 mc_debug (间隔 5ms, 共 100ms)
  C5. 算稳态误差: 末 5 个 iq_ma 均值 vs 1000mA, 断言 < 5% (950-1050mA)
  C6. mc_stop
  注: 上升时间 < 1ms 需示波器测, 脚本测不到 (finsh ~50ms/次轮询)

Section D: enc 模式验证 (需标定表)
  D1. mc_calibrate (~72s)
  D2. 确认 cal_valid=1
  D3. mc_current 500 enc → 启动
  D4. 采 10 个 mc_debug → 断言 iq_ma 接近 500mA, 无振荡
  D5. mc_stop

Section E: 清理
  E1. mc_stop
  E2. 保存 debug 快照到 stage5_bench_log.txt
  E3. 打印 PASS/FAIL 汇总
```

### 关键约束

- **上升时间测量受限**: finsh getchar 轮询 ~50ms/次, 采不到 1ms 上升过程。脚本只测稳态误差 (< 5%) 和分支命中。1ms 上升时间写进 CLAUDE.md 手动验收 (示波器看 iq 波形)。
- **B 段强制 CAL_INVALID**: `mc_cal_erase + reboot` 确保无标定表, 验证 ramp + enc 拒绝逻辑。reboot 后脚本需重新连串口 + 等 msh 提示符。D 段再标定验证 enc。
- **超时与诊断**: 每段 while 设超时 (B/C 30s, D 120s), 超时路径保存 debug 快照 (Stage 4b 根因 3 教训)。

## 9. CLAUDE.md 补充手动验收步骤

```
1. 烧录后串口连板, 限流电源 12V/0.5A
2. mc_cal 零偏标定
3. mc_current 500 ramp 300 → 示波器看三相电流
   - 正弦, 幅值 ~0.5A, 频率 = 300/60 × 7 = 35Hz
4. mc_current 500→1000 ramp 300 阶跃
   - 示波器看 iq 波形: 上升 < 1ms, 超调 < 20%, 稳态误差 < 5%
5. 有标定表后 mc_current 500 enc → 真正 FOC, 电流更干净
6. 异常 (过流/振荡/异响) 立即 mc_stop
```

## 10. 已知限制与风险

| 项 | 说明 | 缓解 |
|---|---|---|
| Kp/Ki 初值 0.5/100 是估算 | 基于估算 R=1Ω/L=1mH, 未实测 | 台架阶跃测试标定, 振荡降 Kp, 慢升 Kp |
| 上升时间脚本测不到 | finsh 轮询 ~50ms | 示波器手动测, 写 CLAUDE.md |
| enc 模式依赖标定表 | 残差 3.66° 可能致 iq 纹波 | shell 层 CAL_INVALID 拒绝; Stage 6 闭环重标定降残差 |
| 输出限幅静态 6V | 未按 Vbus 动态算 | 12V 母线下 6V=Vbus/2 安全; Stage 6+ 动态限幅 |
| CURRENT 的 ramp 模式复制斜坡逻辑 | 5 行重复 | 可接受, 避免 Stage 2-4 回归风险 |

## 11. 资源占用预估

- `current_loop.c`: ~60 行实现 (替换 18 行 stub, 净 +42 行)
- `motor_control_isr.c`: CURRENT 分支 ~30 行 + 4 接口 ~40 行 + 静态变量 4 行
- `motor_shell.c`: mc_current ~50 行 + mc_stop/mc_debug 扩展 ~10 行
- 预估 FLASH +1.5KB, RAM +32B (pid_f32_t ×2 = 28B + 目标值 8B)

## 12. 后续 Stage 衔接

- **Stage 6 速度环**: `speed_loop_run` 输出 `iq_ref` 给电流环, 复用 `current_loop_set_targets`
- **Stage 6 重标定**: 闭环速度环恒速重跑 mc_calibrate, 降残差到 < 1°
- **Stage 7 位置环**: `position_loop_run` 输出 `rpm_ref` 给速度环
- **Stage 8 CAN**: CAN 帧 `iq_ref_mA` 直接喂 `mc_current` 同接口
