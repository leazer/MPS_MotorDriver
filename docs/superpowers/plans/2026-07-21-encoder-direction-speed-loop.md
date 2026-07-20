# Encoder Direction and Speed Loop Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Normalize the MA600A control direction, recover the cascaded speed/current loop, and measure and tune speed-loop behavior on the connected V2 motor.

**Architecture:** Preserve raw encoder and calibration-table semantics, but apply one `MOTOR_ENCODER_DIRECTION=-1` constant when converting sensor position to control electrical angle and sensor motion to control speed. Keep CURRENT mode's 1.5A command range unchanged while initially limiting SPEED mode to 0.5A. Add a compact serial status line and an automated Stage 6 bench so direction, steady error, current, sampling quality, and safe shutdown are measured repeatably.

**Tech Stack:** ARMCC V5.06 C90-compatible embedded C, Python 3.14 + pyserial host tests, WSL GCC host executable tests, Keil MDK build/flash, RT-Thread msh on COM9.

## Global Constraints

- Preserve V2 phase mapping U/V/W = TMR1 CH3/CH2/CH1.
- Preserve the fixed ADC trigger `CCR4=5264`, 180-tick blanking, reconstruction logic, and low-side current polarity normalization.
- Preserve `raw16`, `corrected_raw16`, `raw_unwrapped`, and `corrected_unwrapped` as hardware-direction diagnostics.
- Apply the same direction to calibrated/raw electrical angle and mechanical/electrical speed.
- Keep `IQ_MAX_A=1.5f`, current software protection at 2.0A, and current PI output at ±2V.
- Limit SPEED mode to `SPEED_IQ_LIMIT_A=0.5f` until a later, separately verified expansion.
- Do not stage or overwrite the user-owned `tests/stage5_bench_log.txt` change.
- Every powered test ends with `mc_stop` and assertions for state DISABLED, EN=LOW, three 2812 PWM duties, `CCR4=5264`, and no fatal fault.

---

### Task 1: Normalize Encoder Control Direction

**Files:**
- Create: `tests/encoder_service/test_encoder_direction.c`
- Create: `tests/encoder_service/board_motor_pins.h`
- Create: `tests/encoder_service/encoder_acq_timer_at32m412.h`
- Modify: `application/motor_control/motor_params.h`
- Modify: `application/motor_control/encoder_service.c`
- Modify: `tests/encoder_service/test_encoder_service_static.py`

**Interfaces:**
- Consumes: `encoder_service_init`, `encoder_service_set_zero`, `encoder_service_set_calibration_table`, `encoder_service_update_sample`, `encoder_service_get_snapshot`.
- Produces: `MOTOR_ENCODER_DIRECTION (-1)` and normalized `raw_elec_mrad`, `elec_mrad`, `speed_mech_mrad_s`, and `speed_elec_mrad_s`.

- [ ] **Step 1: Write the failing behavior test**

Create a host test that supplies zero calibration, sets `zero_raw=1000`, and feeds decreasing raw counts. Stub the four target-only collaborators and assert:

```c
void encoder_tracker_update_sample(uint16_t raw16, uint8_t valid)
{ (void)raw16; (void)valid; }
void motor_control_isr_on_encoder_sample(uint16_t raw16) { (void)raw16; }
int motor_encoder_read_raw_frame(uint16_t *raw, int16_t *speed)
{ (void)raw; (void)speed; return -1; }
int motor_encoder_read_angle_raw(uint16_t *raw) { (void)raw; return -1; }

encoder_service_init();
encoder_service_set_zero(1000u);
encoder_service_set_calibration_table(zero_table, true);
assert(encoder_service_update_sample(900u, 0, 1u) == 0);
assert(encoder_service_get_snapshot(&snap));
assert(snap.raw_elec_mrad >= 66 && snap.raw_elec_mrad <= 68);
assert(snap.elec_mrad == snap.raw_elec_mrad);

for (i = 0; i < 40; ++i) {
    assert(encoder_service_update_sample((uint16_t)(900 - i * 10), 0, 1u) == 0);
}
assert(encoder_service_get_snapshot(&snap));
assert(snap.speed_mech_mrad_s > 0);
assert(snap.speed_elec_mrad_s == snap.speed_mech_mrad_s * 7);
```

Reset with `zero_raw=10`, feed `raw=65530`, and assert the normalized electrical angle is a small positive value to cover wrap-around.

- [ ] **Step 2: Run the test to verify RED**

Run:

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/current-sampling-reconstruction; gcc -std=c11 -Wall -Wextra -Werror -Itests/encoder_service -Iapplication/motor_control tests/encoder_service/test_encoder_direction.c application/motor_control/encoder_service.c -o /tmp/test_encoder_direction; /tmp/test_encoder_direction"
```

Expected: assertion failure because decreasing raw counts currently publish negative speed and `corrected-zero` produces the opposite electrical angle.

- [ ] **Step 3: Implement the minimal direction normalization**

Add to `motor_params.h`:

```c
/* MA600A raw decreases when the verified positive electrical field rotates. */
#define MOTOR_ENCODER_DIRECTION        (-1)
```

In `encoder_service.c`, make position conversion select its unsigned zero-relative difference by direction:

```c
if (MOTOR_ENCODER_DIRECTION < 0) {
    mech_diff = (uint16_t)(s_zero_raw - position);
} else {
    mech_diff = (uint16_t)(position - s_zero_raw);
}
```

Apply the same constant to window speed before publication:

```c
mech_speed *= (int32_t)MOTOR_ENCODER_DIRECTION;
s_snapshot.speed_mech_mrad_s = mech_speed;
s_snapshot.speed_elec_mrad_s = mech_speed * (int32_t)MOTOR_POLE_PAIRS;
```

Keep raw/corrected/unwrapped fields unchanged.

- [ ] **Step 4: Strengthen the static contract and verify GREEN**

Require `#define MOTOR_ENCODER_DIRECTION` in `motor_params.h`, require both position branches and the speed multiplication in `encoder_service.c`, then run the host C test and `python tests/encoder_service/test_encoder_service_static.py`.

Expected: both exit 0.

- [ ] **Step 5: Commit the direction fix**

```powershell
git add -- application/motor_control/motor_params.h application/motor_control/encoder_service.c tests/encoder_service/test_encoder_direction.c tests/encoder_service/board_motor_pins.h tests/encoder_service/encoder_acq_timer_at32m412.h tests/encoder_service/test_encoder_service_static.py
git commit -m "fix: normalize encoder control direction"
```

### Task 2: Add a Speed-Mode Current Safety Limit

**Files:**
- Modify: `application/motor_control/motor_params.h`
- Modify: `application/motor_control/speed_loop.c`
- Modify: `tests/motor_control/test_speed_loop.c`
- Modify: `tests/motor_control/test_speed_loop_static.py`

**Interfaces:**
- Consumes: existing speed PI and `IQ_MAX_A=1.5f` used by CURRENT mode.
- Produces: speed-only integral/output clamp `SPEED_IQ_LIMIT_A=0.5f`.

- [ ] **Step 1: Change the host test expectation before production code**

Set the replica's speed limit to 0.5A and assert both positive and negative long-duration errors clamp exactly at ±0.5A:

```c
#define SPEED_IQ_LIMIT_A             0.5f
#define PID_SPEED_INTEGRAL_LIMIT     SPEED_IQ_LIMIT_A
#define PID_SPEED_OUT_LIMIT          SPEED_IQ_LIMIT_A
```

Add a negative saturation loop and `assert(fabsf(iq + SPEED_IQ_LIMIT_A) < 1e-5f);`.

- [ ] **Step 2: Verify RED static contract**

Run `python tests/motor_control/test_speed_loop_static.py` after adding assertions that production parameters contain `SPEED_IQ_LIMIT_A` and both speed limit macros reference it.

Expected: FAIL because the production parameters still reference `IQ_MAX_A`.

- [ ] **Step 3: Implement the speed-only limit**

Replace the speed limit definitions with:

```c
#define SPEED_IQ_LIMIT_A               0.5f
#define PID_SPEED_INTEGRAL_LIMIT        SPEED_IQ_LIMIT_A
#define PID_SPEED_OUT_LIMIT             SPEED_IQ_LIMIT_A
```

No speed-loop algorithm change is required because `speed_loop_init()` already loads those macros.

- [ ] **Step 4: Run focused tests and commit**

Run the static test and compile `test_speed_loop.c` with WSL GCC. Expected: 6 speed-loop tests pass and both saturation signs report 0.5A.

```powershell
git add -- application/motor_control/motor_params.h tests/motor_control/test_speed_loop.c tests/motor_control/test_speed_loop_static.py
git commit -m "safety: limit initial speed loop current"
```

### Task 3: Add Compact Speed Telemetry and Stage 6 Bench

**Files:**
- Modify: `application/motor_shell.c`
- Create: `tests/stage6_bench.py`
- Create: `tests/motor_control/test_stage6_bench.py`

**Interfaces:**
- Produces msh command `mc_speed_status` and host parser `parse_speed_status(text) -> dict | None`.
- Consumes `motor_control_isr_get_debug()` and `fault_manager_get()`.

- [ ] **Step 1: Write parser and safety-flow tests**

Use this exact wire format:

```text
spdstat active=1 target=6283 cmd=6283 meas=6201 iqref=84 id=2 iq=82 invalid=0 streak=0 freeze=0 fault=0x00000000
```

Assert all integers parse, malformed lines return `None`, timeout/failure always calls `mc_stop`, and final state verification requires DISABLED/EN=LOW/2812 duties/CCR4=5264/fault=0.

- [ ] **Step 2: Verify RED**

Run `python tests/motor_control/test_stage6_bench.py`.

Expected: import or attribute failure because `tests/stage6_bench.py` does not exist.

- [ ] **Step 3: Add the compact shell command**

Add a C90-compatible command that prints exactly one status line:

```c
static void mc_speed_status(int argc, char **argv)
{
    motor_control_isr_debug_t dbg;
    (void)argc; (void)argv;
    motor_control_isr_get_debug(&dbg);
    rt_kprintf("spdstat active=%d target=%ld cmd=%ld meas=%ld iqref=%ld "
               "id=%ld iq=%ld invalid=%lu streak=%u freeze=%lu fault=0x%08X\n",
               motor_control_isr_speed_active() ? 1 : 0,
               (long)dbg.spd_target_mrad_s, (long)dbg.spd_cmd_mrad_s,
               (long)dbg.spd_meas_mrad_s, (long)dbg.spd_iq_ref_ma,
               (long)dbg.id_avg_ma, (long)dbg.iq_avg_ma,
               (unsigned long)dbg.sample_invalid_total,
               (unsigned)dbg.sample_invalid_consecutive,
               (unsigned long)dbg.pi_freeze_count,
               (unsigned)fault_manager_get());
}
MSH_CMD_EXPORT(mc_speed_status, show compact speed loop status);
```

- [ ] **Step 4: Implement the bench**

The bench sends full DMA-safe command lines, polls `mc_speed_status` for five seconds per target, stores timestamped samples, rejects fault/invalid/freeze growth or `|iqref|>500`, calculates final one-second median speed and error, and always stops in `finally`. Targets are `+10`, `+60`, `-60`, and `+200rpm_elec`; the 10rpm point is directional only because encoder quantization dominates its percentage error.

- [ ] **Step 5: Verify and commit**

Run parser tests and `python tests/motor_control/test_speed_loop_static.py`, then commit only the command, bench, and tests.

### Task 4: Regression, Build, Flash, and Direction Gates

**Files:**
- Modify only if a test exposes a root-cause defect; do not tune gains in this task.

**Interfaces:**
- Consumes commits from Tasks 1-3.
- Produces a flashed firmware baseline whose direction and current safety are verified.

- [ ] **Step 1: Run all static and focused host tests**

Run every `*_static.py`, the new direction executable, speed-loop executable, Stage 5 parser tests, Stage 6 parser tests, `git diff --check`, and a Keil clean build. Expected: all pass; Keil reports 0 Error, 0 Warning.

- [ ] **Step 2: Flash and verify boot-safe state**

Run `project\MDK_V5\flash.bat only`, reopen COM9, and assert DISABLED, fault=0, EN=LOW, PWM=2812/2812/2812, CCR4=5264, VBUS 8..18V, and valid encoder calibration.

- [ ] **Step 3: Re-run current-loop regression**

Run `python tests\stage5_bench.py COM9`. Expected: all eight points pass and final safe state passes.

- [ ] **Step 4: Re-run open-loop direction proof**

At `mc_open 1000 +60 ramp`, normalized `spd_elec` must be positive; at `-60`, it must be negative. Raw unwrap must retain its original opposite hardware direction.

- [ ] **Step 5: Run short ±200mA torque gates**

On the confirmed free shaft, each sign must cause motion in opposite directions without settling into the previous 10～12° electromagnetic spring. Stop within two seconds, assert no fault, and end safe.

### Task 5: Tune and Qualify Speed-Loop Performance

**Files:**
- Modify: `application/motor_control/motor_params.h` only when a measured candidate is selected.
- Modify: `doc/调试记录.md`
- Modify: `doc/FOC控制器开发记录.md`
- Modify: `CLAUDE.md`

**Interfaces:**
- Consumes `tests/stage6_bench.py` samples.
- Produces selected `PID_SPEED_KP/KI`, measured performance, and explicit qualified range.

- [ ] **Step 1: Measure the existing candidate**

With `Kp=0.01`, `Ki=0.5`, `SPEED_IQ_LIMIT_A=0.5`, run the Stage 6 bench. Record rise/settling estimates, peak speed, final one-second median, steady error, peak Iq, Id, invalid/freeze deltas, and faults for every target.

- [ ] **Step 2: Apply the one-variable decision table**

Use these exact candidates, rebuilding/flashing and rerunning `+60/-60/+200` after each single change:

| Observed behavior | Next candidate |
| --- | --- |
| Sustained oscillation or peak >120% target | `Kp: 0.01 -> 0.005`, keep `Ki=0.5` |
| Stable but final error >5% with `|Iqref|<500mA` | keep selected Kp, `Ki: 0.5 -> 1.0` |
| Still >5% and non-oscillatory | keep selected Kp, `Ki: 1.0 -> 2.0` |
| Rise is slow, no overshoot, final error <=5% | `Kp: 0.01 -> 0.02`, keep selected Ki |
| Iq remains at 500mA and target is not reached | classify load/current limited; do not increase gain or current in this task |

Stop after the first candidate satisfying both-direction steady error <=5%, peak <=120%, no sustained oscillation, no fault, and no invalid/freeze growth. Never stack two unmeasured parameter changes.

- [ ] **Step 3: Confirm repeatability and safe stop**

Run the chosen candidate three times at +60, -60, and +200rpm_elec. Require the same sign, median steady error <=5%, `|Id|<=100mA`, `|Iqref|<=500mA`, no faults, and safe final state each run.

- [ ] **Step 4: Document and commit the selected result**

Record the root cause, before/after direction evidence, selected gains, speed limit, response metrics, supply/shaft conditions, and unqualified range. Commit parameter and documentation changes without staging `tests/stage5_bench_log.txt`.

### Task 6: Final Verification

**Files:**
- No new behavior changes.

- [ ] **Step 1: Run complete verification**

Run focused host tests, all static tests, Keil clean build, Stage 5 current bench, chosen Stage 6 speed bench repeated set, `git diff --check`, and `git status --short`.

- [ ] **Step 2: Confirm final hardware safety**

Query `mc_state`, `fault`, `pwm_info`, `vbus`, `enc_status`, and `mc_speed_status`; require DISABLED, fault=0, EN=LOW, three 2812 duties, CCR4=5264, valid encoder, and inactive speed loop.

- [ ] **Step 3: Hand off the qualified result**

Report the verified current-loop range, speed-loop target range and performance, selected gains/limit, remaining risks, exact commits, and the next independent Stage 7 position-loop design task.
