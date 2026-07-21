# Single Motor Position and Velocity Feedforward Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and bench-qualify one motor as a 100 Hz position-plus-velocity-feedforward actuator over COM9, with local 1 kHz position control, endpoint hold, stream-timeout hold, and explicit safe disable.

**Architecture:** Add a direction-normalized continuous mechanical position to `encoder_service`, then implement a pure position-loop module that owns runtime joint zero, coherent setpoint publication, sequence validation, 20 ms velocity extrapolation, 100 ms lease timeout, and P-to-electrical-speed conversion. Integrate that output into the existing speed/current cascade, expose static and streaming shell commands plus checksummed telemetry, and qualify the result with a Python COM9 bench before any CAN implementation.

**Tech Stack:** ARMCC V5.06 C90-compatible embedded C, Python 3.14 + pyserial, WSL GCC host tests, Keil MDK build/flash, RT-Thread msh on COM9.

## Global Constraints

- Preserve V2 phase mapping U/V/W = TMR1 CH3/CH2/CH1.
- Preserve fixed ADC trigger `CCR4=5264`, current reconstruction, low-side polarity normalization, current PI, and the verified speed-loop baseline.
- Preserve `raw16`, `corrected_raw16`, `raw_unwrapped`, and `corrected_unwrapped` as hardware-direction diagnostics.
- Keep SPEED/POSITION Iq limited to `0.5 A`; do not expand the qualified current range.
- Position input units are mechanical `mdeg` and `mdeg/s`; speed-loop units remain electrical `rad/s`.
- Position outer loop runs at `1 kHz`; incoming stream period is `10 ms`; extrapolation is capped at `20 ms`; stream lease is `100 ms`.
- Normal completion and stream timeout hold position. Only explicit stop or fatal fault disables PWM and MP6540H.
- Joint zero is runtime-only and must be captured after boot before POSITION can start.
- ARMCC5 declarations remain at block starts; ISR code must not call RT-Thread APIs.
- Do not stage, overwrite, format, or revert the user-owned `tests/stage5_bench_log.txt` modification.
- Every powered test ends with `mc_stop` and verifies DISABLED, EN=LOW, duties 2812/2812/2812, `CCR4=5264`, and no fatal fault.

---

### Task 1: Publish Direction-Normalized Continuous Mechanical Position

**Files:**
- Modify: `application/motor_control/encoder_service.h`
- Modify: `application/motor_control/encoder_service.c`
- Modify: `tests/encoder_service/test_encoder_direction.c`
- Modify: `tests/encoder_service/test_encoder_service_static.py`

**Interfaces:**
- Consumes: `corrected_unwrapped`, `s_zero_raw`, `MOTOR_ENCODER_DIRECTION`, and `MOTOR_POLE_PAIRS`.
- Produces: `int32_t encoder_service_get_control_position_mdeg(void)` and snapshot field `control_position_mdeg`.

- [ ] **Step 1: Write the failing continuous-position test**

Extend `test_encoder_direction.c` after the existing direction checks:

```c
encoder_service_init();
encoder_service_set_zero(1000u);
encoder_service_set_calibration_table(zero_table, true);
assert(encoder_service_update_sample(900u, 0, 1u) == 0);
assert(encoder_service_get_snapshot(&snap));
assert(snap.control_position_mdeg >= 548 &&
       snap.control_position_mdeg <= 550);
assert(encoder_service_get_control_position_mdeg() ==
       snap.control_position_mdeg);

assert(encoder_service_update_sample(65500u, 0, 1u) == 0);
assert(encoder_service_update_sample(65400u, 0, 1u) == 0);
assert(encoder_service_get_snapshot(&snap));
assert(snap.control_position_mdeg > 0);
```

Add a separate sequence beginning near `raw=20`, crossing through 65535, and continuing downward. Assert the published position is monotonically positive and does not jump by approximately 360000 mdeg at wrap.

- [ ] **Step 2: Run the test and verify RED**

Run:

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/current-sampling-reconstruction; gcc -std=c11 -Wall -Wextra -Werror -Itests/encoder_service -Iapplication/motor_control tests/encoder_service/test_encoder_direction.c application/motor_control/encoder_service.c -o /tmp/test_encoder_direction; /tmp/test_encoder_direction"
```

Expected: compile failure because the field and getter do not exist.

- [ ] **Step 3: Implement the continuous control position**

Add to the snapshot and public API:

```c
int32_t control_position_mdeg;
int32_t encoder_service_get_control_position_mdeg(void);
```

Add this C90-compatible helper in `encoder_service.c`:

```c
static int32_t encoder_control_position_mdeg(int32_t corrected_unwrapped)
{
    int64_t counts;
    counts = (int64_t)corrected_unwrapped - (int64_t)s_zero_raw;
    counts *= (int64_t)MOTOR_ENCODER_DIRECTION;
    return (int32_t)((counts * 360000LL) / 65536LL);
}
```

Publish it after `corrected_unwrapped` updates and return the snapshot value from the getter. Do not change any existing raw/unwrapped field.

- [ ] **Step 4: Verify GREEN and static contract**

Require the new getter, 64-bit intermediate, direction constant, and unchanged raw diagnostic fields in `test_encoder_service_static.py`. Run the C test and Python static test; both must exit 0.

- [ ] **Step 5: Commit**

```powershell
git add -- application/motor_control/encoder_service.h application/motor_control/encoder_service.c tests/encoder_service/test_encoder_direction.c tests/encoder_service/test_encoder_service_static.py
git commit -m "feat: expose continuous motor position"
```

### Task 2: Implement the Pure Position and Feedforward Controller

**Files:**
- Modify: `application/motor_control/position_loop.h`
- Modify: `application/motor_control/position_loop.c`
- Modify: `application/motor_control/motor_params.h`
- Create: `tests/motor_control/test_position_loop.c`
- Create: `tests/motor_control/test_position_loop_static.py`

**Interfaces:**
- Consumes: direction-normalized sensor position in mechanical mdeg.
- Produces: `position_setpoint_t`, `position_loop_snapshot_t`, origin APIs, coherent submit API, `position_loop_run()`, timeout and tracking-fault diagnostics.

- [ ] **Step 1: Write the failing public-behavior test**

Use this required API in `test_position_loop.c`:

```c
position_setpoint_t point;
position_loop_snapshot_t snap;

point.position_mdeg = 30000;
point.velocity_mdeg_s = 10000;
point.sequence = 1u;
point.lease_ms = 100u;
position_loop_init();
assert(!position_loop_origin_valid());
assert(!position_loop_submit(&point));
position_loop_set_origin(120000, 25000);
assert(position_loop_origin_valid());
assert(position_loop_sensor_to_joint_mdeg(120000) == 25000);

assert(position_loop_submit(&point));
run_one_millisecond(120000);
assert(position_loop_get_snapshot(&snap));
assert(snap.target_position_mdeg == 30000);
assert(snap.velocity_ff_mdeg_s == 10000);
assert(snap.reference_position_mdeg == 30000);
assert(snap.speed_ref_elec_mrad_s > 0);
```

The test helper calls `position_loop_run(sensor_mdeg)` 16 times per millisecond. Add independent assertions for:

- reference advances by 100 mdeg after 10 ms at 10000 mdeg/s;
- extrapolation stops advancing after 20 ms;
- at 100 ms lease expiry, reference freezes and velocity feedforward becomes zero;
- a newer point clears timeout without resetting the origin;
- duplicate, backward, and half-range-ambiguous sequences are rejected;
- `65535 -> 0` sequence wrap is accepted;
- static `lease_ms=0` never times out;
- positive and negative errors produce matching speed signs;
- output clamps at `POSITION_SPEED_LIMIT_ELEC_RAD_S`;
- error beyond `POSITION_MAX_ERROR_MDEG` sets `tracking_fault` and returns zero;
- `position_loop_reset()` clears runtime/setpoint but preserves origin;
- `position_loop_init()` invalidates origin.

- [ ] **Step 2: Compile and verify RED**

Run:

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/current-sampling-reconstruction; gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control tests/motor_control/test_position_loop.c application/motor_control/position_loop.c -lm -o /tmp/test_position_loop; /tmp/test_position_loop"
```

Expected: compile failure because the new types and functions do not exist.

- [ ] **Step 3: Define the exact public interface**

Replace the stub header with:

```c
typedef struct {
    int32_t position_mdeg;
    int32_t velocity_mdeg_s;
    uint16_t sequence;
    uint16_t lease_ms;
} position_setpoint_t;

typedef struct {
    int32_t target_position_mdeg;
    int32_t velocity_ff_mdeg_s;
    int32_t reference_position_mdeg;
    int32_t measured_position_mdeg;
    int32_t error_mdeg;
    int32_t speed_ref_elec_mrad_s;
    uint16_t sequence;
    uint16_t age_ms;
    uint8_t origin_valid;
    uint8_t active;
    uint8_t stream_timeout;
    uint8_t tracking_fault;
} position_loop_snapshot_t;

void position_loop_init(void);
void position_loop_reset(void);
void position_loop_set_origin(int32_t sensor_mdeg, int32_t joint_mdeg);
bool position_loop_origin_valid(void);
int32_t position_loop_sensor_to_joint_mdeg(int32_t sensor_mdeg);
bool position_loop_submit(const position_setpoint_t *setpoint);
float position_loop_run(int32_t sensor_mdeg);
bool position_loop_get_snapshot(position_loop_snapshot_t *out);
```

- [ ] **Step 4: Implement minimal RED-to-GREEN behavior**

Add parameters:

```c
#define PID_POSITION_KP                    5.0f
#define POSITION_SPEED_LIMIT_RPM_ELEC      200.0f
#define POSITION_SPEED_LIMIT_ELEC_RAD_S    (POSITION_SPEED_LIMIT_RPM_ELEC * 6.28318530718f / 60.0f)
#define POSITION_MAX_VELOCITY_MDEG_S       60000
#define POSITION_MAX_ERROR_MDEG            30000
#define POSITION_EXTRAPOLATION_LIMIT_MS    20u
#define POSITION_LOOP_DIV                  16u
```

In `position_loop.c`, use a volatile odd/even generation around the published setpoint. On each 1 kHz update, consume only a stable even generation, validate the sequence with:

```c
delta = (uint16_t)(candidate - previous);
newer = delta != 0u && delta < 0x8000u;
```

Calculate:

```c
reference_mdeg = target_mdeg +
    (int32_t)(((int64_t)velocity_mdeg_s * extrapolation_ms) / 1000LL);
error_mdeg = reference_mdeg - measured_joint_mdeg;
speed_mech_rad_s = ((float)(velocity_ff_mdeg_s +
    (int32_t)(PID_POSITION_KP * (float)error_mdeg))) *
    3.14159265359f / 180000.0f;
speed_elec_rad_s = speed_mech_rad_s * (float)MOTOR_POLE_PAIRS;
```

At lease expiry, freeze the last reference before setting feedforward to zero. Clamp output symmetrically; on tracking fault publish zero speed.

- [ ] **Step 5: Verify GREEN, refactor, and commit**

Run the C test, the new static test, and existing speed-loop tests. Expected: all exit 0 without warnings.

```powershell
git add -- application/motor_control/position_loop.h application/motor_control/position_loop.c application/motor_control/motor_params.h tests/motor_control/test_position_loop.c tests/motor_control/test_position_loop_static.py
git commit -m "feat: add position velocity feedforward loop"
```

### Task 3: Integrate POSITION Into the Existing Speed and Current Cascade

**Files:**
- Modify: `application/motor_control/motor_control_isr.h`
- Modify: `application/motor_control/motor_control_isr.c`
- Modify: `application/motor_control/fault_manager.h`
- Modify: `application/motor_control/motor_app.c`
- Create: `tests/motor_control/test_position_isr_static.py`
- Modify: `tests/fault_manager/test_fault_manager.c`

**Interfaces:**
- Consumes: `encoder_service_get_control_position_mdeg()`, `position_loop_run()`, existing speed/current loops.
- Produces: position start/update/stop/activity APIs and fatal `FAULT_POSITION_TRACKING`.

- [ ] **Step 1: Write static and fault-manager RED tests**

Require these declarations:

```c
int motor_control_isr_position_start(const position_setpoint_t *setpoint);
int motor_control_isr_position_submit(const position_setpoint_t *setpoint);
void motor_control_isr_position_stop(void);
bool motor_control_isr_position_active(void);
```

The static test must assert that the POSITION branch calls all of:

```text
encoder_service_get_control_position_mdeg
position_loop_run
speed_loop_set_target_rad_s
speed_loop_run
current_loop_set_targets
current_loop_run
foc_ipark
foc_svpwm_3phase_high_side
```

It must reject the old `Stage 7+` 50% stub, require start to validate origin/fault/state, require submit not to call any reset function, and require stop to clear all three loop layers and disable PWM output.

Extend `test_fault_manager.c` to prove `FAULT_POSITION_TRACKING` is fatal and independently clearable.

- [ ] **Step 2: Run and verify RED**

Run:

```powershell
python tests/motor_control/test_position_isr_static.py
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/current-sampling-reconstruction; gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control tests/fault_manager/test_fault_manager.c application/motor_control/fault_manager.c -o /tmp/test_fault_manager; /tmp/test_fault_manager"
```

Expected: static assertions and the new fault assertion fail because POSITION remains a stub.

- [ ] **Step 3: Add the position tracking fault**

Add:

```c
FAULT_POSITION_TRACKING = 1u << 8
```

Include it in `FAULT_FATAL_MASK`. The ISR sets it when the position snapshot reports `tracking_fault`; the normal fatal-fault path then disables output.

- [ ] **Step 4: Implement start, live submit, stop, and ISR cascade**

`motor_control_isr_position_start()` must reject null setpoint, missing encoder validity, invalid origin, existing enabled state, and fatal faults. It resets current/speed/position runtime once, submits the first point, sets mode/state/activity, then enables PWM and OVF IRQ.

`motor_control_isr_position_submit()` must only accept an active POSITION mode and call `position_loop_submit()` without resetting any loop.

`motor_control_isr_position_stop()` must clear position activity, reset position/speed/current loops, zero current targets, disable OVF IRQ/output, set 50% duties, and set motor state DISABLED.

Implement POSITION using the SPEED branch's proven FOC path, changing only the speed target source:

```c
position_speed_ref = position_loop_run(
    encoder_service_get_control_position_mdeg());
speed_loop_set_target_rad_s(position_speed_ref);
iq_ref = speed_loop_run(encoder_tracker_get_speed_rad_s());
```

- [ ] **Step 5: Initialize and verify GREEN**

Call `position_loop_init()` from `motor_app_init()` after encoder initialization. Run the new static test, fault test, position-loop test, speed-loop test, and current-loop static tests. Expected: all pass.

- [ ] **Step 6: Commit**

```powershell
git add -- application/motor_control/motor_control_isr.h application/motor_control/motor_control_isr.c application/motor_control/fault_manager.h application/motor_control/motor_app.c tests/motor_control/test_position_isr_static.py tests/fault_manager/test_fault_manager.c
git commit -m "feat: cascade position speed and current control"
```

### Task 4: Add Safe COM9 Position Commands and Checksummed Telemetry

**Files:**
- Modify: `application/motor_shell.c`
- Create: `tests/motor_control/test_position_shell_static.py`
- Create: `tests/stage7_bench.py`
- Create: `tests/motor_control/test_stage7_bench.py`

**Interfaces:**
- Consumes: position ISR APIs, encoder control position, position snapshot, existing debug/fault snapshots.
- Produces: `mc_pos_zero`, `mc_pos`, `mc_pos_stream`, `mc_pos_status`, and parser `parse_position_status()`.

- [ ] **Step 1: Write shell-contract and parser RED tests**

Require these command signatures and safety rules:

```text
mc_pos_zero [known_mdeg]       disabled only
mc_pos <position_mdeg>         lease_ms=0
mc_pos_stream <seq> <position_mdeg> <velocity_mdeg_s>  lease_ms=100
mc_pos_status                  one checksummed line
```

The parser accepts only this complete format:

```text
ps a=1 t=10000 v=30000 r=10150 m=9820 e=330 w=1260 x=1200 q=180 g=15 o=0 n=42 f=00000000 k=89ABCDEF
```

Checksum is XOR over signed/unsigned numeric fields with seed `0x504F5331`. Tests must reject a changed field, a truncated line, missing key, duplicate key, and nonnumeric value.

- [ ] **Step 2: Run and verify RED**

Run:

```powershell
python tests/motor_control/test_position_shell_static.py
python tests/motor_control/test_stage7_bench.py
```

Expected: failures because commands, status format, parser, and bench do not exist.

- [ ] **Step 3: Implement joint-zero and target commands**

`mc_pos_zero` reads `encoder_service_get_control_position_mdeg()` and calls `position_loop_set_origin(sensor, known)`. It rejects enabled state.

`mc_pos` and `mc_pos_stream` share one helper that validates signed 32-bit parsing, target range, velocity range, sequence range, and origin validity. When disabled it calls `motor_control_isr_position_start()`; when already in POSITION it calls `motor_control_isr_position_submit()`; any other active mode is rejected.

Use a shell-owned incrementing sequence for `mc_pos`; use the explicit 16-bit sequence for `mc_pos_stream`.

- [ ] **Step 4: Implement compact status and parser**

Publish fields:

```text
a active
t target mdeg
v velocity feedforward mdeg/s
r extrapolated reference mdeg
m measured joint position mdeg
e position error mdeg
w electrical speed target mrad/s
x electrical speed measured mrad/s
q Iq reference mA
g setpoint age ms
o timeout flag
n applied sequence
f fault bitmap hex
k XOR checksum hex
```

Keep the line shorter than the verified DMA-safe serial limit. The Python parser recomputes the checksum before returning a sample.

- [ ] **Step 5: Implement deterministic bench cleanup**

The bench must:

```python
try:
    assert_safe_boot(serial_port)
    capture_joint_zero(serial_port, 0)
    run_static_steps(serial_port, [5000, -5000])
    run_reversal(serial_port, 10000, -10000)
    run_sine_stream(serial_port, amplitude_deg=10.0,
                    peak_velocity_deg_s=30.0, period_ms=10)
    verify_stream_timeout_hold(serial_port, timeout_ms=100)
finally:
    send_command(serial_port, "mc_stop")
    assert_final_safe_state(serial_port)
```

It records every accepted status sample, rejects any checksum/fault/current/sample-quality failure, and calculates static error, overshoot, P95 dynamic error, timeout-to-zero-feedforward, and final drift.

- [ ] **Step 6: Verify GREEN and commit**

Run both Python tests plus position/static ISR tests. Expected: all pass.

```powershell
git add -- application/motor_shell.c tests/motor_control/test_position_shell_static.py tests/stage7_bench.py tests/motor_control/test_stage7_bench.py
git commit -m "feat: add position stream bench interface"
```

### Task 5: Full Regression, Firmware Build, Flash, and Low-Risk Bring-Up

**Files:**
- Modify production code only when a failing test or measured root cause requires it.

**Interfaces:**
- Consumes: Tasks 1-4.
- Produces: a flashed position-mode baseline with current and speed regressions preserved.

- [ ] **Step 1: Run full host/static verification**

Run all `tests/**/*_static.py`, encoder direction executable, position-loop executable, speed-loop executable, fault-manager executable, Stage 5/6/7 parser tests, and `git diff --check`. Expected: all exit 0.

- [ ] **Step 2: Run Keil clean build**

Run the repository's ARMCC5 clean build through `project\MDK_V5\build.bat`. Expected: HEX created, `0 Error(s), 0 Warning(s)`.

- [ ] **Step 3: Flash and verify boot-safe state**

Run `project\MDK_V5\flash.bat only`, reconnect COM9, then query `mc_state`, `fault`, `pwm_info`, `vbus`, `enc_status`, and `mc_pos_status`. Require DISABLED, position inactive, fault=0, EN=LOW, duties 2812/2812/2812, `CCR4=5264`, VBUS 8..18V, and valid encoder calibration.

- [ ] **Step 4: Preserve inner-loop qualification**

Run `python tests\stage5_bench.py COM9`, followed by the existing Stage 6 qualified points. Require the current matrix and speed baseline to remain within their recorded gates and end safe.

- [ ] **Step 5: Bring up POSITION in bounded steps**

With the shaft free:

1. `mc_pos_zero 0`.
2. Command +1000 mdeg, stop, inspect status and direction.
3. Command -1000 mdeg, stop, inspect status and direction.
4. Increase to ±5000 mdeg only after both signs move correctly without fault or saturation.

At the first wrong sign, fatal fault, sustained 500 mA saturation, or motion beyond the commanded direction, immediately run `mc_stop` and return to a failing automated test for the identified defect.

### Task 6: Tune and Qualify Single-Motor Position Performance

**Files:**
- Modify: `application/motor_control/motor_params.h` only for a measured selected parameter.
- Modify: `docs/superpowers/specs/2026-07-21-single-motor-position-feedforward-design.md` only if measured hardware forces an approved contract correction.
- Modify: `doc/调试记录.md`
- Modify: `doc/FOC控制器开发记录.md`
- Modify: `CLAUDE.md`

**Interfaces:**
- Consumes: `tests/stage7_bench.py` metrics.
- Produces: selected position gain/speed cap and a documented qualified operating envelope.

- [ ] **Step 1: Measure the initial candidate**

With `Kp_position=5.0 s^-1`, electrical cap 200 rpm, and Iq cap 0.5 A, run static ±5 deg and reversal ±10 deg. Record settle time, final absolute error, overshoot, peak Iq, peak speed, invalid/freeze deltas, and faults.

- [ ] **Step 2: Apply one-variable tuning decisions**

| Measured behavior | Next single change |
| --- | --- |
| sustained position oscillation or overshoot >20% | `Kp_position: 5.0 -> 3.0` |
| stable, slow, static error >0.5 deg without current saturation | `Kp_position: 5.0 -> 7.0` |
| speed target repeatedly hits 200 rpm while error remains bounded | keep gain, raise no limits; reduce test trajectory speed |
| Iq remains at 0.5 A | classify current/load limited; do not increase current |
| dynamic ripple but mean path is correct | keep controller; quantify ripple before considering a later compensation task |

Rebuild, flash, and rerun both signs after each single change. Stop at the first candidate meeting static error and overshoot gates.

- [ ] **Step 3: Run the 100 Hz feedforward trajectory**

Send a 10 deg sine with analytic velocity feedforward and peak speed at most 30 deg/s. Require P95 position error <=2 deg, correct velocity sign, no command discontinuity, no fault, and `|Iq_ref|<=500 mA`.

- [ ] **Step 4: Verify timeout hold and recovery**

Stop transmitting during motion. Require the timeout flag, zero velocity feedforward within 30 ms after lease expiry, frozen reference, and no continued reference drift. Submit the next valid sequence and require smooth recovery without restarting the ISR mode.

- [ ] **Step 5: Confirm repeatability**

Repeat static ±5 deg, reversal ±10 deg, sine stream, and timeout recovery three times. Every run must satisfy the same gates and final safe state.

- [ ] **Step 6: Document and commit the selected result**

Record supply, free-shaft condition, firmware commit, zeroing procedure, selected parameters, per-run metrics, qualified position/velocity range, remaining low-speed ripple, and the explicit absence of CAN/mechanism validation.

```powershell
git add -- application/motor_control/motor_params.h docs/superpowers/specs/2026-07-21-single-motor-position-feedforward-design.md 'doc/调试记录.md' 'doc/FOC控制器开发记录.md' CLAUDE.md
git commit -m "docs: qualify single motor position control"
```

Stage only paths actually changed; omit unchanged files from `git add`.

### Task 7: Final Verification and Handoff to CAN Integration

**Files:**
- No behavior changes.

- [ ] **Step 1: Run fresh complete verification**

Run all focused C tests, all static Python tests, Stage 5 current bench, qualified Stage 6 speed bench, Stage 7 position bench repeated set, ARMCC5 clean build, `git diff --check`, and `git status --short`.

- [ ] **Step 2: Confirm final hardware safety**

Query `mc_state`, `fault`, `pwm_info`, `vbus`, `enc_status`, `mc_speed_status`, and `mc_pos_status`. Require DISABLED, fault=0, EN=LOW, duties 2812/2812/2812, `CCR4=5264`, valid encoder, speed inactive, and position inactive.

- [ ] **Step 3: Check every design gate**

Map fresh evidence to static error, reversal overshoot, sine P95 error, timeout behavior, repeatability, current limit, sampling quality, build result, and final safety. Report any unmet gate as incomplete rather than broadening the qualification.

- [ ] **Step 4: Hand off the stable interface**

Report the exact commit, qualified control range, COM9 command contract, `position_setpoint_t` contract, measured performance, remaining risks, and the next CAN task: map the 8-byte position/velocity/sequence frame into `position_loop_submit()` with a 100 ms lease, without changing the position controller.
