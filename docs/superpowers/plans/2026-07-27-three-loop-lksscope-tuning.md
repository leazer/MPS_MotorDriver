# Three-Loop LKS Scope Tuning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add RAM-backed live tuning for the current, speed, position, overcurrent, and current-sampling parameters, then expose the tuning block and complete motor diagnostics in the LKS Scope numeric table without changing its chart.

**Architecture:** A new `motor_tuning` module owns the externally linked `volatile g_motor_tuning` parameter block and `volatile g_motor_loop_debug` runtime snapshot. Startup copies existing `motor_params.h` defaults into RAM; each loop and protection decision reads the RAM fields at execution time, while existing authoritative motor-state and encoder snapshots remain the LKS Scope sources for state and sensor data.

**Tech Stack:** C11/ARMCC5-compatible C, Python 3 XML/static tests, WSL host GCC tests, CMake/arm-none-eabi-gcc firmware build, Keil MDK project XML, LKS Scope XML v1.4.8.

## Global Constraints

- Work only in `E:/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/motor-driver-can-node`.
- Preserve every pre-existing uncommitted change, especially `application/motor_app.c`, `application/motor_control/motor_params.h`, `debug.lksscope`, `project/MDK_V5/MPS_MotorDriver.uvprojx`, and the modified tests.
- Do not create implementation commits that would absorb pre-existing changes from overlapping files; leave implementation changes uncommitted for user review.
- Parameters are RAM-only and reset to `motor_params.h` defaults on boot.
- Do not add guessed hard ceilings, automatic clamping of tuning fields, Flash persistence, CAN tuning commands, or chart changes.
- Keep PWM frequency, current/speed/position loop dividers, pole pairs, and encoder direction compile-time fixed.
- Retain existing fault latching and PWM shutdown behavior while making the current/imbalance/sample thresholds live.
- Use aligned native scalar fields only; LKS Scope writes one field at a time with no multi-field transaction.

---

## File Structure

- Create `application/motor_control/motor_tuning.h`: public tuning/debug structures, globals, and initialization API.
- Create `application/motor_control/motor_tuning.c`: global storage and reset-to-default initialization.
- Create `tests/motor_control/test_motor_tuning.c`: host behavior tests for defaults and live loop response.
- Create `tests/motor_control/test_motor_tuning_static.py`: build integration and live-parameter wiring contracts.
- Modify `application/motor_control/current_loop.c`: read current tuning fields and publish D/Q diagnostics.
- Modify `application/motor_control/speed_loop.c`: read speed tuning fields and publish speed diagnostics.
- Modify `application/motor_control/position_loop.c`: read position tuning fields and publish position diagnostics.
- Modify `application/motor_control/can_motion_service.c`: use the same live position command/velocity/error limits as the position loop.
- Modify `application/motor_control/motor_control_isr.c`: read live protection thresholds and publish protection diagnostics.
- Modify `application/motor_app.c`: initialize the tuning block before all consumers.
- Modify `CMakeLists.txt`: compile `motor_tuning.c`.
- Modify `project/MDK_V5/MPS_MotorDriver.uvprojx`: add `motor_tuning.c` to the existing motor-control group without altering other user changes.
- Modify `tests/motor_control/test_position_loop.c`: initialize the tuning block for the real position-loop host test.
- Modify `tests/motor_control/test_can_motion_service.c`: initialize the tuning block for live position-limit tests.
- Modify affected Python static tests so they assert live behavior/wiring instead of requiring runtime code to read fixed macros.
- Modify `tests/motor_control/test_lksscope_static.py`: validate the expanded numeric list while preserving the current chart exactly.
- Modify `debug.lksscope`: add type-7 numeric rows only.

---

### Task 1: Define the failing live-tuning contracts

**Files:**
- Create: `tests/motor_control/test_motor_tuning.c`
- Create: `tests/motor_control/test_motor_tuning_static.py`
- Modify: `tests/motor_control/test_lksscope_static.py`

**Interfaces:**
- Consumes: existing `motor_params.h`, loop APIs, `debug.lksscope`, CMake source list, and Keil motor-control group.
- Produces: executable contracts for `motor_tuning_init()`, `g_motor_tuning`, `g_motor_loop_debug`, build integration, live loop response, and the exact numeric variable set.

- [ ] **Step 1: Write the failing default and live-response host test**

Create `test_motor_tuning.c` with real production calls:

```c
#include <assert.h>
#include <math.h>
#include "current_loop.h"
#include "motor_params.h"
#include "motor_tuning.h"
#include "position_loop.h"
#include "speed_loop.h"

static void test_defaults_are_loaded(void)
{
    motor_tuning_init();
    assert(g_motor_tuning.current.id_kp == PID_ID_KP);
    assert(g_motor_tuning.current.iq_ki == PID_IQ_KI);
    assert(g_motor_tuning.current.id_output_limit_v == PID_CURRENT_OUT_LIMIT);
    assert(g_motor_tuning.speed.kp == PID_SPEED_KP);
    assert(g_motor_tuning.speed.output_limit_A == PID_SPEED_OUT_LIMIT);
    assert(g_motor_tuning.position.kp == PID_POSITION_KP);
    assert(g_motor_tuning.position.max_error_mdeg == POSITION_MAX_ERROR_MDEG);
    assert(g_motor_tuning.protection.phase_overcurrent_A == IQ_OVERCURRENT_A);
    assert(g_motor_tuning.protection.sample_invalid_limit ==
           CURRENT_SAMPLE_INVALID_LIMIT);
}

static void test_current_loop_reads_changed_ram_gain(void)
{
    float vd;
    float vq;
    motor_tuning_init();
    current_loop_init();
    g_motor_tuning.current.id_kp = 2.0f;
    g_motor_tuning.current.id_ki = 0.0f;
    g_motor_tuning.current.id_output_limit_v = 10.0f;
    current_loop_set_targets(1.0f, 0.0f);
    current_loop_run(0.0f, 0.0f, &vd, &vq);
    assert(fabsf(vd - 2.0f) < 1.0e-6f);
    assert(g_motor_loop_debug.current.id_error_A == 1.0f);
    assert(g_motor_loop_debug.current.vd_output_v == vd);
}
```

Add equivalent real tests for:

```c
/* after 16 speed_loop_run() calls:
 * target=10, measured=0, ramp=100000, kp=0.1, ki=0, friction=0,
 * output_limit=10 => Iq output 1.0 A */
assert(fabsf(iq - 1.0f) < 1.0e-5f);

/* a 1000 mdeg position error with kp=1.0 produces a smaller speed
 * reference than the same setup after changing kp to 2.0 */
assert(second_speed_ref > first_speed_ref);
```

The production change each test catches is a loop continuing to use a compile-time macro or an initialization-time copy after LKS Scope changes RAM.

- [ ] **Step 2: Write failing build and wiring tests**

Create `test_motor_tuning_static.py` that:

```python
assert "motor_tuning.c" in CMakeLists.txt
assert "<FileName>motor_tuning.c</FileName>" in the Keil motor-control group
assert "motor_tuning_init();" occurs before "current_loop_init();" in motor_app.c
assert "g_motor_tuning.current" in current_loop.c
assert "g_motor_tuning.speed" in speed_loop.c
assert "g_motor_tuning.position" in position_loop.c
assert "g_motor_tuning.protection" in motor_control_isr.c
assert "POSITION_MAX_ERROR_MDEG" not in position_loop_run()
assert "IQ_OVERCURRENT_A" not in motor_control_isr_tick()
```

Use the existing brace-counting `function_body()` pattern so assertions inspect behavior-bearing functions rather than unrelated source text.

- [ ] **Step 3: Expand the failing LKS Scope contract**

In `test_lksscope_static.py`, add exact required-name sets for:

```python
REQUIRED_TUNING = {
    "g_motor_tuning.current.id_kp",
    "g_motor_tuning.current.id_ki",
    "g_motor_tuning.current.iq_kp",
    "g_motor_tuning.current.iq_ki",
    "g_motor_tuning.current.id_integral_limit_v",
    "g_motor_tuning.current.iq_integral_limit_v",
    "g_motor_tuning.current.id_output_limit_v",
    "g_motor_tuning.current.iq_output_limit_v",
    "g_motor_tuning.speed.kp",
    "g_motor_tuning.speed.kp_brake",
    "g_motor_tuning.speed.ki",
    "g_motor_tuning.speed.integral_limit_A",
    "g_motor_tuning.speed.output_limit_A",
    "g_motor_tuning.speed.friction_A",
    "g_motor_tuning.speed.ramp_rad_s2",
    "g_motor_tuning.position.kp",
    "g_motor_tuning.position.speed_limit_elec_rad_s",
    "g_motor_tuning.position.max_velocity_mdeg_s",
    "g_motor_tuning.position.max_error_mdeg",
    "g_motor_tuning.position.command_limit_mdeg",
    "g_motor_tuning.position.extrapolation_limit_ms",
    "g_motor_tuning.position.iq_friction_A",
    "g_motor_tuning.position.iq_friction_moving_A",
    "g_motor_tuning.position.iq_friction_error_mdeg",
    "g_motor_tuning.protection.phase_overcurrent_A",
    "g_motor_tuning.protection.overcurrent_debounce_ticks",
    "g_motor_tuning.protection.imbalance_threshold_A",
    "g_motor_tuning.protection.imbalance_debounce_ticks",
    "g_motor_tuning.protection.sample_blanking_ticks",
    "g_motor_tuning.protection.sample_invalid_limit",
}
```

Add required loop-debug fields for current errors/integrals/outputs, speed error/integral/feedforward/raw and limited Iq, position proportional/raw and limited speed output, and protection measurements/counters. Continue asserting the pre-existing motor-state, mode, setpoints, raw/calibrated encoder fields, ADC fields, and reconstruction fields.

Change the mapping assertion to the user’s current `./build/Debug/MPS_MotorDriver.elf`. Accept either blank addresses (symbol-resolved on load) or `0x2000...` RAM addresses. Preserve the existing `REQUIRED_CURVES` exact-set assertion so any chart change fails.

- [ ] **Step 4: Run RED verification**

Run:

```powershell
python tests/motor_control/test_motor_tuning_static.py
python tests/motor_control/test_lksscope_static.py
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/motor-driver-can-node; gcc -std=c11 -Wall -Wextra -Werror -Itests/motor_control/stubs -Iapplication/motor_control -Iplatform/at32m412 tests/motor_control/test_motor_tuning.c application/motor_control/motor_tuning.c application/motor_control/current_loop.c application/motor_control/speed_loop.c application/motor_control/position_loop.c -lm -o /tmp/test_motor_tuning"
```

Expected: the static tests fail because the new symbols/rows are absent; host compilation fails because `motor_tuning.h/.c` do not exist.

---

### Task 2: Add the central RAM parameter and debug blocks

**Files:**
- Create: `application/motor_control/motor_tuning.h`
- Create: `application/motor_control/motor_tuning.c`
- Create: `tests/motor_control/stubs/board_motor_pins.h`
- Modify: `application/motor_app.c`
- Modify: `CMakeLists.txt`
- Modify: `project/MDK_V5/MPS_MotorDriver.uvprojx`

**Interfaces:**
- Consumes: defaults from `motor_params.h`.
- Produces: `void motor_tuning_init(void)`, `volatile motor_tuning_t g_motor_tuning`, and `volatile motor_loop_debug_t g_motor_loop_debug`.

- [ ] **Step 1: Define exact public structures**

`motor_tuning.h` defines four parameter substructures named `current`, `speed`, `position`, and `protection`, using the field names from Task 1.

Define debug substructures with these exact fields:

```c
typedef struct {
    float id_ref_A, iq_ref_A;
    float id_measured_A, iq_measured_A;
    float id_error_A, iq_error_A;
    float id_integral_v, iq_integral_v;
    float vd_unlimited_v, vq_unlimited_v;
    float vd_output_v, vq_output_v;
    uint8_t id_saturated, iq_saturated;
} motor_current_loop_debug_t;

typedef struct {
    float target_rad_s, command_rad_s, measured_rad_s, error_rad_s;
    float integral_A, active_kp, friction_A;
    float iq_unlimited_A, iq_output_A;
    uint8_t saturated;
} motor_speed_loop_debug_t;

typedef struct {
    int32_t target_position_mdeg, reference_position_mdeg;
    int32_t measured_position_mdeg, error_mdeg, velocity_ff_mdeg_s;
    float proportional_velocity_mdeg_s;
    float speed_unlimited_elec_rad_s, speed_output_elec_rad_s;
    float iq_feedforward_A;
} motor_position_loop_debug_t;

typedef struct {
    float max_phase_current_A, imbalance_A;
    uint16_t overcurrent_consecutive, invalid_consecutive;
    uint16_t imbalance_consecutive;
    uint32_t invalid_total;
    uint8_t frame_valid, overcurrent_active;
} motor_protection_debug_t;
```

`motor_loop_debug_t` aggregates those four blocks. Include C++ guards and only standard integer headers.

- [ ] **Step 2: Implement reset-to-default initialization**

`motor_tuning.c` defines both globals and explicitly assigns every parameter field from these existing macros:

```c
PID_ID_KP, PID_ID_KI, PID_IQ_KP, PID_IQ_KI,
PID_CURRENT_INTEGRAL_LIMIT, PID_CURRENT_OUT_LIMIT,
PID_SPEED_KP, PID_SPEED_KP_BRAKE, PID_SPEED_KI,
PID_SPEED_INTEGRAL_LIMIT, PID_SPEED_OUT_LIMIT,
SPEED_IQ_FRICTION_A,
PID_POSITION_KP, POSITION_SPEED_LIMIT_ELEC_RAD_S,
POSITION_MAX_VELOCITY_MDEG_S, POSITION_MAX_ERROR_MDEG,
POSITION_COMMAND_LIMIT_MDEG, POSITION_EXTRAPOLATION_LIMIT_MS,
POSITION_IQ_FRICTION_A, POSITION_IQ_FRICTION_MOVING_A,
POSITION_IQ_FRICTION_ERROR_MDEG,
IQ_OVERCURRENT_A, OVERCURRENT_DEBOUNCE_TICKS,
IMBALANCE_THRESHOLD_A, IMBALANCE_DEBOUNCE_TICKS,
CURRENT_SAMPLE_BLANKING_TICKS, CURRENT_SAMPLE_INVALID_LIMIT
```

Set `speed.ramp_rad_s2` to the existing `50.0f` default. Zero the debug block with `memset`.

- [ ] **Step 3: Add startup and both build integrations**

Call `motor_tuning_init()` after `motor_control_init()` and before `current_loop_init()`.

Add:

```cmake
${CMAKE_SOURCE_DIR}/application/motor_control/motor_tuning.c
```

next to the other motor-control sources. Add the equivalent Keil `<File>` entry inside `application/motor_control`.

The host stub contains only:

```c
#ifndef BOARD_MOTOR_PINS_H
#define BOARD_MOTOR_PINS_H
#define PWM_FREQUENCY_HZ 16000u
#endif
```

- [ ] **Step 4: Run focused GREEN verification**

Run the Task 1 static test. Compile `test_motor_tuning.c`; it may still fail link/assert because loops are not wired yet, but the failure must advance from missing files/symbols to the expected live-behavior assertions.

---

### Task 3: Wire all three loops and publish diagnostics

**Files:**
- Modify: `application/motor_control/current_loop.c`
- Modify: `application/motor_control/speed_loop.c`
- Modify: `application/motor_control/position_loop.c`
- Modify: `application/motor_control/can_motion_service.c`
- Modify: `tests/motor_control/test_position_loop.c`
- Modify: `tests/motor_control/test_can_motion_service.c`
- Modify: affected speed/position/current Python static tests

**Interfaces:**
- Consumes: `g_motor_tuning.current`, `.speed`, `.position`.
- Produces: `g_motor_loop_debug.current`, `.speed`, `.position`.

- [ ] **Step 1: Wire current-loop live parameters**

Before each D/Q execution, copy the corresponding live Kp, Ki, integral limit, and output limit into `s_pid_d`/`s_pid_q`. Keep accumulated integrals unchanged when gains change.

For each axis compute:

```c
error = reference - measured;
output = pid_f32_exec(...);
unlimited = pid.kp * error + pid.integral;
saturated = output != unlimited;
```

Publish references, measurements, errors, integrals, unlimited outputs, limited outputs, and flags to `g_motor_loop_debug.current`.

- [ ] **Step 2: Wire speed-loop live parameters**

At each 1 kHz update:

- use `speed.ramp_rad_s2` for `max_step`;
- use `speed.ki` and `speed.integral_limit_A` for integration;
- choose `speed.kp` or `speed.kp_brake`;
- use `speed.friction_A`;
- clamp with `speed.output_limit_A`.

Publish the exact values used in the calculation to `g_motor_loop_debug.speed`. On divider-skipped ISR calls, continue publishing the latest measurement and retain the last 1 kHz calculation fields.

- [ ] **Step 3: Wire position-loop and CAN limits**

Replace runtime reads of the position macros with the corresponding live fields in:

- first-target safety;
- setpoint velocity validation;
- extrapolation;
- tracking-fault comparison;
- proportional position calculation;
- speed clamp;
- static/moving friction feedforward;
- CAN command position/velocity validation and first-target delta validation.

Publish the position proportional contribution, unlimited electrical speed, limited electrical speed, and Iq feedforward beside the existing snapshot values.

- [ ] **Step 4: Update real host fixtures and run GREEN tests**

Call `motor_tuning_init()` at the start of `test_position_loop.c` and `test_can_motion_service.c` main functions. Link `motor_tuning.c` in their host commands.

Run:

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/motor-driver-can-node; gcc -std=c11 -Wall -Wextra -Werror -Itests/motor_control/stubs -Iapplication/motor_control -Iplatform/at32m412 tests/motor_control/test_motor_tuning.c application/motor_control/motor_tuning.c application/motor_control/current_loop.c application/motor_control/speed_loop.c application/motor_control/position_loop.c -lm -o /tmp/test_motor_tuning; /tmp/test_motor_tuning; gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control tests/motor_control/test_position_loop.c application/motor_control/motor_tuning.c application/motor_control/position_loop.c -lm -o /tmp/test_position_loop; /tmp/test_position_loop; gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control -Icommunication tests/motor_control/test_can_motion_service.c application/motor_control/motor_tuning.c application/motor_control/can_motion_service.c communication/can_protocol.c -o /tmp/test_can_motion_service; /tmp/test_can_motion_service"
```

Expected: all three executables pass with `-Werror`.

---

### Task 4: Wire live protection thresholds and diagnostics

**Files:**
- Modify: `application/motor_control/motor_control_isr.c`
- Modify: `tests/motor_control/test_current_fault_debounce_static.py`
- Modify: `tests/motor_control/test_current_sampling_static.py`
- Modify: `tests/current_sense/test_current_reconstruction.c`

**Interfaces:**
- Consumes: `g_motor_tuning.protection`.
- Produces: `g_motor_loop_debug.protection`.

- [ ] **Step 1: Add failing protection-limit tests**

Extend `test_current_reconstruction.c` with literal cases proving:

- `blanking_ticks=10` accepts a margin of 10 while `blanking_ticks=11` rejects it;
- `overcurrent_limit=2` trips on the second consecutive valid overcurrent frame;
- changing `overcurrent_limit=4` delays the trip to the fourth;
- changing `invalid_limit=3` trips on the third invalid frame.

Update static tests to require ISR arguments from `g_motor_tuning.protection` rather than fixed macros.

Run tests and confirm the static ISR test fails because macros are still used.

- [ ] **Step 2: Replace ISR threshold reads**

Use the live fields for:

```c
current_reconstruction_run(..., g_motor_tuning.protection.sample_blanking_ticks, ...);
fabsf(sample.phase) >= g_motor_tuning.protection.phase_overcurrent_A;
current_sample_guard_step(...,
    g_motor_tuning.protection.overcurrent_debounce_ticks,
    g_motor_tuning.protection.sample_invalid_limit);
fabsf(i_sum) > g_motor_tuning.protection.imbalance_threshold_A;
s_imbal_consec >= g_motor_tuning.protection.imbalance_debounce_ticks;
```

Do not alter fault bits, guard reset rules, or PWM shutdown branches.

- [ ] **Step 3: Publish protection runtime values**

Publish:

```c
max_phase_current_A = max(fabsf(sample.ia), fabsf(sample.ib), fabsf(sample.ic));
imbalance_A = ia + ib + ic;
frame_valid = sample.frame_valid;
overcurrent_active = phase_overcurrent;
overcurrent_consecutive = s_sample_guard.overcurrent_consecutive;
invalid_consecutive = s_sample_guard.invalid_consecutive;
invalid_total = s_sample_guard.invalid_total;
imbalance_consecutive = s_imbal_consec;
```

- [ ] **Step 4: Run focused protection tests**

Run:

```powershell
python tests/motor_control/test_current_fault_debounce_static.py
python tests/motor_control/test_current_sampling_static.py
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/motor-driver-can-node; gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control tests/current_sense/test_current_reconstruction.c application/motor_control/current_reconstruction.c -lm -o /tmp/test_current_reconstruction; /tmp/test_current_reconstruction"
```

Expected: all pass.

---

### Task 5: Populate only the LKS Scope numeric table

**Files:**
- Modify: `debug.lksscope`
- Modify: `tests/motor_control/test_lksscope_static.py`

**Interfaces:**
- Consumes: stable ELF names from `g_motor_tuning`, `g_motor_loop_debug`, `s_motor_control`, `s_snapshot`, ISR diagnostics, and sampling diagnostics.
- Produces: one type-7 list containing all requested writable parameters and observable variables.

- [ ] **Step 1: Verify RED after production symbols exist**

Run `python tests/motor_control/test_lksscope_static.py`.

Expected: fail only because the new names are missing from the type-7 numeric list.

- [ ] **Step 2: Add parameter rows**

Add one `<var>` per `REQUIRED_TUNING` item with:

- Chinese descriptions prefixed `电流参数/`, `速度参数/`, `位置参数/`, or `保护参数/`;
- engineering unit matching the C field;
- `showType="Float"` for `float`, `showType="Signed"` for signed integers, and `showType="Unsigned"` for unsigned integers;
- `interval="10"`;
- `addr=""`, allowing the current ELF mapping to resolve symbols.

- [ ] **Step 3: Add runtime rows and preserve existing monitoring**

Add all required `g_motor_loop_debug` fields. Keep existing rows for:

```text
s_motor_control.state
s_motor_control.mode
s_motor_control.fault_flags
s_motor_control.iq_ref_ma
s_motor_control.speed_ref_rpm
s_motor_control.position_ref_mdeg
s_snapshot.*
s_sampling_debug_snapshot.*
s_dbg_*
```

Do not edit the type-8 form or layout blobs.

- [ ] **Step 4: Verify XML behavior**

Run:

```powershell
python tests/motor_control/test_lksscope_static.py
[xml](Get-Content -LiteralPath 'debug.lksscope' -Raw) | Out-Null
```

Expected: static tests pass, XML parses, numeric names and descriptions have no duplicates, current curve set is byte-for-byte semantically unchanged.

---

### Task 6: Full regression, firmware build, and symbol verification

**Files:**
- Verify all task files; do not modify unrelated files.

**Interfaces:**
- Consumes: complete implementation.
- Produces: passing host/static suite and resolvable target symbols.

- [ ] **Step 1: Run focused host suite**

Run the Task 3 and Task 4 host commands plus the existing current-loop and speed-loop executables:

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/motor-driver-can-node; gcc -std=c11 -Wall -Wextra -Werror tests/motor_control/test_current_loop.c -lm -o /tmp/test_current_loop; /tmp/test_current_loop; gcc -std=c11 -Wall -Wextra -Werror tests/motor_control/test_speed_loop.c -lm -o /tmp/test_speed_loop; /tmp/test_speed_loop"
```

- [ ] **Step 2: Run all repository static tests**

Run each tracked `tests/**/*_static.py` directly, excluding inaccessible cache directories. Expected: every script exits 0.

- [ ] **Step 3: Build firmware**

Run:

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/motor-driver-can-node; cmake -S . -B build/Debug -DCMAKE_BUILD_TYPE=Debug; cmake --build build/Debug --clean-first -j2"
```

Expected: `build/Debug/MPS_MotorDriver.elf` is produced with no compile or link errors.

- [ ] **Step 4: Verify symbols and XML references**

Run `arm-none-eabi-nm -S build/Debug/MPS_MotorDriver.elf` and require data/BSS symbols for:

```text
g_motor_tuning
g_motor_loop_debug
s_motor_control
s_snapshot
s_sampling_debug_snapshot
```

Parse the type-7 LKS names and confirm each base symbol exists in `nm` output.

- [ ] **Step 5: Check patch hygiene**

Run:

```powershell
git diff --check
git status --short
git diff --stat
```

Expected: no whitespace errors; all pre-existing user modifications remain present; no generated host executable is newly added by this task; implementation remains uncommitted because touched files overlap user-owned changes.
