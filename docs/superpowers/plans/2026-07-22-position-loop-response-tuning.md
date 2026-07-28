# Position Loop Response Tuning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** After X-Track realtime CAN timing passes, tune unloaded Node 1 position response from the qualified `Kp=5.0` baseline to the lowest gain that meets the `+5/-5 deg` response gates without changing global speed-mode behavior.

**Architecture:** Use X-Track as the 100 Hz trajectory source and COM9 as an independent measurement/safety channel. First capture a post-transport-fix baseline with unchanged MotorDriver control parameters; then change only `PID_POSITION_KP` to 7.5, rebuild/flash, and compare identical 60 s runs. A 10.0 gain or position-only speed/friction change is allowed only when the previous measured stage misses a named gate.

**Tech Stack:** AT32M412 MotorDriver, ARMCC5/C90-compatible C, position-speed-current cascade, Python 3.14/pyserial, GCC host tests, Keil MDK, J-Link `20721552`, COM9, X-Track J-Link `20721850`.

## Global Constraints

- Do not begin until the X-Track realtime CAN run demonstrates `sync_age < 30 ms` with no reference freeze.
- Node 1 remains unloaded and unobstructed; every exception path issues `mc_stop` and verifies DISABLED/EN-low/neutral PWM.
- Tune one variable at a time. Do not change `PID_SPEED_KP`, `PID_SPEED_KI`, current-loop gains, `POSITION_SPEED_LIMIT_RPM`, or `POSITION_IQ_LIMIT_A` during the Kp stages.
- Keep peak `|Iq_ref| <= 0.5 A`, overshoot `<=0.5 deg`, settle-to-`+/-0.5 deg <=350 ms`, and measured span `>=9.5 deg` for the 10 deg command span.
- Do not modify or stage `project/MDK_V5/MPS_MotorDriver.uvprojx`.
- Preserve raw logs, parameter commit, firmware hash, serial port, J-Link serial, timing statistics, and final safe state for the forum post/video history.

---

### Task 1: Capture the Unchanged-Gain Realtime-CAN Baseline

**Files:**
- Create: `docs/motor-control/evidence/2026-07-22-position-response-kp5.txt`
- Create: `tests/position_response_metrics.py`
- Create: `tests/motor_control/test_position_response_metrics.py`

**Interfaces:**
- Consumes: X-Track realtime CAN evidence and existing COM9 `mc_pos_status`/`can_status` output.
- Produces: deterministic metrics for command span, measured span, 0.5 deg settling time, overshoot, peak Iq, and CAN counter growth.

- [ ] **Step 1: Write failing parser/metric tests**

Use a synthetic two-reversal log and require:

```python
metrics = summarize(samples)
assert metrics.command_span_deg == 10.0
assert metrics.measured_span_deg == 9.6
assert metrics.max_settle_ms == 320
assert metrics.max_overshoot_deg == 0.3
assert metrics.peak_abs_iq_a == 0.18
```

Also require missing reversals, truncated telemetry, a counter increment, or no final DISABLED record to raise `ValueError`.

Run: `python tests/motor_control/test_position_response_metrics.py`

Expected: FAIL because `tests/position_response_metrics.py` does not exist.

- [ ] **Step 2: Implement the minimal metrics helper**

Parse saved timestamped samples rather than opening serial. Split segments on command-direction reversal; compute extrema and the first timestamp after each reversal that enters and remains within 0.5 deg through endpoint hold. Report overshoot beyond the commanded endpoint, maximum absolute `iqref`, and deltas for protocol/RX-overflow/bus-off/timeout/fault counters.

- [ ] **Step 3: Verify metrics and capture the Kp=5 baseline**

Run the unit test, then sample at least 60 s of the already running X-Track commissioning motion with current `PID_POSITION_KP 5.0f`. Save raw samples and the computed summary. Require final `mc_stop`, DISABLED, fault 0, EN low, and neutral PWM even if a performance gate fails.

- [ ] **Step 4: Commit evidence tooling and baseline**

```powershell
git add -- tests/position_response_metrics.py tests/motor_control/test_position_response_metrics.py docs/motor-control/evidence/2026-07-22-position-response-kp5.txt
git commit -m "test: capture realtime CAN position baseline"
```

### Task 2: Raise Position Kp to 7.5 and Qualify

**Files:**
- Modify: `application/motor_control/motor_params.h`
- Create: `tests/motor_control/test_position_tuning_profile.py`
- Create: `docs/motor-control/evidence/2026-07-22-position-response-kp7p5.txt`

**Interfaces:**
- Consumes: Task 1 metric helper and exact Kp=5 baseline.
- Produces: qualified `PID_POSITION_KP 7.5f` or evidence that the next stage is necessary.

- [ ] **Step 1: Write the failing profile test**

Parse `motor_params.h` and require exactly:

```python
assert define("PID_POSITION_KP") == "7.5f"
assert define("PID_SPEED_KP") == "0.01f"
assert define("PID_SPEED_KI") == "0.01f"
assert define("SPEED_IQ_LIMIT_A") == "0.5f"
assert define("POSITION_IQ_FRICTION_MOVING_A") == "0.04f"
```

Run: `python tests/motor_control/test_position_tuning_profile.py`

Expected: FAIL showing `PID_POSITION_KP` is `5.0f`.

- [ ] **Step 2: Make the single production change**

Change only:

```c
#define PID_POSITION_KP                 7.5f
```

- [ ] **Step 3: Run host regression and build**

Run the new profile test, existing `test_position_loop.c`, position static tests, CAN motion service/integration tests, Stage 5/6/7 bench unit tests, and the full repository host suite used by the branch. Rebuild Keil with ARMCC5 and require 0 errors, 0 warnings. Confirm the user `uvprojx` hash did not change.

- [ ] **Step 4: Flash and run the identical 60 s qualification**

Stop Node 1 first, flash/verify through J-Link `20721552`, reset MotorDriver and X-Track, then capture the same samples and metrics as Task 1. Abort on fault, counter growth, `|Iq_ref|>0.5 A`, or persistent oscillation. Finish with `mc_stop` and safe-state verification.

Pass when measured span is at least 9.5 deg, settling is at most 350 ms, overshoot is at most 0.5 deg, and all safety/CAN gates remain clean. If it passes, do not perform Task 3.

- [ ] **Step 5: Commit Kp=7.5 and evidence**

```powershell
git add -- application/motor_control/motor_params.h tests/motor_control/test_position_tuning_profile.py docs/motor-control/evidence/2026-07-22-position-response-kp7p5.txt
git commit -m "tune: stiffen position response"
```

### Task 3: Conditionally Evaluate Kp 10.0

**Files:**
- Modify: `application/motor_control/motor_params.h`
- Modify: `tests/motor_control/test_position_tuning_profile.py`
- Create: `docs/motor-control/evidence/2026-07-22-position-response-kp10.txt`

**Interfaces:**
- Entry condition: Kp=7.5 misses span or settling gate while overshoot, current, CAN, and safety gates pass.
- Produces: qualified Kp=10.0 or a documented decision to retain 7.5.

- [ ] **Step 1: Record the entry condition**

In the Kp=7.5 evidence, name the failed numeric gate. Do not enter this task for CAN jitter, invalid telemetry, current saturation, overshoot, or oscillation; those require diagnosis rather than more Kp.

- [ ] **Step 2: Update the test first and verify RED**

Change the expected profile to `PID_POSITION_KP == "10.0f"` and run the profile test. Expected: FAIL showing 7.5f.

- [ ] **Step 3: Change only Kp and verify GREEN**

Change only:

```c
#define PID_POSITION_KP                 10.0f
```

Run the same host, Keil, flash, 60 s motion, metrics, and cleanup sequence from Task 2.

- [ ] **Step 4: Choose the lowest passing gain and commit**

If 10.0 passes and materially improves the failed gate, retain it. Otherwise restore both production define and test to 7.5, rerun the profile test/build, and retain 7.5. Save all measurements, including rejected-stage evidence.

```powershell
git add -- application/motor_control/motor_params.h tests/motor_control/test_position_tuning_profile.py docs/motor-control/evidence/2026-07-22-position-response-kp10.txt
git commit -m "tune: qualify final position gain"
```

### Task 4: Final Cross-Project Verification and History Record

**Files:**
- Create: `docs/motor-control/test-reports/2026-07-22-xtrack-position-response.md`
- Modify: the relevant MotorDriver and X-Track evidence manifests if present.

**Interfaces:**
- Consumes: final X-Track commit, final MotorDriver parameter commit, both raw evidence sets.
- Produces: a forum/video-ready comparison and reproducible final state.

- [ ] **Step 1: Re-run the final product pair**

Run X-Track host/simulator/production/commissioning builds and MotorDriver host/Keil builds fresh. Flash the final pair and execute one additional 60 s run. Verify the same timing, response, current, fault, and cleanup gates.

- [ ] **Step 2: Write the comparison report**

Include a table for Kp 5.0, 7.5, and 10.0 when tested: measured span, maximum settle time, maximum overshoot, peak Iq, maximum sync age, error-counter deltas, result, and rejection reason. Link raw logs and state explicitly that the communication fix preceded tuning.

- [ ] **Step 3: Verify repository hygiene and commit**

Run `git diff --check`, fresh relevant test suites, and `git status --short` in both worktrees. Confirm only each user's pre-existing `uvprojx` file remains unstaged, then commit the report/manifests without staging either project file.
