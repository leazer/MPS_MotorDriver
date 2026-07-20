# Low-Side Current Polarity Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Normalize MP6540H low-side current polarity before phase reconstruction so the full-quadrant current loop uses negative feedback and passes the approved low-current hardware matrix.

**Architecture:** Keep ADC conversion and raw SOx diagnostics unchanged. Perform the single polarity conversion inside the pure `current_reconstruction` boundary, reconstruct the discarded phase from normalized currents, then feed only normalized results to protection, Clarke/Park, and corrected diagnostics.

**Tech Stack:** C11 host tests under WSL GCC, AT32M412 firmware built with CMake/arm-none-eabi-gcc and Keil ARMCC, SEGGER J-Link Commander, Python/pyserial COM9 bench automation.

## Global Constraints

- Fixed sample tick remains `5264`; blanking remains `180` timer ticks.
- Command limit remains ±1.5A; automated hardware points remain exactly ±50/100/200/500mA.
- Software overcurrent remains 2.0A with four valid-frame debounce; invalid sampling remains eight consecutive frames.
- Current PI output remains limited to ±2V.
- Preserve `raw_ia/raw_ib/raw_ic` as low-side device-current-sign diagnostics.
- Do not enter speed mode until every low-current current-loop point passes.
- Keep the external supply limited to 1A and issue `mc_stop` on every failure path.

---

### Task 1: Add the Low-Side Polarity Regression

**Files:**
- Modify: `tests/current_sense/test_current_reconstruction.c`

**Interfaces:**
- Consumes: `current_reconstruction_run(const current_sample_plan_t *, float, float, float, uint16_t, current_reconstruction_result_t *)`
- Produces: host assertions defining raw low-side polarity and corrected FOC polarity.

- [ ] **Step 1: Write the failing host test**

Add a test using an all-valid plan whose smallest margin discards phase C. Supply low-side readings `raw=(-0.30,+0.10,99.0)` and assert raw fields are unchanged while corrected currents equal `( +0.30,-0.10,-0.20 )` and sum to zero. Update the existing exactly-two-valid expectations to the same normalized-current semantics.

- [ ] **Step 2: Run the test to verify RED**

Run:

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/current-sampling-reconstruction; gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control tests/current_sense/test_current_reconstruction.c application/motor_control/current_reconstruction.c -lm -o /tmp/test_current_reconstruction; /tmp/test_current_reconstruction"
```

Expected: assertion failure because the old implementation publishes `ia=-0.30` instead of `+0.30`.

### Task 2: Normalize Polarity at the Reconstruction Boundary

**Files:**
- Modify: `application/motor_control/current_reconstruction.c`
- Modify: `application/motor_control/current_reconstruction.h`

**Interfaces:**
- Consumes: calibrated SOx values in low-side device-current polarity.
- Produces: `result.raw_i*` unchanged and `result.i*` normalized to FOC phase-current polarity.

- [ ] **Step 1: Implement the minimal fix**

Replace the three assignments made after the valid-count gate with:

```c
out->ia = -raw_ia;
out->ib = -raw_ib;
out->ic = -raw_ic;
```

Keep the existing reconstruction switch unchanged so the missing phase is computed from normalized values.

- [ ] **Step 2: Document the result semantics**

Add concise field comments in `current_reconstruction_result_t` stating that `raw_i*` use the low-side sense-device sign and `i*` use the FOC phase-current sign.

- [ ] **Step 3: Run the focused test to verify GREEN**

Run the exact command from Task 1. Expected: `current reconstruction: all tests passed` with zero warnings.

- [ ] **Step 4: Commit the code fix**

```powershell
git add application/motor_control/current_reconstruction.c application/motor_control/current_reconstruction.h tests/current_sense/test_current_reconstruction.c
git diff --cached --check
git commit -m "fix: normalize low-side current polarity"
```

### Task 3: Run Software Regression and Build Both Firmware Projects

**Files:**
- Verify only; do not change production code unless a regression exposes a root cause.

**Interfaces:**
- Consumes: corrected reconstruction module.
- Produces: fresh host/static/build evidence.

- [ ] **Step 1: Run all repository `*_static.py` scripts and the Stage 5 bench parser regression**

Run:

```powershell
$files = @(rg --files | Where-Object { $_ -like '*_static.py' } | Sort-Object)
foreach ($file in $files) {
    python $file
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
python tests\motor_control\test_stage5_bench.py
python -m py_compile tests\stage5_bench.py
```

Expected: every script exits 0 and the focused bench regression prints `stage5 bench tests passed`.

- [ ] **Step 2: Compile and run strict host C tests for reconstruction, current loop, and speed loop**

Run:

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/current-sampling-reconstruction; gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control tests/current_sense/test_current_reconstruction.c application/motor_control/current_reconstruction.c -lm -o /tmp/test_current_reconstruction; /tmp/test_current_reconstruction; gcc -std=c11 -Wall -Wextra -Werror tests/motor_control/test_current_loop.c -lm -o /tmp/test_current_loop; /tmp/test_current_loop; gcc -std=c11 -Wall -Wextra -Werror tests/motor_control/test_speed_loop.c -lm -o /tmp/test_speed_loop; /tmp/test_speed_loop"
```

Expected: reconstruction pass line, 6 current-loop tests passed, and 5 speed-loop tests passed.

- [ ] **Step 3: Build the CMake Debug preset and run `project\\MDK_V5\\build.bat clean`**

Run:

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/current-sampling-reconstruction; cmake --build --preset Debug"
cmd.exe /c project\MDK_V5\build.bat clean
```

Expected: CMake reaches `[100%] Built target MPS_MotorDriver`; Keil reports `0 Error(s), 0 Warning(s)`.

- [ ] **Step 4: Run `git diff --check` and inspect the complete worktree diff**

Run:

```powershell
git diff --check
git status --short
git diff -- application/motor_control/current_reconstruction.c application/motor_control/current_reconstruction.h tests/current_sense/test_current_reconstruction.c
```

Expected: every test exits 0; CMake reaches 100%; Keil reports 0 Error(s), 0 Warning(s).

### Task 4: Flash and Run the Hardware Polarity Gate

**Files:**
- Modify: `doc/调试记录.md`
- Modify: `doc/FOC控制器开发记录.md`

**Interfaces:**
- Consumes: Keil HEX and COM9 shell commands.
- Produces: measured polarity and safe-state evidence.

- [ ] **Step 1: Send `mc_stop`, confirm DISABLED/EN=LOW/fault=0, then flash with J-Link Commander and verify**

Use the existing COM9 helper from `tests/stage5_bench.py` to send `mc_stop`, `mc_state`, `fault`, and `pwm_info`. Generate a temporary ignored J-Link command file with these exact commands:

```text
device AT32M412KBU7-4
si SWD
speed 1000
connect
r
h
loadfile E:\WorkSpaces\2_MotorDriver\MPS_MotorDriver\.worktrees\current-sampling-reconstruction\project\MDK_V5\objects\MPS_MotorDriver.hex
r
g
exit
```

Run:

```powershell
& 'C:\Program Files\SEGGER\JLink_V942\JLink.exe' -NoGui 1 -CommandFile '.superpowers\jlink_flash.jlink'
```

Expected: programming and verifying reach 100%, followed by `O.K.`.

- [ ] **Step 2: Run `mc_cal` and require `offset_valid=1`; require persisted encoder calibration `valid=1`**

Send, in order: `mc_stop`, `fault_clear`, `mc_cal`, `enc_cal_status`. Capture the complete replies. Do not proceed unless `mc_cal result: PASS offset_valid=1` and `valid : 1` are both present.

- [ ] **Step 3: Run `mc_open 500 0`, collect three `mc_debug` snapshots, and require corrected `Ia>0, Ib<0, Ic<0`, stable valid frames, and no fatal fault**

Use `tests.stage5_bench.send_cmd()` semantics: send `mc_open 500 0`, wait 0.5s, collect three `mc_debug` replies 0.6s apart, then read `fault`. Require `valid_mask=0x07`, zero invalid-consecutive growth, and `(fault & 0x9F)==0` in addition to the polarity check.

- [ ] **Step 4: Always issue `mc_stop` and verify DISABLED/EN=LOW before continuing**

Send `mc_stop`, `mc_state`, and `pwm_info`. Require `state : 0`, `CCR1/2/3 : 2812 / 2812 / 2812`, and `EN(PB10) : 0`.

### Task 5: Run the Full Low-Current Matrix and Record Evidence

**Files:**
- Modify: `tests/stage5_bench_log.txt`
- Modify: `doc/调试记录.md`
- Modify: `doc/FOC控制器开发记录.md`

**Interfaces:**
- Consumes: `python tests/stage5_bench.py COM9`.
- Produces: eight-point full-quadrant hardware result and final safe state.

- [ ] **Step 1: Run ±50mA first and require tracking tolerance, `|Id|<=100mA`, stable invalid/freeze counters, and no fatal fault**

Run one point per enable/stop cycle using `mc_cur -50 enc` and `mc_cur 50 enc`. For each point, collect a before snapshot and three running snapshots. Require `|Iq-target|<=20mA`, `|Id|<=100mA`, no growth in `invalid_total` or `pi_freeze`, `invalid_consecutive=0`, and `(fault & 0x9F)==0`.

- [ ] **Step 2: Only after ±50mA passes, run the existing complete ±50/100/200/500mA automation**

Run:

```powershell
$env:PYTHONUNBUFFERED='1'
python tests\stage5_bench.py COM9
```

Expected: eight `PASS` lines followed by `=== ALL PASS ===` and exit code 0.

- [ ] **Step 3: Append actual offsets, currents, masks, reconstruction phases, counters, faults, supply limit, and final stop state to both development records**

- [ ] **Step 4: Commit hardware evidence separately**

```powershell
git add doc/FOC控制器开发记录.md doc/调试记录.md tests/stage5_bench_log.txt
git diff --cached --check
git commit -m "test: verify low-side current polarity on hardware"
```

### Task 6: Final Review and Completion Audit

**Files:**
- Review all changes since `bf26919`.

**Interfaces:**
- Consumes: all software and hardware evidence.
- Produces: merge-readiness decision without claiming the unrun >500mA or speed gates.

- [ ] **Step 1: Re-run the focused host test, all static tests, both firmware builds, and `git diff --check`**

Repeat the exact Task 3 commands after the hardware evidence commit.

- [ ] **Step 2: Confirm the hardware log contains eight passing points and the final board state is DISABLED, EN=LOW, fault=0**

Run:

```powershell
rg -n "^PASS [+-](50|100|200|500)mA|ALL PASS|FAIL" tests\stage5_bench_log.txt
```

Then query COM9 with `mc_stop`, `mc_state`, `fault`, and `pwm_info` and capture their complete output.

- [ ] **Step 3: Review the branch diff for double-negation, diagnostic semantic mismatch, protection regression, and unrelated changes**

- [ ] **Step 4: Mark the objective complete only if every gate above has authoritative fresh evidence**
