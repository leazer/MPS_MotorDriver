# V1/V2 LKS Scope Debug View Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add stable debug signals and an LKS Scope layout that shows encoder, current, and current-loop phase-voltage curves on both V1 and V2 firmware.

**Architecture:** Extend the existing encoder and ISR debug snapshots only where required. Put inverse Clarke in `foc_core` as a tested pure function, publish fixed-point values already suitable for J-Link memory polling, and keep one XML schema whose symbol set is identical in both worktrees.

**Tech Stack:** ARMCC C90-compatible C, Python 3 static tests, WSL GCC host tests, Keil MDK AXF build, SEGGER J-Link symbols, LKS Scope XML v1.4.8.

## Global Constraints

- `uu/uv/uw` are current-loop command phase voltages after inverse Park and inverse Clarke, before SVPWM zero-sequence injection, in mV.
- `raw_elec_mrad` skips the nonlinear error table but uses the same electrical zero and pole-pair conversion as calibrated `elec_mrad`.
- Keep the command limit at ±1.5A, software overcurrent at 2A, PWM at 16kHz, and the existing sampling plan unchanged.
- Preserve COM9, J-Link clock 10MHz, 10ms LKS sample interval, and the user's current dock/layout serialization.
- Do not overwrite or stage `E:/WorkSpaces/2_MotorDriver/MPS_MotorDriver/debug.lksscope`; use it only as the UI-layout template.
- V1 and V2 must expose the same configured symbol names; verify actual addresses from each freshly built AXF.
- This task performs no flashing or powered motor test.

---

### Task 1: Add RED tests for missing debug semantics

**Files:**
- Modify: `tests/foc_core/test_foc_clarke.c`
- Create: `tests/foc_core/board_motor_pins.h`
- Modify: `tests/encoder_service/test_encoder_service_static.py`
- Create: `tests/motor_control/test_lksscope_static.py`

**Interfaces:**
- Consumes: existing `foc_core`, encoder snapshot, ISR debug globals, and `debug.lksscope`.
- Produces: regression gates for `foc_inv_clarke`, `raw_elec_mrad`, phase-voltage publication, and the 11 required XML curves.

- [ ] **Step 1: Add inverse-Clarke assertions**

Extend the host test with calls equivalent to:

```c
foc_inv_clarke(1.0f, 0.0f, &vu, &vv, &vw);
assert(approx_eq(vu, 1.0f, 1e-4f));
assert(approx_eq(vv, -0.5f, 1e-4f));
assert(approx_eq(vw, -0.5f, 1e-4f));

foc_inv_clarke(0.0f, 1.0f, &vu, &vv, &vw);
assert(approx_eq(vu + vv + vw, 0.0f, 1e-4f));
```

Add a test-only `board_motor_pins.h` defining `TMR1_ARR 7499u` so host GCC can compile the complete `foc_core.c`.

- [ ] **Step 2: Add encoder and scope static contracts**

Require `raw_elec_mrad` in `encoder_snapshot_t`; require the source to calculate one electrical angle from raw position and one from corrected position. Parse `debug.lksscope` with `xml.etree.ElementTree`, require COM9/10ms/local AXF path, the 13 requested curves, and the agreed monitoring variables.

- [ ] **Step 3: Verify RED**

Run:

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/current-sampling-reconstruction; gcc -std=c11 -Wall -Wextra -Werror -Itests/foc_core -Iapplication/motor_control tests/foc_core/test_foc_clarke.c application/motor_control/foc_core.c -lm -o /tmp/test_foc_core; /tmp/test_foc_core"
python tests/encoder_service/test_encoder_service_static.py
python tests/motor_control/test_lksscope_static.py
```

Expected: inverse-Clarke compilation fails because `foc_inv_clarke` is absent; encoder static test fails because `raw_elec_mrad` is absent; scope test fails because the requested lists are absent.

---

### Task 2: Publish the missing V2 firmware debug values

**Files:**
- Modify: `application/motor_control/foc_core.h`
- Modify: `application/motor_control/foc_core.c`
- Modify: `application/motor_control/encoder_service.h`
- Modify: `application/motor_control/encoder_service.c`
- Modify: `application/motor_control/motor_control_isr.h`
- Modify: `application/motor_control/motor_control_isr.c`

**Interfaces:**
- Produces: `foc_inv_clarke(float,float,float*,float*,float*)`; `encoder_snapshot_t.raw_elec_mrad`; `s_dbg_uu_mv`, `s_dbg_uv_mv`, `s_dbg_uw_mv`; and `motor_control_isr_debug_t.uu_mv/uv_mv/uw_mv`.

- [ ] **Step 1: Implement and reuse inverse Clarke**

Add the public pure function:

```c
void foc_inv_clarke(float v_alpha, float v_beta,
                     float *vu, float *vv, float *vw)
{
    *vu = v_alpha;
    *vv = -0.5f * v_alpha + 0.86602540f * v_beta;
    *vw = -0.5f * v_alpha - 0.86602540f * v_beta;
}
```

Replace the duplicate three assignments in SVPWM with `foc_inv_clarke(v_alpha, v_beta, &va, &vb, &vc)`.

- [ ] **Step 2: Publish raw and calibrated electrical angles**

Refactor the position-to-electrical-angle math into a helper that accepts an already selected 16-bit position. In `encoder_accept_sample`, assign:

```c
s_snapshot.raw_elec_mrad = encoder_elec_mrad_from_position(raw);
s_snapshot.elec_mrad = encoder_elec_mrad_from_position(corrected);
```

Keep both values normalized to 0..6282/6283mrad and preserve the existing calibrated control angle behavior.

- [ ] **Step 3: Publish current-loop U/V/W voltage commands**

After inverse Park in CURRENT and SPEED, call inverse Clarke and convert V to mV into the three volatile debug globals. Reset all three to zero whenever closed-loop activity is stopped. Copy them into the public debug getter structure.

- [ ] **Step 4: Verify GREEN**

Run the focused commands from Task 1. Expected: host output reports all FOC assertions passed; both Python scripts exit 0.

---

### Task 3: Build the V2 LKS Scope configuration

**Files:**
- Modify: `debug.lksscope`

**Interfaces:**
- Consumes: V2 AXF symbols and the main checkout's current XML params/layout.
- Produces: LKS Scope v1.4.8 XML with one numeric form and one four-subplot curve form.

- [ ] **Step 1: Build V2 and extract fresh symbols**

Run the repository's Keil clean build. Use `fromelf --text -s project/MDK_V5/objects/MPS_MotorDriver.axf` to extract every configured base symbol, then calculate structure-field offsets from the compiled layout.

- [ ] **Step 2: Update XML from the user layout template**

Keep all `<params>`, `<layouts>`, `<tabs>`, window geometry, COM9 and map path from the main checkout template. Populate the type-7 form with current addresses and populate the type-8 form with:

```text
encoder: s_snapshot.raw16, s_snapshot.raw_elec_mrad, s_snapshot.elec_mrad
dq:      s_dbg_id_ma, s_dbg_iq_ma
phase I: s_sampling_debug_snapshot.ia_ma/.ib_ma/.ic_ma
phase U: s_dbg_uu_mv, s_dbg_uv_mv, s_dbg_uw_mv
```

Use descriptions `编码器原始值`, `标定前电角度(mrad)`, `标定后电角度(mrad)`, `Id/Iq(mA)`, `Iu/Iv/Iw(mA)`, and `Uu/Uv/Uw(mV)`.

- [ ] **Step 3: Validate XML and V2 AXF linkage**

Run `python tests/motor_control/test_lksscope_static.py`, parse XML with PowerShell `[xml]`, and verify all base symbols through `fromelf`. Expected: no missing curve/list symbols and no duplicate variable descriptions.

---

### Task 4: Port and verify the identical V1 instrumentation

**Files:**
- Apply the Task 1-3 code, test and scope changes to the `compare/v1-current-loop` worktree.

**Interfaces:**
- Consumes: verified V2 implementation commits.
- Produces: V1 source/config with identical signal semantics and a V1-local AXF path.

- [ ] **Step 1: Apply the V2 implementation commits to V1**

Cherry-pick only the focused debug implementation/config commits. Resolve no board mapping files; V1 must remain U/V/W=CH1/CH2/CH3 and LED=PA0.

- [ ] **Step 2: Run V1 focused and static tests**

Run the same host FOC test, encoder static test, scope static test, and the complete existing `*_static.py` suite in the V1 worktree. Expected: all exit 0.

- [ ] **Step 3: Build V1 and audit symbols**

Run a Keil clean build and `fromelf --text -s` on the V1 AXF. Compare the configured V1/V2 address tables. If addresses differ, update only V1 `addr` attributes and rerun its scope test.

---

### Task 5: Final verification and handoff

**Files:**
- No new behavior changes.

**Interfaces:**
- Produces: clean evidence for both worktrees and concise LKS usage notes.

- [ ] **Step 1: Run final verification**

For both worktrees run `git diff --check`, focused host tests, all Python static tests, XML parsing, Keil clean build, and AXF symbol resolution. Confirm the main checkout still has only its original user-owned `debug.lksscope` modification.

- [ ] **Step 2: Review requirement coverage**

Check all 11 requested curves, the common numeric variables, V1/V2 mappings, units, pre/post calibration semantics, and current-loop voltage semantics against the approved design.

- [ ] **Step 3: Commit intentionally**

Commit V2 implementation/tests/config as focused commits and port them to V1. Do not merge either branch into `main` or `hw/v2-coaxial-encoder` in this task.
