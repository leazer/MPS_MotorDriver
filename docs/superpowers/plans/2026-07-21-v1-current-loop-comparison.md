# V1 Current-Loop Comparison Branch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `compare/v1-current-loop` from `main@89b29a8` with the verified current-loop/reconstruction software from `feat/current-sampling-reconstruction`, while retaining the V1 board mapping for an honest V1/V2 hardware A/B test.

**Architecture:** Apply the source branch's final tree as a squash onto an isolated V1 worktree, then keep the hardware difference behind `board_motor_pins.h`: V1 uses U/V/W=CH1/CH2/CH3 and LED=PA0. A mapping-specific static test provides the RED/GREEN gate, while a source-tree equality audit proves the control ISR, reconstruction, protection, current-sense HAL, PWM state machine, shell, and bench script remain identical to the validated V2 source.

**Tech Stack:** C11, Python 3 static/bench tests, Git worktrees, WSL host GCC, GNU Arm Embedded CMake build, Keil MDK ARMCC, AT32M412 + MP6540H.

## Global Constraints

- Base must be exactly `main@89b29a8`; do not merge or rebase `9519e6f` into the comparison branch.
- Source control snapshot is `694e644`; later source-branch commits may contain only this V1 design and plan documentation.
- V1 mapping is U/V/W=`TMR_SELECT_CHANNEL_1/2/3` and LED=`GPIOA/GPIO_PINS_0`.
- Current-loop behavior, low-side polarity normalization, sampling at CCR4=5264, protection, diagnostics, and bench thresholds must remain identical to the validated source.
- Existing V2 bench output is reference evidence only; V1 results remain explicitly `NOT RUN` until tested on the old board.
- Power testing remains limited to `Iq_ref=±50/±100/±200/±500mA` with the supply current limit at 1A.
- Do not modify or stage the user's `debug.lksscope` in the main checkout.
- Do not update `main` or `hw/v2-coaxial-encoder` during this plan; deliver the comparison branch first.

---

### Task 1: Create the isolated V1 comparison worktree

**Files:**
- No tracked files changed.
- Create worktree directory: `.worktrees/v1-current-loop`

**Interfaces:**
- Consumes: `main@89b29a8`, ignored `.worktrees/` directory.
- Produces: branch `compare/v1-current-loop` checked out at `E:/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/v1-current-loop`.

- [ ] **Step 1: Verify branch identities and source-code freeze**

Run from `E:/WorkSpaces/2_MotorDriver/MPS_MotorDriver`:

```powershell
git rev-parse main
git rev-parse feat/current-sampling-reconstruction
git merge-base main feat/current-sampling-reconstruction
git diff --quiet 694e644..feat/current-sampling-reconstruction -- CMakeLists.txt application platform project/MDK_V5 tests
```

Expected: `main` prints `89b29a8...`; the source branch contains `694e644`; merge base is `89b29a8...`; the final command exits 0, proving later commits changed documentation only.

- [ ] **Step 2: Verify the worktree directory is ignored**

Run:

```powershell
git check-ignore -q .worktrees
```

Expected: exit 0.

- [ ] **Step 3: Create the comparison branch and worktree**

Run:

```powershell
git worktree add .worktrees/v1-current-loop -b compare/v1-current-loop main
```

Expected: Git reports a new worktree at `89b29a8` on `compare/v1-current-loop`.

- [ ] **Step 4: Verify the isolated baseline**

Run from the new worktree:

```powershell
git status --short --branch
Get-ChildItem tests -Recurse -Filter '*_static.py' | Sort-Object FullName | ForEach-Object { python $_.FullName; if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE } }
```

Expected: clean `compare/v1-current-loop`; every static test present on `main` passes. If the baseline fails, stop before transplanting.

---

### Task 2: Squash the validated control tree and create the V1 RED mapping gate

**Files:**
- Modify: `tests/test_pwm_mapping_static.py`
- Pending source transplant: all paths changed by `main..feat/current-sampling-reconstruction`

**Interfaces:**
- Consumes: final source tree from `feat/current-sampling-reconstruction`, `function_body()` test helper, `PWM_PHASE_*_TMR_CHANNEL` macros.
- Produces: a failing V1 mapping test against the temporarily transplanted V2 mapping.

- [ ] **Step 1: Apply the source tree without committing**

Run from the V1 worktree:

```powershell
git merge --squash feat/current-sampling-reconstruction
git status --short
```

Expected: the source changes are staged, HEAD remains based on `main`, and no merge commit is created.

- [ ] **Step 2: Replace the V2 mapping assertions with the V1 contract**

Modify `tests/test_pwm_mapping_static.py` so its constants and two tests are:

```python
WK_CONFIG = ROOT / "project" / "inc" / "at32m412_416_wk_config.h"


def test_v1_preserves_direct_pwm_phase_outputs():
    pins = read(PINS)
    pwm = read(PWM)
    setter = function_body(pwm, "motor_pwm_at32m412_set_duty_ticks")
    apply_duty = function_body(pwm, "pwm_apply_duty_ticks")

    assert re.search(r"#define\s+PWM_PHASE_U_TMR_CHANNEL\s+TMR_SELECT_CHANNEL_1\b", pins)
    assert re.search(r"#define\s+PWM_PHASE_V_TMR_CHANNEL\s+TMR_SELECT_CHANNEL_2\b", pins)
    assert re.search(r"#define\s+PWM_PHASE_W_TMR_CHANNEL\s+TMR_SELECT_CHANNEL_3\b", pins)
    assert re.search(r"\ba\s*=\s*pwm_clamp_duty\(phase_u\)", setter)
    assert re.search(r"\bb\s*=\s*pwm_clamp_duty\(phase_v\)", setter)
    assert re.search(r"\bc\s*=\s*pwm_clamp_duty\(phase_w\)", setter)
    assert "pwm_apply_duty_ticks(a, b, c)" in setter
    assert "tmr_channel_value_set(TMR1, PWM_PHASE_U_TMR_CHANNEL, a)" in apply_duty
    assert "tmr_channel_value_set(TMR1, PWM_PHASE_V_TMR_CHANNEL, b)" in apply_duty
    assert "tmr_channel_value_set(TMR1, PWM_PHASE_W_TMR_CHANNEL, c)" in apply_duty


def test_v1_led_stays_on_pa0():
    pins = read(PINS)
    wk_config = read(WK_CONFIG)
    board_init = read(BOARD_INIT)

    assert re.search(r"#define\s+LED_GPIO_PORT\s+GPIOA\b", pins)
    assert re.search(r"#define\s+LED_PIN\s+GPIO_PINS_0\b", pins)
    assert re.search(r"#define\s+LED_GPIO_PORT\s+GPIOA\b", wk_config)
    assert re.search(r"#define\s+LED_PIN\s+GPIO_PINS_0\b", wk_config)
    assert "gpio_init(LED_GPIO_PORT, &gpio_init_struct)" in board_init
    assert "gpio_init(SPI2_CS_GPIO_PORT, &gpio_init_struct)" in board_init
```

Update the `__main__` block to call these two new test names and keep printing `pwm mapping static tests passed`.

- [ ] **Step 3: Run the focused test to prove RED**

Run:

```powershell
python tests/test_pwm_mapping_static.py
```

Expected: FAIL on the first V1 mapping assertion because the transplanted source still has U=CH3/W=CH1 or LED=PB8. A pass here means the test does not distinguish V1 from V2 and must be corrected before proceeding.

---

### Task 3: Implement the V1 hardware boundary and isolate V2 evidence

**Files:**
- Modify: `platform/at32m412/board_motor_pins.h`
- Modify: `platform/at32m412/board_init_at32m412.c`
- Modify: `project/inc/at32m412_416_wk_config.h`
- Modify: `tests/test_pwm_mapping_static.py`
- Rename: `tests/stage5_bench_log.txt` to `tests/stage5_bench_v2_reference_log.txt`
- Create: `tests/stage5_bench_log.txt`
- Modify: `doc/调试记录.md`

**Interfaces:**
- Consumes: the V1 RED test from Task 2 and generic PWM channel macros used by `pwm_apply_duty_ticks()`.
- Produces: explicit V1 BSP mapping with unchanged control-layer data flow and clearly separated V1/V2 bench evidence.

- [ ] **Step 1: Set V1 PWM and LED macros**

In `platform/at32m412/board_motor_pins.h`, use:

```c
/* V1 hardware direct mapping: phase U -> CH1, V -> CH2, W -> CH3 */
#define PWM_PHASE_U_TMR_CHANNEL  TMR_SELECT_CHANNEL_1
#define PWM_PHASE_V_TMR_CHANNEL  TMR_SELECT_CHANNEL_2
#define PWM_PHASE_W_TMR_CHANNEL  TMR_SELECT_CHANNEL_3

/* ===== LED (PA0, negative terminal to IO, active low) ===== */
#define LED_GPIO_PORT            GPIOA
#define LED_PIN                  GPIO_PINS_0
```

In `project/inc/at32m412_416_wk_config.h`, use:

```c
#define LED_PIN    GPIO_PINS_0
#define LED_GPIO_PORT    GPIOA
```

- [ ] **Step 2: Correct V1-only board comments without changing initialization order**

In `platform/at32m412/board_init_at32m412.c`, change the two `LED (PB8)` comments to `LED (PA0)`. Keep the separate calls below unchanged so both accesses remain macro-driven:

```c
gpio_init(LED_GPIO_PORT, &gpio_init_struct);
gpio_init(SPI2_CS_GPIO_PORT, &gpio_init_struct);
```

- [ ] **Step 3: Run the V1 mapping test to prove GREEN**

Run:

```powershell
python tests/test_pwm_mapping_static.py
```

Expected: `pwm mapping static tests passed`.

- [ ] **Step 4: Separate the imported V2 log from the pending V1 result**

Run:

```powershell
git mv tests/stage5_bench_log.txt tests/stage5_bench_v2_reference_log.txt
```

Create `tests/stage5_bench_log.txt` with exactly:

```text
=== V1 HARDWARE BENCH: NOT RUN ===
Branch: compare/v1-current-loop
Required matrix: -50,+50,-100,+100,-200,+200,-500,+500 mA
Safety limit: bench supply current limit 1A
Run: python tests/stage5_bench.py COM9
```

- [ ] **Step 5: Add an explicit pending-V1 entry to the debug record**

Append to `doc/调试记录.md`:

```markdown
## 2026-07-21 V1/V2 电流环硬件对比准备

- V1 对比分支：`compare/v1-current-loop`，基于硬件调整前 `main@89b29a8`。
- 控制软件来源：`feat/current-sampling-reconstruction`，低侧极性修复提交 `314080c`，V2 权威八点记录提交 `eb2c0da`。
- V1 映射：U/V/W=CH1/CH2/CH3，LED=PA0；V2 映射提交 `9519e6f` 未进入 V1 运行配置。
- `tests/stage5_bench_v2_reference_log.txt` 是 V2 参考证据；`tests/stage5_bench_log.txt` 在旧板实测前保持 `NOT RUN`。
- V1 实机仍需按停机检查、零偏标定、`mc_open 500 0` 极性门禁、±50mA 门禁、八点矩阵和最终安全状态的顺序执行。
```

- [ ] **Step 6: Audit forbidden V2 mappings**

Run:

```powershell
rg -n "PWM_PHASE_U_TMR_CHANNEL\s+TMR_SELECT_CHANNEL_3|PWM_PHASE_W_TMR_CHANNEL\s+TMR_SELECT_CHANNEL_1|LED \(PB8\)|LED_GPIO_PORT\s+GPIOB|LED_PIN\s+GPIO_PINS_8" platform/at32m412/board_motor_pins.h platform/at32m412/board_init_at32m412.c project/inc/at32m412_416_wk_config.h
```

Expected: no matches and `rg` exit code 1.

- [ ] **Step 7: Prove control files equal the validated source snapshot**

Run:

```powershell
git diff --exit-code 694e644 -- CMakeLists.txt application/motor_app.c application/motor_shell.c application/motor_control platform/at32m412/current_sense_at32m412.c platform/at32m412/current_sense_at32m412.h platform/at32m412/motor_pwm_at32m412.c platform/at32m412/motor_pwm_at32m412.h project/MDK_V5/MPS_MotorDriver.uvprojx tests/current_sense tests/motor_control tests/stage4_bench.py tests/stage5_bench.py
```

Expected: no output and exit 0. Hardware mapping headers, board initialization comments, mapping test, logs, and documentation are intentionally excluded.

- [ ] **Step 8: Commit the audited squash transplant**

Run:

```powershell
git add -A
git diff --cached --check
git commit -m "feat: backport verified current loop to v1 hardware" -m "Validated control source: 694e644; V1 BSP mapping retained."
```

Expected: one commit on `compare/v1-current-loop`; commit message body or notes identify `694e644` as the validated control source.

---

### Task 4: Verify the complete V1 software result

**Files:**
- Generated/ignored: `build/Debug/**`
- Generated/ignored: `project/MDK_V5/objects/**`, `project/MDK_V5/listings/**`, `project/MDK_V5/keil_build.log`
- No tracked source modifications expected.

**Interfaces:**
- Consumes: committed V1 comparison tree from Task 3.
- Produces: host-test, target-build, mapping, and repository-cleanliness evidence required before any V1 flash.

- [ ] **Step 1: Run every repository static test**

Run:

```powershell
Get-ChildItem tests -Recurse -Filter '*_static.py' | Sort-Object FullName | ForEach-Object { python $_.FullName; if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE } }
```

Expected: every script exits 0, including `pwm mapping static tests passed` with the V1 contract.

- [ ] **Step 2: Run the Stage 5 bench-script unit tests and syntax check**

Run:

```powershell
python tests/motor_control/test_stage5_bench.py
python -m py_compile tests/stage5_bench.py
```

Expected: `stage5 bench tests passed`; `py_compile` exits 0. The test intentionally prints a fake `=== FAIL: boom ===` while verifying cleanup behavior and still exits 0.

- [ ] **Step 3: Run strict host C tests under WSL**

Run:

```powershell
wsl.exe -e bash -lc "set -euo pipefail; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/v1-current-loop; mkdir -p /tmp/mps-v1-tests; gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control tests/current_sense/test_current_reconstruction.c application/motor_control/current_reconstruction.c -lm -o /tmp/mps-v1-tests/test_current_reconstruction; /tmp/mps-v1-tests/test_current_reconstruction; gcc -std=c11 -Wall -Wextra -Werror tests/motor_control/test_current_loop.c -lm -o /tmp/mps-v1-tests/test_current_loop; /tmp/mps-v1-tests/test_current_loop; gcc -std=c11 -Wall -Wextra -Werror tests/motor_control/test_speed_loop.c -lm -o /tmp/mps-v1-tests/test_speed_loop; /tmp/mps-v1-tests/test_speed_loop"
```

Expected: reconstruction passes, current loop reports 6 tests passed, speed loop reports 5 tests passed.

- [ ] **Step 4: Configure and clean-build the GNU Arm target**

Run:

```powershell
wsl.exe -e bash -lc "set -euo pipefail; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/v1-current-loop; cmake -S . -B build/Debug -DCMAKE_BUILD_TYPE=Debug; cmake --build build/Debug --clean-first -j2"
```

Expected: `[100%] Built target MPS_MotorDriver`; existing RT-Thread/FinSH macro warnings are allowed but new errors are not.

- [ ] **Step 5: Clean-build the Keil target**

Run from `project/MDK_V5`:

```powershell
cmd.exe /c build.bat clean
```

Expected: AXF and HEX are generated with `0 Error(s)`; record the warning count and program size exactly.

- [ ] **Step 6: Run the final source and branch audit**

Run:

```powershell
git diff --check main...HEAD
git diff --exit-code 694e644 -- CMakeLists.txt application/motor_app.c application/motor_shell.c application/motor_control platform/at32m412/current_sense_at32m412.c platform/at32m412/current_sense_at32m412.h platform/at32m412/motor_pwm_at32m412.c platform/at32m412/motor_pwm_at32m412.h project/MDK_V5/MPS_MotorDriver.uvprojx tests/current_sense tests/motor_control tests/stage4_bench.py tests/stage5_bench.py
git status --short --branch
```

Expected: diff checks exit 0 and the branch is clean.

---

### Task 5: Deliver the two-branch A/B test handoff

**Files:**
- No tracked changes unless actual V1 bench data is produced in a later hardware session.

**Interfaces:**
- Consumes: verified `compare/v1-current-loop` and existing V2 source evidence.
- Produces: unambiguous flash/test identities and a forum-ready measurement matrix.

- [ ] **Step 1: Record immutable branch identities**

Run:

```powershell
git rev-parse compare/v1-current-loop
git rev-parse feat/current-sampling-reconstruction
git diff compare/v1-current-loop feat/current-sampling-reconstruction -- platform/at32m412/board_motor_pins.h project/inc/at32m412_416_wk_config.h
```

Expected: two commit IDs and a diff showing V1 versus V2 mapping.

- [ ] **Step 2: Hand off the V1 hardware sequence without flashing automatically**

Use this order after the user connects the old board:

```text
1. Build/flash compare/v1-current-loop and confirm DISABLED, fault=0, EN=0.
2. Run mc_cal and require PASS with offset_valid=1.
3. Run mc_open 500 0 and require corrected Ia>0, Ib<0, Ic<0.
4. Run -50mA and +50mA current gates.
5. Run python tests/stage5_bench.py COM9 for all eight points.
6. Confirm FINAL SAFE and save the resulting V1 tests/stage5_bench_log.txt.
7. Flash the V2 control branch and repeat under the same supply, motor, calibration, and current limits.
```

- [ ] **Step 3: Prepare the forum comparison fields**

For both V1 and V2, report:

```text
branch/commit, board revision, bus voltage, supply current limit,
offset a/b/c, command Iq, measured Iq, measured Id,
sample tick, valid mask, recon phase, invalid/freeze counts,
fault flags, final DISABLED/EN/CCR state
```

Expected: the V1 column remains marked `NOT RUN` until real old-board data is collected; no V2 value is copied into it.
