# Final review fix report

- Date: 2026-07-15 (Asia/Shanghai)
- Worktree: `E:\WorkSpaces\2_MotorDriver\MPS_MotorDriver\.worktrees\current-sampling-reconstruction`
- Scope: all five findings from `final-fix-brief.md`
- Hardware actions: none. No COM port was opened, no firmware was flashed, and no motor command was issued.

## Result

All requested fixes are implemented and all software gates pass. The firmware build succeeds. The only concern is the existing RT-Thread FinSH export-macro warnings emitted when the modified shell translation unit is rebuilt; no warning points to the new logic.

## Implemented fixes

1. All four start APIs now return `-3` before mutation when the control state is already `ENABLED`. Starts remain permitted from `DISABLED`, and from `FAULT` only after fatal flags are clear. Repeated `motor_pwm_at32m412_enable_ovf_irq()` calls now return before tracker rearm.
2. Raw and reconstructed currents, ADC values, validity/reconstruction metadata, sampling plan/margins, protection counters, invalid/overcurrent guard counters, and PI-freeze count are published together through an internal odd/even sequence snapshot with `__DMB()` barriers. The shell-facing getter retries until it reads an unchanged even sequence.
3. Offset calibration now clears and waits for `ADC_PCCE_FLAG` for each of 1024 distinct preempt-sequence completions, uses a finite timeout, validates candidates before committing, and preserves prior offsets/validity on failure. `mc_cal` emits an authoritative `mc_cal result: PASS|FAIL offset_valid=...` line.
4. The Stage 5 bench parser requires calibration success and valid offsets before any `mc_cur` command. Its `main()` returns nonzero after assertions or exceptions while still attempting `mc_stop` and serial close.
5. Phase overcurrent uses the exact `>= IQ_OVERCURRENT_A` boundary.

## TDD evidence

The required TDD skill was read before production edits. The four preserved test files were inspected before execution. Two `.index()` checks and one assignment substring were repaired because they obscured missing behavior with `ValueError`/a false positive; the tests' intended assertions were not weakened.

### Runner availability

Attempted command:

```powershell
python -m pytest tests/motor_control/test_current_fault_debounce_static.py tests/motor_control/test_current_sampling_static.py tests/motor_control/test_final_fix_static.py tests/motor_control/test_stage5_bench.py -q
```

Environment result (not counted as RED):

```text
No module named pytest
```

The repository tests are standalone Python scripts, so they were run directly.

### RED

Commands:

```powershell
python tests/motor_control/test_current_fault_debounce_static.py
python tests/motor_control/test_current_sampling_static.py
python tests/motor_control/test_final_fix_static.py
python tests/motor_control/test_stage5_bench.py
```

Expected failures observed:

```text
AssertionError: fabsf(sample.<phase>) >= IQ_OVERCURRENT_A missing
AssertionError: if (s_ovf_irq_enabled) missing
AssertionError: motor_control_isr_open_loop_start does not reject a live mode switch
AssertionError: missing calibration result parser
```

During review of the initial GREEN implementation, a retry-loop hazard was identified: `continue` in a `do/while` reader could evaluate a condition containing uninitialized `sequence_end` when the sequence was odd. A regression was added first:

```powershell
python tests/motor_control/test_current_sampling_static.py
```

RED result:

```text
AssertionError: assert "for (;;)" in getter
```

The reader was then changed to an explicit infinite retry loop.

### Focused GREEN

Commands and results:

```text
python tests/motor_control/test_current_fault_debounce_static.py
  current fault debounce static tests passed
python tests/motor_control/test_current_sampling_static.py
  current sampling static tests passed
python tests/motor_control/test_final_fix_static.py
  final fix static tests passed
python tests/motor_control/test_stage5_bench.py
  === FAIL: boom ===
  stage5 bench tests passed
python -m py_compile tests/stage5_bench.py
  exit 0
```

The `=== FAIL: boom ===` line is the deliberately injected exception whose nonzero status and cleanup behavior the host test verifies; the test script itself exits 0.

## Full Python static suite

Command:

```powershell
$tests = Get-ChildItem tests\motor_control\test_*_static.py, tests\encoder_service\test_*_static.py
foreach ($test in $tests) {
    python $test.FullName
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
python tests\test_pwm_mapping_static.py
```

Result: exit 0, 12/12 scripts passed.

```text
test_calibration_shell_static.py             PASS
test_current_fault_debounce_static.py        PASS
test_current_loop_tuning_static.py           PASS
test_current_phase_mapping_static.py         PASS
test_current_sampling_static.py              PASS
test_final_fix_static.py                     PASS
test_speed_loop_static.py                    PASS
test_vbus_fault_static.py                    PASS
test_vbus_timeout_static.py                  PASS
test_encoder_service_static.py               PASS
test_encoder_speed_static.py                 PASS
test_pwm_mapping_static.py                   PASS
```

## Strict host C tests

Command:

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/current-sampling-reconstruction; gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control tests/current_sense/test_current_reconstruction.c application/motor_control/current_reconstruction.c -lm -o /tmp/test_current_reconstruction; /tmp/test_current_reconstruction; gcc -std=c11 -Wall -Wextra -Werror tests/motor_control/test_current_loop.c -lm -o /tmp/test_current_loop; /tmp/test_current_loop; gcc -std=c11 -Wall -Wextra -Werror tests/motor_control/test_speed_loop.c -lm -o /tmp/test_speed_loop; /tmp/test_speed_loop"
```

The sandboxed WSL launch was denied with `Wsl/Service/CreateInstance/E_ACCESSDENIED`; the identical approved command then passed:

```text
current reconstruction: all tests passed
6 current_loop tests passed
5 speed_loop tests passed
```

Result: 3/3 executables passed; strict host compiler warnings: 0.

## Firmware build and size

Command:

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/current-sampling-reconstruction; cmake --build build/Debug; arm-none-eabi-size build/Debug/MPS_MotorDriver.elf"
```

Result:

```text
[100%] Built target MPS_MotorDriver
Memory region         Used Size  Region Size  %age Used
           FLASH:       73116 B       127 KB     56.22%
             RAM:       11360 B        16 KB     69.34%

   text    data     bss     dec     hex filename
  72808     304   11064   84176   148d0 build/Debug/MPS_MotorDriver.elf
```

Warnings: 52 existing FinSH macro warnings were emitted while recompiling `motor_shell.c`: 26 incompatible function-pointer cast warnings and 26 ISO C extra top-level semicolon warnings, one pair per existing `MSH_CMD_EXPORT`. No warning identifies the new calibration result line or any other new logic.

## Integrity and scope checks

```powershell
git diff --check
```

Result before report creation: exit 0, no whitespace errors. The generated `tests/stage5_bench_log.txt` change from the focused test was restored, and the test now mocks report-file opening so future host runs do not modify the tracked hardware log.

No hardware evidence is claimed. Physical scope/bench validation remains outside this final software-fix wave and was explicitly prohibited for this task.
