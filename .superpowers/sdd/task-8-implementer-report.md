# Task 8 Implementer Report

## Status

Implemented checked CAN diagnostics, owner-scoped atomic diagnostic resets, a
strict bench parser/metric library, and deterministic injected-adapter cleanup.
No powered motor, CAN bus, serial port, Flash, or hardware execution was used.

## RED / GREEN

Tests were written before production changes.

- Initial parser RED: `python tests/motor_control/test_stage8_can_bench.py`
  exited 1 because `tests/stage8_can_bench.py` did not exist.
- Initial shell RED: `python tests/motor_control/test_can_motion_integration_static.py`
  exited 1 because `static void can_status(` did not exist.
- Cleanup setup RED: after the first GREEN cycle, a regression for COM-open
  failure exited 1 because peer STOP was never attempted. Opening the owned
  serial adapter was moved inside the guarded region; the current suite proves
  all three peer STOP attempts run even when COM setup raises.
- GREEN: parser/metrics/cleanup, integration/mutation, driver static, production
  service, test-macro service, and CAN protocol suites all pass.

## Implemented contract

- `can_status` prints exactly `id s se p a pa sa rx tx pe ro bo te f k`, with
  eight-digit uppercase fault/checksum fields. Checksum is seed `0x43414E31`
  XOR every preceding printed numeric value in order as `uint32_t`. The brief's
  illustrative values compute to `43415C89`, not the illustrative `89ABCDEF`.
- The strict Python parser accepts one complete ordered ASCII line (optionally
  terminated by CRLF) and rejects missing, duplicate, extra, malformed,
  truncated, lowercase/bad hex, checksum mutations, width overflow, node IDs
  outside 1..2, and states outside 0..5.
- Worst-case printed line is 162 bytes including translated CRLF, below the
  USART1 translated-DMA stage size of 256 bytes.
- The motion snapshot is protected against TMR6 updates. Motion reset clears
  only RX, TX, TX-failure, and protocol counters. The driver reset masks IRQs
  and clears cumulative RX/TX/IRQ/overflow/bus-off-event/error diagnostics,
  preserving REC/TEC, error-passive, fatal/bus-off latches, node, and queues.
- Shell reset requires every motor mode stopped, motor state DISABLED, PWM EN
  low, and motion-node state exactly READY before calling owner reset APIs.
- Bench helpers validate and compute point rate, wrap-aware missing/duplicate
  sequences, static/reversal/sine errors, peak absolute Iq, HOLD/fatal latency,
  and driver-counter deltas.
- Cleanup always attempts peer STOP exactly three times, then explicit COM9
  `mc_stop`, then the Stage 7 final-safe-state contract. Each step continues
  after errors. Primary setup/workload/metric/assert exceptions are re-raised
  with cleanup details; cleanup-only failures raise `BenchCleanupError`.
  Only serial resources opened by the harness are closed.

## Verification

- `python tests/motor_control/test_stage8_can_bench.py`: PASS.
- `python tests/motor_control/test_can_motion_integration_static.py`: PASS,
  including checksum-seed and READY-gate mutation rejection.
- `python tests/communication/test_can_at32m412_static.py`: PASS (21 tests).
- WSL production `test_can_motion_service`: PASS, exit 0.
- WSL `CAN_MOTION_SERVICE_TEST` reset/saturation service: PASS, exit 0.
- WSL `test_can_protocol`: `can protocol: PASS`, exit 0.
- `python -m py_compile` for the new bench and test: exit 0.
- `git diff --check`: exit 0.
- Keil ARMCC5 clean rebuild: Code 56916, RO-data 6512, RW-data 696,
  ZI-data 10728; 0 errors, 0 warnings.

The local Python installation does not provide the optional `pytest` module;
all required focused Python files were executed through their direct test
entrypoints instead.

## Files

- `application/motor_shell.c`
- `application/motor_control/can_motion_service.h/.c`
- `communication/can_at32m412.h/.c`
- `tests/stage8_can_bench.py`
- `tests/motor_control/test_stage8_can_bench.py`
- `tests/motor_control/test_can_motion_integration_static.py`
- `tests/motor_control/test_can_motion_service.c`
- `tests/communication/test_can_at32m412_static.py`

## Concerns

This task intentionally supplies an adapter-neutral software harness and fake
tests only. A concrete CAN-peer qualification callback and powered Node 1 bench
evidence remain Task 10 work. The mixed-instant driver telemetry remains as
documented; its copied values are stable for formatting/checksum, while safety
continues to use the independent fatal latch.

## External Review Correction

External review after commit `7bbbc58` identified three gaps. Each correction
was test-driven and is included in the follow-up commit recorded by Git history.

### Additional RED evidence

- Driver getter RED: CAN static exited 1 because
  `can_at32m412_get_diag()` still called `can_snapshot_error_state()`.
- Independent PWM cleanup RED: Stage 8 tests exited 1 because
  `parse_pwm_info()` did not exist and cleanup delegated to an opaque Stage 7
  composite verifier.
- Atomic gate/action RED: integration static exited 1 because there was no
  single `motor_shell_can_diag_reset_if_safe()` critical-span helper.
- Full read-only chain RED: the strengthened transitive checker exited 1
  because `can_motion_service_get_snapshot()` called the mutating
  `read_faults()` refresh helper.
- ARMCC declaration RED: the first clean review-fix build reported two
  implicit-declaration warnings for `rt_hw_interrupt_disable/enable`; a static
  include contract then failed until `rthw.h` was included.

### Corrections and mutation coverage

- CAN status is now semantically read-only through both snapshot paths. The
  driver getter copies only ISR-maintained fields; motion snapshot copies its
  already-maintained fault field. Status/error IRQ paths retain hardware error
  refresh responsibility. Function-scoped and transitive mutation tests reject
  reintroducing refresh or diagnostic writes.
- Cleanup now independently sends `mc_stop`, then always sends `pwm_info` and
  parses/asserts EN=0, CCR1/2/3=2812, and CCR4=5264. Fake serial tests exercise
  failed STOP, failed/unsafe PWM query, three failing peer STOP attempts,
  cleanup-only aggregation, primary-exception preservation, and owned close.
- One short RT interrupt critical span now checks control state, all five mode
  flags, PWM EN, and exact motion READY before calling both owner reset APIs.
  There is no logging/blocking in the span. Mutation tests reject READY changes
  and either reset escaping the single disable/enable span.

### Review-fix verification

- Stage 8 parser/metrics/direct-PWM cleanup: PASS.
- CAN integration/transitive/critical-span mutations: PASS.
- CAN driver static: PASS (22 tests).
- WSL service production: PASS, exit 0.
- WSL service test-macro reset/saturation: PASS, exit 0.
- WSL CAN protocol: PASS, exit 0.
- Keil ARMCC5 clean: Code 57060, RO-data 6512, RW-data 696,
  ZI-data 10728; 0 errors, 0 warnings.
