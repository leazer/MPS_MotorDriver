from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
PARAMS_H = ROOT / "application" / "motor_control" / "motor_params.h"
ISR_C = ROOT / "application" / "motor_control" / "motor_control_isr.c"


def test_phase_overcurrent_uses_valid_frame_guard_and_short_debounce():
    params = PARAMS_H.read_text(encoding="utf-8")
    source = ISR_C.read_text(encoding="utf-8")
    assert re.search(r"#define\s+OVERCURRENT_DEBOUNCE_TICKS\s+4u", params)
    assert "current_sample_guard_step" in source
    assert "sample.frame_valid" in source
    assert "current_fault_latch(mc, FAULT_OVERCURRENT)" in source
    for phase in ("ia", "ib", "ic"):
        assert re.search(
            rf"fabsf\(sample\.{phase}\)\s*>=\s*"
            r"g_motor_tuning\.protection\.phase_overcurrent_A",
            source,
        )
    assert "g_motor_tuning.protection.overcurrent_debounce_ticks" in source
    assert "g_motor_tuning.protection.sample_invalid_limit" in source
    assert "g_motor_loop_debug.protection.overcurrent_active" in source


if __name__ == "__main__":
    test_phase_overcurrent_uses_valid_frame_guard_and_short_debounce()
    print("current fault debounce static tests passed")
