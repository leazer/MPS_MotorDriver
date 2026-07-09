from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
PARAMS_H = ROOT / "application" / "motor_control" / "motor_params.h"
ISR_C = ROOT / "application" / "motor_control" / "motor_control_isr.c"


def test_phase_overcurrent_uses_short_debounce():
    params = PARAMS_H.read_text(encoding="utf-8")
    source = ISR_C.read_text(encoding="utf-8")

    assert "#define OVERCURRENT_DEBOUNCE_TICKS" in params
    assert "s_oc_consec" in source
    assert re.search(
        r"if\s*\(s_oc_consec\s*>=\s*OVERCURRENT_DEBOUNCE_TICKS\)\s*{[^{}]*fault_manager_set\(FAULT_OVERCURRENT\);",
        source,
        re.DOTALL,
    )


if __name__ == "__main__":
    test_phase_overcurrent_uses_short_debounce()
    print("current fault debounce static tests passed")
