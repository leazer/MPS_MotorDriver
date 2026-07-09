from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
ISR_C = ROOT / "application" / "motor_control" / "motor_control_isr.c"


def test_vbus_faults_are_not_latched_from_foc_isr():
    source = ISR_C.read_text(encoding="utf-8")

    assert "fault_manager_set(FAULT_UNDERVOLTAGE)" not in source
    assert "fault_manager_set(FAULT_OVERVOLTAGE)" not in source


if __name__ == "__main__":
    test_vbus_faults_are_not_latched_from_foc_isr()
    print("vbus fault static tests passed")
