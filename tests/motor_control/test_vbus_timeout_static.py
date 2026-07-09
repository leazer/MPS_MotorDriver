from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
ISR_C = ROOT / "application" / "motor_control" / "motor_control_isr.c"


def test_foc_isr_does_not_trigger_vbus_ordinary_conversion():
    source = ISR_C.read_text(encoding="utf-8")

    assert "current_sense_at32m412_read_vbus_raw()" not in source
    assert "current_sense_at32m412_read_vbus()" not in source


if __name__ == "__main__":
    test_foc_isr_does_not_trigger_vbus_ordinary_conversion()
    print("vbus timeout static tests passed")
