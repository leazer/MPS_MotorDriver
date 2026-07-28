import re
from pathlib import Path


PARAMS = Path(__file__).parents[2] / "application/motor_control/motor_params.h"


def define(text, name):
    match = re.search(
        rf"^#define\s+{re.escape(name)}\s+([^\s/]+)",
        text,
        re.MULTILINE,
    )
    if match is None:
        raise AssertionError("missing define " + name)
    return match.group(1)


def main():
    text = PARAMS.read_text(encoding="utf-8")
    assert define(text, "PID_POSITION_KP") == "7.5f"
    assert define(text, "PID_SPEED_KP") == "0.01f"
    assert define(text, "PID_SPEED_KI") == "0.01f"
    assert define(text, "SPEED_IQ_LIMIT_A") == "0.5f"
    assert define(text, "POSITION_IQ_FRICTION_MOVING_A") == "0.04f"
    print("position tuning profile: PASS")


if __name__ == "__main__":
    main()
