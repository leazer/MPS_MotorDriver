from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
TUNING_H = ROOT / "application" / "motor_control" / "motor_tuning.h"
TUNING_C = ROOT / "application" / "motor_control" / "motor_tuning.c"
CURRENT_C = ROOT / "application" / "motor_control" / "current_loop.c"
SPEED_C = ROOT / "application" / "motor_control" / "speed_loop.c"
POSITION_C = ROOT / "application" / "motor_control" / "position_loop.c"
CAN_MOTION_C = ROOT / "application" / "motor_control" / "can_motion_service.c"
ISR_C = ROOT / "application" / "motor_control" / "motor_control_isr.c"
APP_C = ROOT / "application" / "motor_app.c"
CMAKE = ROOT / "CMakeLists.txt"
MDK = ROOT / "project" / "MDK_V5" / "MPS_MotorDriver.uvprojx"


def read(path):
    return path.read_text(encoding="utf-8")


def function_body(source, name):
    match = re.search(rf"\b{re.escape(name)}\s*\([^;]*?\)\s*\{{",
                      source, re.DOTALL)
    assert match, f"missing function {name}"
    start = match.end() - 1
    depth = 0
    for index in range(start, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start + 1:index]
    raise AssertionError(f"unterminated function {name}")


def test_public_ram_blocks_and_initializer_exist():
    header = read(TUNING_H)
    source = read(TUNING_C)

    for token in (
        "motor_tuning_t",
        "motor_loop_debug_t",
        "g_motor_tuning",
        "g_motor_loop_debug",
        "motor_tuning_init",
    ):
        assert token in header
        assert token in source
    assert "volatile motor_tuning_t g_motor_tuning" in source
    assert "volatile motor_loop_debug_t g_motor_loop_debug" in source


def test_tuning_module_is_in_both_firmware_builds():
    cmake = read(CMAKE)
    project = read(MDK)
    app = read(APP_C)
    group = re.search(
        r"<Group>\s*<GroupName>application/motor_control</GroupName>"
        r"(?P<body>[\s\S]*?)</Group>",
        project,
    )

    assert "${CMAKE_SOURCE_DIR}/application/motor_control/motor_tuning.c" in cmake
    assert group
    assert "<FileName>motor_tuning.c</FileName>" in group.group("body")
    assert (
        r"<FilePath>..\..\application\motor_control\motor_tuning.c</FilePath>"
        in group.group("body")
    )
    init = function_body(app, "motor_app_init")
    assert init.index("motor_tuning_init();") < init.index("current_loop_init();")
    assert init.index("motor_tuning_init();") < init.index("speed_loop_init();")
    assert init.index("motor_tuning_init();") < init.index("position_loop_init();")


def test_all_runtime_consumers_read_live_ram():
    current_run = function_body(read(CURRENT_C), "current_loop_run")
    speed_run = function_body(read(SPEED_C), "speed_loop_run")
    position_run = function_body(read(POSITION_C), "position_loop_run")
    can_validate = read(CAN_MOTION_C)
    isr_tick = function_body(read(ISR_C), "motor_control_isr_tick")

    assert "g_motor_tuning.current" in current_run
    assert "g_motor_loop_debug.current" in current_run
    assert "g_motor_tuning.speed" in speed_run
    assert "g_motor_loop_debug.speed" in speed_run
    assert "g_motor_tuning.position" in position_run
    assert "g_motor_loop_debug.position" in position_run
    assert "g_motor_tuning.position" in can_validate
    assert "g_motor_tuning.protection" in isr_tick
    assert "g_motor_loop_debug.protection" in isr_tick

    assert "PID_CURRENT_OUT_LIMIT" not in current_run
    assert "PID_SPEED_KP_BRAKE" not in speed_run
    assert "PID_POSITION_KP" not in position_run
    assert "POSITION_MAX_ERROR_MDEG" not in position_run
    assert "IQ_OVERCURRENT_A" not in isr_tick
    assert "OVERCURRENT_DEBOUNCE_TICKS" not in isr_tick


if __name__ == "__main__":
    test_public_ram_blocks_and_initializer_exist()
    test_tuning_module_is_in_both_firmware_builds()
    test_all_runtime_consumers_read_live_ram()
    print("motor tuning static tests passed")
