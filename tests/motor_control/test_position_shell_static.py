from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SHELL = ROOT / "application" / "motor_shell.c"
PARAMS = ROOT / "application" / "motor_control" / "motor_params.h"


def read(path):
    assert path.exists(), f"missing {path}"
    return path.read_text(encoding="utf-8")


def command_body(source, name, next_marker):
    start = source.index(f"static void {name}(")
    return source[start:source.index(next_marker, start)]


def test_position_shell_commands_are_exported():
    shell = read(SHELL)
    for name in ["mc_pos_zero", "mc_pos", "mc_pos_stream", "mc_pos_status"]:
        assert f"static void {name}" in shell
        assert f"MSH_CMD_EXPORT({name}" in shell


def test_joint_zero_is_disabled_only_and_uses_control_position():
    shell = read(SHELL)
    body = command_body(shell, "mc_pos_zero", "MSH_CMD_EXPORT(mc_pos_zero")
    assert "motor_shell_reject_if_running" in body
    assert "encoder_service_get_control_position_mdeg" in body
    assert "position_loop_set_origin" in body


def test_static_and_stream_commands_use_distinct_leases():
    shell = read(SHELL)
    static_body = command_body(shell, "mc_pos", "MSH_CMD_EXPORT(mc_pos")
    stream_body = command_body(shell, "mc_pos_stream", "MSH_CMD_EXPORT(mc_pos_stream")
    assert "lease_ms = 0u" in static_body
    assert "velocity_mdeg_s = 0" in static_body
    assert "lease_ms = POSITION_STREAM_LEASE_MS" in stream_body
    assert "motor_control_isr_position_start" in shell
    assert "motor_control_isr_position_submit" in shell
    assert "POSITION_COMMAND_LIMIT_MDEG" in shell
    assert "POSITION_MAX_VELOCITY_MDEG_S" in shell


def test_compact_position_status_is_bounded_and_checksummed():
    shell = read(SHELL)
    body = command_body(shell, "mc_pos_status", "MSH_CMD_EXPORT(mc_pos_status")
    assert "0x504F5331u" in body
    assert 'ps a=%d t=%ld v=%ld r=%ld m=%ld e=%ld ' in body
    assert 'f=%08X k=%08X\\n' in body
    worst = (
        "ps a=1 t=-180000 v=-60000 r=-180000 m=-180000 e=-30000 "
        "w=-20943 x=-20943 q=-500 g=100 o=1 n=65535 "
        "f=00000100 k=FFFFFFFF"
    )
    assert len(worst) < 128


def test_stop_clears_position_mode_before_other_modes():
    shell = read(SHELL)
    body = command_body(shell, "mc_stop", "MSH_CMD_EXPORT(mc_stop")
    assert "motor_control_isr_position_stop" in body
    assert body.index("motor_control_isr_position_stop") < body.index(
        "motor_control_isr_speed_stop"
    )


def test_position_command_bounds_and_stream_lease_are_explicit():
    params = read(PARAMS)
    assert "#define POSITION_COMMAND_LIMIT_MDEG" in params
    assert "#define POSITION_STREAM_LEASE_MS" in params


if __name__ == "__main__":
    test_position_shell_commands_are_exported()
    test_joint_zero_is_disabled_only_and_uses_control_position()
    test_static_and_stream_commands_use_distinct_leases()
    test_compact_position_status_is_bounded_and_checksummed()
    test_stop_clears_position_mode_before_other_modes()
    test_position_command_bounds_and_stream_lease_are_explicit()
    print("position shell static tests passed")
