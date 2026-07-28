from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "application" / "motor_control" / "position_loop.h"
SOURCE = ROOT / "application" / "motor_control" / "position_loop.c"
PARAMS = ROOT / "application" / "motor_control" / "motor_params.h"
CMAKE = ROOT / "CMakeLists.txt"
MDK = ROOT / "project" / "MDK_V5" / "MPS_MotorDriver.uvprojx"


def read(path):
    assert path.exists(), f"missing {path}"
    return path.read_text(encoding="utf-8")


def normalized(source):
    return re.sub(r"\s+", " ", source).strip()


def function_body(source, signature):
    start = source.index(signature)
    opening_brace = source.index("{", start)
    depth = 0
    for index in range(opening_brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening_brace + 1:index]
    raise AssertionError(f"unterminated function body for {signature}")


def assert_function_mutation_rejected(
    check, source, signature, needle, replacement
):
    start = source.index(signature)
    opening_brace = source.index("{", start)
    body = function_body(source, signature)
    offset = body.index(needle)
    mutation_start = opening_brace + 1 + offset
    mutated = (
        source[:mutation_start]
        + replacement
        + source[mutation_start + len(needle):]
    )
    try:
        check(mutated)
    except AssertionError:
        return
    raise AssertionError(
        f"contract checker accepted mutation in {signature}: {needle}"
    )


def assert_atomic_joint_transform_contract(source):
    init = normalized(function_body(source, "void position_loop_init(void)"))
    setter = normalized(function_body(
        source,
        "bool position_loop_set_joint_origin(int32_t sensor_mdeg,",
    ))
    lock = normalized(function_body(
        source, "static uint32_t position_loop_origin_lock(void)"
    ))
    unlock = normalized(function_body(
        source, "static void position_loop_origin_unlock(uint32_t primask)"
    ))

    assert "__get_PRIMASK()" in lock
    assert "__disable_irq();" in lock
    assert "__DMB();" in unlock
    assert "__enable_irq();" in unlock

    setter_order = [
        "position_loop_reset();",
        "primask = position_loop_origin_lock();",
        "s_sensor_anchor_mdeg = sensor_mdeg;",
        "s_joint_anchor_mdeg = joint_mdeg;",
        "s_joint_direction = joint_direction;",
        "s_origin_valid = 1u;",
        "s_position_snapshot.origin_valid = 1u;",
        "position_loop_origin_unlock(primask);",
    ]
    for token in setter_order:
        assert token in setter
    setter_indices = [setter.index(token) for token in setter_order]
    assert setter_indices == sorted(setter_indices)

    assert "primask = position_loop_origin_lock();" in init
    assert "s_origin_valid = 0u;" in init
    assert "s_position_snapshot.origin_valid = 0u;" in init
    assert "position_loop_origin_unlock(primask);" in init
    assert init.index("primask = position_loop_origin_lock();") < init.index(
        "s_origin_valid = 0u;"
    ) < init.index("position_loop_origin_unlock(primask);")


def test_position_loop_public_contract():
    header = read(HEADER)
    for token in [
        "position_setpoint_t",
        "position_loop_snapshot_t",
        "position_loop_init",
        "position_loop_reset",
        "position_loop_set_origin",
        "position_loop_set_joint_origin",
        "position_loop_joint_direction",
        "position_loop_origin_valid",
        "position_loop_sensor_to_joint_mdeg",
        "position_loop_control_to_joint_velocity_mdeg_s",
        "position_loop_first_target_safe",
        "position_loop_submit",
        "position_loop_run",
        "position_loop_get_iq_feedforward_A",
        "position_loop_get_snapshot",
    ]:
        assert token in header, token


def test_position_loop_has_coherent_publish_and_bounded_extrapolation():
    source = read(SOURCE)
    params = read(PARAMS)
    assert "s_publish_generation" in source
    assert "generation_before" in source
    assert "generation_after" in source
    assert "g_motor_tuning.position.extrapolation_limit_ms" in source
    assert "g_motor_tuning.position.max_error_mdeg" in source
    assert "g_motor_tuning.position.max_velocity_mdeg_s" in source
    assert "g_motor_tuning.position.speed_limit_elec_rad_s" in source
    assert "g_motor_tuning.position.iq_friction_A" in source
    assert "g_motor_tuning.position.iq_friction_moving_A" in source
    assert "g_motor_tuning.position.iq_friction_error_mdeg" in source
    assert "POSITION_SPEED_LIMIT_ELEC_RAD_S" in params
    assert "POSITION_LOOP_DIV" in params
    assert "POSITION_IQ_FRICTION_A" in params
    assert "POSITION_IQ_FRICTION_MOVING_A" in params
    assert "POSITION_IQ_FRICTION_ERROR_MDEG" in params


def test_joint_transform_is_published_atomically_against_timer_isrs():
    source = read(SOURCE)
    assert_atomic_joint_transform_contract(source)

    for signature, needle, replacement in [
        (
            "bool position_loop_set_joint_origin(int32_t sensor_mdeg,",
            "primask = position_loop_origin_lock();",
            "primask = 0u;",
        ),
        (
            "bool position_loop_set_joint_origin(int32_t sensor_mdeg,",
            "s_joint_direction = joint_direction;",
            "s_joint_direction = 1;",
        ),
        (
            "bool position_loop_set_joint_origin(int32_t sensor_mdeg,",
            "position_loop_origin_unlock(primask);",
            "",
        ),
        (
            "void position_loop_init(void)",
            "primask = position_loop_origin_lock();",
            "primask = 0u;",
        ),
    ]:
        assert_function_mutation_rejected(
            assert_atomic_joint_transform_contract,
            source,
            signature,
            needle,
            replacement,
        )


def test_snapshot_reader_retries_odd_generation_before_comparing_samples():
    source = read(SOURCE)
    reader = source.split("bool position_loop_get_snapshot", 1)[1]

    assert "for (;;)" in reader
    assert "if (generation_before == generation_after &&" in reader
    assert "return true;" in reader


def test_position_loop_uses_mechanical_units_and_pole_pairs_once():
    source = read(SOURCE)
    assert "180000.0f" in source
    assert "MOTOR_POLE_PAIRS" in source
    assert source.count("(float)MOTOR_POLE_PAIRS") == 1
    assert "speed_mech_rad_s * (float)s_joint_direction" in source


def test_position_loop_is_in_both_target_builds():
    assert "application/motor_control/position_loop.c" in read(CMAKE)
    mdk = read(MDK)
    assert "<FileName>position_loop.c</FileName>" in mdk
    assert "..\\..\\application\\motor_control\\position_loop.c" in mdk


if __name__ == "__main__":
    test_position_loop_public_contract()
    test_position_loop_has_coherent_publish_and_bounded_extrapolation()
    test_joint_transform_is_published_atomically_against_timer_isrs()
    test_snapshot_reader_retries_odd_generation_before_comparing_samples()
    test_position_loop_uses_mechanical_units_and_pole_pairs_once()
    test_position_loop_is_in_both_target_builds()
    print("position loop static tests passed")
