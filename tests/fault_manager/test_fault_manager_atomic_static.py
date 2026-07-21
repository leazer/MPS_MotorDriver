from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "application" / "motor_control" / "fault_manager.h"
SOURCE = ROOT / "application" / "motor_control" / "fault_manager.c"
SHELL = ROOT / "application" / "motor_shell.c"


def read(path):
    assert path.exists(), f"missing {path}"
    return path.read_text(encoding="utf-8")


def normalized(source):
    return re.sub(r"\s+", " ", source).strip()


def function_body(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1:index]
    raise AssertionError(f"unterminated {signature}")


def assert_atomic_manager_contract(source):
    compact = normalized(source)
    set_body = normalized(function_body(
        source, "void fault_manager_set_bits(uint32_t bits)"
    ))
    clear_body = normalized(function_body(
        source, "void fault_manager_clear_bits(uint32_t bits)"
    ))
    clear_all_body = normalized(function_body(
        source, "void fault_manager_clear_all(void)"
    ))

    for token in [
        "__get_PRIMASK()",
        "__disable_irq()",
        "__DMB()",
        "__enable_irq()",
        "atomic_fetch_or_explicit",
        "atomic_fetch_and_explicit",
        "atomic_store_explicit",
    ]:
        assert token in compact
    assert "fault_manager_irq_lock()" in set_body
    assert "s_fault_flags |= bits" in set_body
    assert "fault_manager_irq_unlock(primask)" in set_body
    assert "atomic_fetch_or_explicit" in set_body
    assert "fault_manager_irq_lock()" in clear_body
    assert "s_fault_flags &= ~bits" in clear_body
    assert "fault_manager_irq_unlock(primask)" in clear_body
    assert "atomic_fetch_and_explicit" in clear_body
    assert "fault_manager_irq_lock()" in clear_all_body
    assert "atomic_store_explicit" in clear_all_body


def assert_mutation_rejected(source, needle, replacement):
    assert needle in source
    mutated = source.replace(needle, replacement, 1)
    try:
        assert_atomic_manager_contract(mutated)
    except (AssertionError, ValueError):
        return
    raise AssertionError(f"atomic contract accepted mutation: {needle}")


def assert_shell_clear_gate(shell):
    gate = normalized(function_body(
        shell, "static bool motor_shell_reject_if_running(void)"
    ))
    clear_body = normalized(function_body(
        shell, "static void fault_clear(int argc, char **argv)"
    ))
    complete_gate = (
        "if (motor_control_get_state(mc) != MOTOR_CONTROL_STATE_DISABLED || "
        "motor_control_isr_open_loop_active() || "
        "motor_control_isr_align_active() || "
        "motor_control_isr_current_active() || "
        "motor_control_isr_speed_active() || "
        "motor_control_isr_position_active()) {"
    )
    assert complete_gate in gate
    assert "if (motor_shell_reject_if_running()) { return; }" in clear_body
    assert clear_body.index("motor_shell_reject_if_running") < clear_body.index(
        "fault_manager_clear_all();"
    )


def assert_shell_mutation_rejected(shell, needle, replacement):
    assert needle in shell
    mutated = shell.replace(needle, replacement, 1)
    try:
        assert_shell_clear_gate(mutated)
    except (AssertionError, ValueError):
        return
    raise AssertionError(f"shell clear gate accepted mutation: {needle}")


def test_manager_owns_arm_and_host_atomic_updates():
    header = read(HEADER)
    source = read(SOURCE)
    assert "void fault_manager_set_bits(uint32_t bits);" in header
    assert "void fault_manager_clear_bits(uint32_t bits);" in header
    assert "all-disabled" in header
    assert_atomic_manager_contract(source)
    for needle, replacement in [
        ("__disable_irq();", ""),
        ("fault_manager_irq_unlock(primask);", ""),
        ("atomic_fetch_or_explicit", "atomic_load_explicit"),
        ("atomic_fetch_and_explicit", "atomic_load_explicit"),
    ]:
        assert_mutation_rejected(source, needle, replacement)


def test_all_production_fault_writers_use_manager_owned_bit_updates():
    offenders = []
    for base in [ROOT / "application", ROOT / "platform", ROOT / "communication"]:
        for path in base.rglob("*.c"):
            if path == SOURCE:
                continue
            text = read(path)
            if re.search(r"\bfault_manager_(set|clear)\s*\(", text):
                offenders.append(str(path.relative_to(ROOT)))
    assert not offenders, offenders

    shell = read(SHELL)
    assert_shell_clear_gate(shell)
    for needle in [
        "motor_control_get_state(mc) != MOTOR_CONTROL_STATE_DISABLED",
        "motor_control_isr_open_loop_active()",
        "motor_control_isr_align_active()",
        "motor_control_isr_current_active()",
        "motor_control_isr_speed_active()",
        "motor_control_isr_position_active()",
    ]:
        assert_shell_mutation_rejected(shell, needle, "false")


if __name__ == "__main__":
    test_manager_owns_arm_and_host_atomic_updates()
    test_all_production_fault_writers_use_manager_owned_bit_updates()
    print("fault manager atomic static tests passed")
