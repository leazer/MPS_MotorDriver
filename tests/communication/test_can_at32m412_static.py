from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "communication" / "can_at32m412.h"
SOURCE = ROOT / "communication" / "can_at32m412.c"
INTERRUPTS = ROOT / "project" / "src" / "at32m412_416_int.c"
MOTOR_APP = ROOT / "application" / "motor_app.c"


def read(path):
    assert path.exists(), f"missing {path}"
    return path.read_text(encoding="utf-8")


def function_body(source, name):
    match = re.search(rf"\b{re.escape(name)}\s*\([^;]*?\)\s*\{{", source, re.DOTALL)
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


def block_after(source, marker):
    assert marker in source, f"missing block marker: {marker}"
    marker_index = source.index(marker)
    start = source.index("{", marker_index)
    depth = 0
    for index in range(start, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start + 1:index], start, index
    raise AssertionError(f"unterminated block after {marker}")


def assert_diagnostic_getter_read_only(source):
    getter = function_body(source, "can_at32m412_get_diag")
    assert "can_snapshot_error_state" not in getter
    assert "can_sat_increment" not in getter
    assert not re.search(r"\bs_diag\s*\.[A-Za-z0-9_]+\s*(?:=|\+\+|--)", getter)
    for mutator in ("can_at32m412_reset_diagnostics", "can_busoff_reset"):
        assert mutator not in getter


def test_public_api_and_diagnostics_contract():
    header = read(HEADER)
    declarations = (
        r"bool\s+can_at32m412_init\s*\(\s*uint8_t\s+node_id\s*\)\s*;",
        r"bool\s+can_at32m412_rx_pop\s*\(\s*can_frame_t\s*\*\s*out\s*\)\s*;",
        r"bool\s+can_at32m412_tx_push\s*\(\s*const\s+can_frame_t\s*\*\s*frame\s*\)\s*;",
        r"void\s+can_at32m412_tx_kick\s*\(\s*void\s*\)\s*;",
        r"void\s+can_at32m412_get_diag\s*\(\s*can_at32m412_diag_t\s*\*\s*out\s*\)\s*;",
        r"void\s+can_at32m412_reset_diagnostics\s*\(\s*void\s*\)\s*;",
        r"bool\s+can_at32m412_fatal_bus_error\s*\(\s*void\s*\)\s*;",
        r"void\s+can_at32m412_irq_rx\s*\(\s*void\s*\)\s*;",
        r"void\s+can_at32m412_irq_tx\s*\(\s*void\s*\)\s*;",
        r"void\s+can_at32m412_irq_status\s*\(\s*void\s*\)\s*;",
        r"void\s+can_at32m412_irq_error\s*\(\s*void\s*\)\s*;",
    )
    assert '#include "can_frame.h"' in header
    assert "can_at32m412_diag_t" in header
    for declaration in declarations:
        assert re.search(declaration, header), declaration
    for field in (
        "rec", "tec", "error_passive", "bus_off_latched", "fatal_latched",
        "rx_received", "rx_rejected", "rx_overflow", "tx_queued",
        "tx_completed", "tx_rejected", "status_irqs", "error_irqs",
        "bus_off_events",
    ):
        assert re.search(rf"\b{field}\s*;", header), f"missing diagnostic field {field}"


def test_runtime_diagnostic_reset_is_atomic_and_preserves_live_safety_state():
    source = read(SOURCE)
    body = function_body(source, "can_at32m412_reset_diagnostics")
    for field in (
        "rx_received", "rx_rejected", "rx_overflow", "tx_queued",
        "tx_completed", "tx_rejected", "tx_errors", "status_irqs",
        "error_irqs", "bus_off_events",
    ):
        assert f"s_diag.{field} = 0u;" in body
    for field in ("rec", "tec", "error_passive", "bus_off_latched",
                  "fatal_latched"):
        assert f"s_diag.{field} =" not in body
    for field in ("s_node_id", "s_rx_head", "s_rx_tail", "s_tx_head",
                  "s_tx_tail", "s_tx_active"):
        assert f"{field} =" not in body
    assert "__get_PRIMASK()" in body
    assert "__disable_irq();" in body
    assert body.index("__disable_irq();") < body.index("s_diag.rx_received = 0u;")
    assert "__DMB();" in body


def test_bus_off_event_counter_is_edge_triggered_and_saturating():
    source = read(SOURCE)
    snapshot = function_body(source, "can_snapshot_error_state")
    assert "if (!s_diag.bus_off_latched)" in snapshot
    assert "can_sat_increment(&s_diag.bus_off_events);" in snapshot
    assert snapshot.index("if (!s_diag.bus_off_latched)") < snapshot.index(
        "s_diag.bus_off_latched = true;"
    )


def test_init_rejects_invalid_nodes_before_hardware_and_configures_pins_clocks():
    body = function_body(read(SOURCE), "can_at32m412_init")
    guard = body.index("node_id != 1u")
    assert "node_id != 2u" in body[guard:]
    assert body.index("return false", guard) < body.index("crm_periph_clock_enable")
    for token in (
        "CRM_GPIOA_PERIPH_CLOCK", "CRM_CAN1_PERIPH_CLOCK",
        "crm_can_clock_select(CRM_CAN1, CRM_CAN_CLOCK_SOURCE_PLL)",
        "GPIO_PINS_SOURCE12, GPIO_MUX_9", "GPIO_PINS_SOURCE11, GPIO_MUX_9",
        "GPIO_PINS_12 | GPIO_PINS_11", "GPIO_MODE_MUX",
    ):
        assert token in body, token


def test_exact_one_megabit_normal_mode_and_bounded_retransmission_setup():
    body = function_body(read(SOURCE), "can_at32m412_init")
    assignments = {
        "bittime.bittime_div": "10u",
        "bittime.ac_bts1_size": "14u",
        "bittime.ac_bts2_size": "4u",
        "bittime.ac_rsaw_size": "2u",
    }
    for lhs, rhs in assignments.items():
        assert re.search(rf"{re.escape(lhs)}\s*=\s*{rhs}\s*;", body), lhs
    assert "can_bittime_set(CAN1, &bittime)" in body
    assert "can_mode_set(CAN1, CAN_MODE_COMMUNICATE)" in body
    assert "can_retransmission_limit_set(CAN1, CAN_RE_TRANS_TIMES_UNLIMIT)" in body
    assert "can_rearbitration_limit_set(CAN1, CAN_RE_ARBI_TIMES_UNLIMIT)" in body
    assert "can_rxbuf_overflow_mode_set(CAN1, CAN_RXBUF_OVERFLOW_BE_LOSE)" in body


def test_hardware_filters_are_exact_standard_data_dlc8_ids():
    source = read(SOURCE)
    helper = function_body(source, "can_configure_exact_filter")
    init = function_body(source, "can_at32m412_init")
    for token in (
        "code_para.id = id", "code_para.id_type = CAN_ID_STANDARD",
        "code_para.frame_type = CAN_FRAME_DATA", "code_para.data_length = CAN_DLC_BYTES_8",
        "mask_para.id = 0x7ffu", "mask_para.id_type = TRUE",
        "mask_para.frame_type = TRUE", "mask_para.data_length = 0x0fu",
        "can_filter_set(CAN1, filter_number, &filter)",
        "can_filter_enable(CAN1, filter_number, TRUE)",
    ):
        assert token in helper, token
    assert "can_configure_exact_filter(CAN_FILTER_NUM_0, 0x080u)" in init
    assert re.search(
        r"can_configure_exact_filter\s*\(\s*CAN_FILTER_NUM_1\s*,\s*"
        r"\(uint16_t\)\(0x100u \+ node_id\)\s*\)",
        init,
    )


def test_init_disables_all_filters_before_enabling_exactly_two():
    init = function_body(read(SOURCE), "can_at32m412_init")
    disable_loop, loop_start, loop_end = block_after(
        init, "for (filter_number = 0u; filter_number < 16u; ++filter_number)"
    )
    assert "can_filter_enable(CAN1, (can_filter_type)filter_number, FALSE)" in disable_loop
    assert "TRUE" not in disable_loop
    configured = list(re.finditer(r"can_configure_exact_filter\s*\(", init))
    assert len(configured) == 2, "init must enable exactly two acceptance filters"
    assert all(match.start() > loop_end for match in configured), \
        "all hardware filters must be disabled before either exact filter is enabled"
    assert not re.search(r"can_filter_enable\s*\([^;]*TRUE", init), \
        "init must enable filters only through the exact-filter helper"


def test_edge_triggered_interrupts_are_enabled_at_priority_three():
    body = function_body(read(SOURCE), "can_at32m412_init")
    for interrupt in ("CAN_RIE_INT", "CAN_TPIE_INT", "CAN_ROIE_INT"):
        assert interrupt in body
    for irq in ("CAN1_RX_IRQn", "CAN1_TX_IRQn", "CAN1_STAT_IRQn", "CAN1_ERR_IRQn"):
        assert re.search(rf"nvic_irq_enable\s*\(\s*{irq}\s*,\s*3u\s*,\s*0u?\s*\)", body), irq


def test_software_rings_have_eight_usable_slots_and_spsc_cursors():
    source = read(SOURCE)
    capacities = {}
    for name in ("CAN_RX_QUEUE_CAPACITY", "CAN_TX_QUEUE_CAPACITY"):
        match = re.search(rf"#define\s+{name}\s+([0-9]+)u", source)
        assert match, name
        capacities[name] = int(match.group(1))
        assert capacities[name] >= 8
    for cursor in ("s_rx_head", "s_rx_tail", "s_tx_head", "s_tx_tail"):
        assert re.search(rf"static\s+volatile\s+uint8_t\s+{cursor}\s*;", source), cursor
    assert "s_rx_count" not in source
    assert "s_tx_count" not in source
    assert "malloc" not in source and "calloc" not in source
    rx_full = function_body(source, "can_rx_queue_full")
    tx_full = function_body(source, "can_tx_queue_full")
    assert "s_rx_head - s_rx_tail" in rx_full and "CAN_RX_QUEUE_CAPACITY" in rx_full
    assert "s_tx_head - s_tx_tail" in tx_full and "CAN_TX_QUEUE_CAPACITY" in tx_full


def test_public_queue_operations_are_nonblocking_and_validate_inputs():
    source = read(SOURCE)
    pop = function_body(source, "can_at32m412_rx_pop")
    push = function_body(source, "can_at32m412_tx_push")
    assert "out == NULL" in pop and "return false" in pop
    assert "s_rx_head == s_rx_tail" in pop
    assert "frame == NULL" in push and "return false" in push
    assert "frame->id > 0x7ffu" in push and "frame->dlc != 8u" in push
    assert "can_tx_queue_full()" in push
    for body, name in ((pop, "rx_pop"), (push, "tx_push")):
        assert "while" not in body, f"{name} must not wait"
        assert "for (" not in body, f"{name} must not loop"


def test_rx_irq_is_hardware_bounded_releases_every_read_and_latches_overflow():
    body = function_body(read(SOURCE), "can_at32m412_irq_rx")
    assert "CAN_HW_RX_DRAIN_LIMIT" in body
    assert re.search(r"for\s*\([^;]+;[^;]*CAN_HW_RX_DRAIN_LIMIT", body)
    read_index = body.index("can_rxbuf_read(CAN1, &rxbuf)")
    validate_index = body.index("can_rx_frame_valid(&rxbuf)")
    assert read_index < validate_index, "vendor read must release before software rejection"
    assert "rx_rejected" in body
    full_index = body.index("can_rx_queue_full()")
    assert "rx_overflow" in body[full_index:]
    assert "fatal_latched = true" in body[full_index:]
    assert "s_rx_queue[s_rx_head" in body[full_index:]
    fatal_index = body.index("fatal_latched = true", full_index)
    enqueue_index = body.index("s_rx_queue[s_rx_head", full_index)
    assert fatal_index < enqueue_index, "full queue must not overwrite unread data"


def test_rx_read_failure_explicitly_releases_before_continuing():
    irq = function_body(read(SOURCE), "can_at32m412_irq_rx")
    failure, _, _ = block_after(
        irq, "if (can_rxbuf_read(CAN1, &rxbuf) != SUCCESS)"
    )
    assert "can_rxbuf_release(CAN1)" in failure, \
        "read failure must explicitly release/discard the hardware buffer"
    release = failure.index("can_rxbuf_release(CAN1)")
    reject = failure.index("rx_rejected")
    retry = failure.index("continue;")
    assert release < retry
    assert reject < retry
    assert "s_rx_queue[" not in failure


def test_software_rx_validation_rejects_extended_remote_wrong_dlc_and_wrong_id():
    body = function_body(read(SOURCE), "can_rx_frame_valid")
    for token in (
        "rxbuf->id_type != CAN_ID_STANDARD",
        "rxbuf->frame_type != CAN_FRAME_DATA",
        "rxbuf->data_length != CAN_DLC_BYTES_8",
        "rxbuf->id == 0x080u",
        "rxbuf->id == (uint32_t)(0x100u + s_node_id)",
    ):
        assert token in body, token


def test_tx_kick_is_single_attempt_without_completion_wait():
    source = read(SOURCE)
    kick = function_body(source, "can_at32m412_tx_kick")
    start = function_body(source, "can_tx_start_next")
    assert "can_tx_start_next()" in kick
    assert start.count("can_txbuf_write(") == 1
    assert start.count("can_txbuf_transmit(") == 1
    assert start.index("s_tx_active = true") < start.index("can_txbuf_transmit("), \
        "completion IRQ must not race ahead of active-state publication"
    for body in (kick, start):
        assert "while" not in body
        assert "for (" not in body
    assert "can_transmit_status_get" not in kick
    assert "can_transmit_status_get" not in start


def test_tx_start_failure_rolls_back_active_without_dequeue():
    start = function_body(read(SOURCE), "can_tx_start_next")
    failure, _, _ = block_after(
        start, "if (can_txbuf_transmit(CAN1, CAN_TRANSMIT_PTB) != SUCCESS)"
    )
    assert "s_tx_active = false" in failure
    assert "tx_errors" in failure
    assert "return;" in failure
    assert "s_tx_tail" not in failure, "failed start must leave the frame queued for retry"
    assert start.index("s_tx_active = true") < start.index("can_txbuf_transmit(")


def test_tx_irq_only_consumes_matching_successful_completion_then_kicks_once():
    body = function_body(read(SOURCE), "can_at32m412_irq_tx")
    assert "CAN_TPIF_FLAG" in body and "can_flag_clear" in body
    assert "can_transmit_status_get(CAN1, &status)" in body
    accepted = body.index("status.final_tstat == CAN_TSTAT_TRANSMITTED")
    assert "status.final_handle == s_tx_active_handle" in body[:accepted + 1]
    advance = body.index("s_tx_tail++")
    assert accepted < advance
    assert body.count("s_tx_tail++") == 1
    assert body.count("can_tx_start_next()") == 1


def test_tx_completion_failure_clears_active_without_dequeue():
    irq = function_body(read(SOURCE), "can_at32m412_irq_tx")
    success, success_start, success_end = block_after(
        irq, "if (s_tx_active &&"
    )
    assert "s_tx_tail++" in success
    after_success = irq[success_end + 1:]
    assert "s_tx_active = false" in after_success, \
        "all completion outcomes must clear active state so a queued frame can retry"
    clear = after_success.index("s_tx_active = false")
    kick = after_success.index("can_tx_start_next()")
    assert clear < kick
    assert "s_tx_tail++" not in after_success, \
        "failed/mismatched completion must not dequeue the pending frame"


def test_status_error_capture_is_latched_and_saturating():
    source = read(SOURCE)
    snapshot = function_body(source, "can_snapshot_error_state")
    status = function_body(source, "can_at32m412_irq_status")
    error = function_body(source, "can_at32m412_irq_error")
    assert "can_receive_error_counter_get(CAN1)" in snapshot
    assert "can_transmit_error_counter_get(CAN1)" in snapshot
    assert "CAN_EPASS_FLAG" in snapshot
    assert "can_busoff_get(CAN1)" in snapshot
    assert "bus_off_latched = true" in snapshot
    assert "fatal_latched = true" in snapshot
    assert "can_busoff_reset" not in source
    assert "can_sat_increment" in status and "status_irqs" in status
    assert "can_sat_increment" in error and "error_irqs" in error
    assert "can_snapshot_error_state();" in status
    assert "can_snapshot_error_state();" in error
    assert "can_flag_clear" in status and "can_flag_clear" in error


def test_persistent_error_states_are_polled_instead_of_interrupt_driven():
    source = read(SOURCE)
    init = function_body(source, "can_at32m412_init")
    fatal = function_body(source, "can_at32m412_fatal_bus_error")
    app_run = function_body(read(MOTOR_APP), "motor_app_run")

    interrupt_call = re.search(
        r"can_interrupt_enable\s*\(\s*CAN1\s*,(.*?)\,\s*TRUE\s*\)\s*;",
        init,
        re.DOTALL,
    )
    assert interrupt_call, "missing CAN interrupt enable call"
    enabled_sources = interrupt_call.group(1)
    for persistent_source in ("CAN_EIE_INT", "CAN_BEIE_INT", "CAN_EPIE_INT"):
        assert persistent_source not in enabled_sources, (
            f"{persistent_source} can retrigger continuously while EWARN/EPASS remains set"
        )
    assert "CAN_ROIE_INT" in enabled_sources
    assert "can_snapshot_error_state();" in fatal
    assert fatal.index("nvic_irq_disable(CAN1_ERR_IRQn);") < fatal.index(
        "can_snapshot_error_state();"
    )
    assert fatal.index("can_snapshot_error_state();") < fatal.index(
        "nvic_irq_enable(CAN1_ERR_IRQn, CAN_IRQ_PRIORITY, 0u);"
    )
    assert "can_at32m412_fatal_bus_error()" in app_run


def test_diagnostic_counter_increment_saturates_at_uint32_max():
    increment = function_body(read(SOURCE), "can_sat_increment")
    guarded, _, guard_end = block_after(increment, "if (*counter != 0xffffffffu)")
    assert "++(*counter)" in guarded
    assert "++(*counter)" not in increment[guard_end + 1:]
    assert "(*counter)++" not in increment


def test_diagnostic_getter_documents_and_implements_mixed_snapshot_contract():
    header = read(HEADER)
    source = read(SOURCE)
    getter = function_body(source, "can_at32m412_get_diag")
    for phrase in (
        "eventually consistent",
        "not a transactional snapshot",
        "32-bit fields are",
        "individually atomic on Cortex-M4",
        "never refreshes hardware or mutates diagnostics",
        "can_at32m412_fatal_bus_error()",
    ):
        assert phrase in header, f"missing diagnostic contract phrase: {phrase}"
    assert "nvic_irq_disable" not in getter and "__disable_irq" not in getter
    assert_diagnostic_getter_read_only(source)
    for field in (
        "rec", "tec", "error_passive", "bus_off_latched", "fatal_latched",
        "rx_received", "rx_rejected", "rx_overflow", "tx_queued",
        "tx_completed", "tx_rejected", "tx_errors", "status_irqs", "error_irqs",
        "bus_off_events",
    ):
        assert f"out->{field} = s_diag.{field};" in getter


def test_read_only_getter_checker_rejects_refresh_mutation():
    source = read(SOURCE)
    marker = "void can_at32m412_get_diag(can_at32m412_diag_t *out)\n{"
    assert marker in source
    mutated = source.replace(
        marker, marker + "\n    can_snapshot_error_state();", 1
    )
    try:
        assert_diagnostic_getter_read_only(mutated)
    except AssertionError:
        pass
    else:
        raise AssertionError("getter checker accepted a state-refresh mutation")


def test_irq_wrappers_are_only_the_four_adapter_calls():
    source = read(INTERRUPTS)
    expected = {
        "CAN1_RX_IRQHandler": "can_at32m412_irq_rx();",
        "CAN1_TX_IRQHandler": "can_at32m412_irq_tx();",
        "CAN1_STAT_IRQHandler": "can_at32m412_irq_status();",
        "CAN1_ERR_IRQHandler": "can_at32m412_irq_error();",
    }
    assert '#include "can_at32m412.h"' in source
    for wrapper, call in expected.items():
        body = function_body(source, wrapper)
        code = re.sub(r"/\*.*?\*/", "", body, flags=re.DOTALL).strip()
        assert code == call, f"{wrapper} contains more than {call}"


if __name__ == "__main__":
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_")]
    for test in tests:
        test()
    print(f"can_at32m412 static contract: PASS ({len(tests)} tests)")
