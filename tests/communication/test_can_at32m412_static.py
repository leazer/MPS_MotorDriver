from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "communication" / "can_at32m412.h"
SOURCE = ROOT / "communication" / "can_at32m412.c"
INTERRUPTS = ROOT / "project" / "src" / "at32m412_416_int.c"


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


def test_public_api_and_diagnostics_contract():
    header = read(HEADER)
    declarations = (
        r"bool\s+can_at32m412_init\s*\(\s*uint8_t\s+node_id\s*\)\s*;",
        r"bool\s+can_at32m412_rx_pop\s*\(\s*can_frame_t\s*\*\s*out\s*\)\s*;",
        r"bool\s+can_at32m412_tx_push\s*\(\s*const\s+can_frame_t\s*\*\s*frame\s*\)\s*;",
        r"void\s+can_at32m412_tx_kick\s*\(\s*void\s*\)\s*;",
        r"void\s+can_at32m412_get_diag\s*\(\s*can_at32m412_diag_t\s*\*\s*out\s*\)\s*;",
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
    ):
        assert re.search(rf"\b{field}\s*;", header), f"missing diagnostic field {field}"


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


def test_interrupts_are_enabled_at_priority_three():
    body = function_body(read(SOURCE), "can_at32m412_init")
    for interrupt in ("CAN_RIE_INT", "CAN_TPIE_INT", "CAN_EPIE_INT", "CAN_EIE_INT", "CAN_BEIE_INT", "CAN_ROIE_INT"):
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
    assert "can_flag_clear" in status and "can_flag_clear" in error


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
