from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
TIMER_H = ROOT / "platform" / "at32m412" / "can_motion_timer_at32m412.h"
TIMER_C = ROOT / "platform" / "at32m412" / "can_motion_timer_at32m412.c"
APP_C = ROOT / "application" / "motor_app.c"
ISR_C = ROOT / "application" / "motor_control" / "motor_control_isr.c"
SERVICE_C = ROOT / "application" / "motor_control" / "can_motion_service.c"
SERVICE_H = ROOT / "application" / "motor_control" / "can_motion_service.h"
SHELL_C = ROOT / "application" / "motor_shell.c"
CAN_H = ROOT / "communication" / "can_at32m412.h"
CAN_C = ROOT / "communication" / "can_at32m412.c"
USART_DMA_H = ROOT / "platform" / "at32m412" / "board_usart1_dma.h"
CMAKE = ROOT / "CMakeLists.txt"
UVPROJ = ROOT / "project" / "MDK_V5" / "MPS_MotorDriver.uvprojx"


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


def mutate_function(source, signature, needle, replacement):
    start = source.index(signature)
    opening_brace = source.index("{", start)
    body = function_body(source, signature)
    offset = body.index(needle)
    mutation_start = opening_brace + 1 + offset
    return (
        source[:mutation_start]
        + replacement
        + source[mutation_start + len(needle):]
    )


def assert_mutation_rejected(check, source, signature, needle, replacement):
    mutated = mutate_function(source, signature, needle, replacement)
    try:
        check(mutated)
    except AssertionError:
        return
    raise AssertionError(
        f"contract checker accepted mutation in {signature}: {needle}"
    )


def assert_timer_contract(timer):
    timer_normalized = normalized(timer)
    init = normalized(function_body(
        timer, "void can_motion_timer_at32m412_init(void)"
    ))
    handler = normalized(function_body(
        timer, "void TMR6_DAC_GLOBAL_IRQHandler(void)"
    ))

    for token in [
        "#define CAN_MOTION_TIMER TMR6",
        "#define CAN_MOTION_TIMER_IRQn TMR6_DAC_GLOBAL_IRQn",
        "#define CAN_MOTION_TIMER_CLOCK CRM_TMR6_PERIPH_CLOCK",
        "#define CAN_MOTION_TIMER_PRIO 4u",
        "#define CAN_MOTION_TIMER_PRESCALER 179u",
        "#define CAN_MOTION_TIMER_PERIOD 999u",
    ]:
        assert token in timer_normalized
    assert (
        "tmr_base_init(CAN_MOTION_TIMER, CAN_MOTION_TIMER_PERIOD, "
        "CAN_MOTION_TIMER_PRESCALER);"
    ) in init
    clear = "tmr_flag_clear(CAN_MOTION_TIMER, TMR_OVF_FLAG);"
    interrupt = (
        "tmr_interrupt_enable(CAN_MOTION_TIMER, TMR_OVF_INT, TRUE);"
    )
    counter = "tmr_counter_enable(CAN_MOTION_TIMER, TRUE);"
    assert clear in init
    assert interrupt in init
    assert counter in init
    assert init.index(clear) < init.index(interrupt) < init.index(counter)
    assert (
        "nvic_irq_enable(CAN_MOTION_TIMER_IRQn, CAN_MOTION_TIMER_PRIO, 0);"
    ) in init

    expected_handler = normalized("""
        if (tmr_flag_get(CAN_MOTION_TIMER, TMR_OVF_FLAG) != RESET) {
            tmr_flag_clear(CAN_MOTION_TIMER, TMR_OVF_FLAG);
            can_motion_service_tick_1ms();
        }
    """)
    assert handler == expected_handler


def assert_app_contract(app):
    init = normalized(function_body(app, "void motor_app_init(void)"))
    run = normalized(function_body(app, "void motor_app_run(void)"))
    callbacks = [
        "motor_app_can_rx_pop",
        "motor_app_can_tx_push",
        "motor_app_can_position_start",
        "motor_app_can_position_submit",
        "motor_app_can_position_stop",
        "motor_app_can_position_mdeg",
        "motor_app_can_velocity_mdeg_s",
        "motor_app_can_vbus_10mv",
        "motor_app_can_fault_get",
        "motor_app_can_fault_set",
        "motor_app_can_fault_clear",
    ]

    ops_start = app.index("static const can_motion_ops_t s_can_motion_ops")
    ops_end = app.index("};", ops_start)
    ops = app[ops_start:ops_end]
    for callback in callbacks:
        assert ops.count(callback) == 1, callback
    assert ops_start < app.index("void motor_app_init(void)")

    init_order = [
        "motor_pwm_at32m412_safe_init();",
        "current_sense_at32m412_init();",
        "motor_encoder_at32m412_init();",
        "fault_manager_init();",
        "motor_control_init(&s_motor_control);",
        "motor_calibration_load();",
        "current_loop_init();",
        "motor_control_isr_sampling_init();",
        "speed_loop_init();",
        "position_loop_init();",
        "joint_config_service_init();",
        "can_motion_service_init(&s_can_motion_ops);",
        "can_motion_timer_at32m412_init();",
        "encoder_acq_timer_at32m412_init();",
    ]
    for call in init_order:
        assert call in init
    indices = [init.index(call) for call in init_order]
    assert indices == sorted(indices)
    assert "can_at32m412_init" not in init
    assert "can_motion_service_set_joint_config(true" not in init

    one_shot = normalized("""
        if (!s_can_init_attempted) {
            uint8_t node_id;
            if (joint_config_service_lock_runtime(&node_id)) {
                s_can_init_attempted = true;
                if (can_at32m412_init(node_id)) {
                    s_can_ready = true;
                    can_motion_service_set_joint_config(true, node_id);
                } else {
                    can_motion_service_force_stop();
                }
            }
        }
    """)
    assert one_shot in run
    assert run.count("can_at32m412_init(node_id)") == 1
    assert run.index("s_can_init_attempted = true;") < run.index(
        "can_at32m412_init(node_id)"
    )
    assert run.index("joint_config_service_lock_runtime(&node_id)") < run.index(
        "s_can_init_attempted = true;"
    )
    assert "joint_config_service_ready()" not in run
    assert "joint_config_service_node_id()" not in run
    assert run.index("joint_config_service_poll();") < run.index(one_shot)

    ready_block = normalized("""
        if (s_can_ready) {
            can_at32m412_get_diag(&can_diag);
            if (can_diag.fatal_latched) {
                can_motion_service_force_stop();
            }
            can_motion_service_poll_tx();
            can_at32m412_tx_kick();
        }
    """)
    assert ready_block in run
    assert run.index(ready_block) < run.index("rt_thread_mdelay(10);")

    assert "return can_at32m412_rx_pop(out);" in function_body(
        app, "static bool motor_app_can_rx_pop(can_frame_t *out)"
    )
    assert "return can_at32m412_tx_push(frame);" in function_body(
        app, "static bool motor_app_can_tx_push(const can_frame_t *frame)"
    )
    assert "return motor_control_isr_position_start(setpoint);" in function_body(
        app,
        "static int motor_app_can_position_start(const position_setpoint_t *setpoint)",
    )
    assert "return motor_control_isr_position_submit(setpoint);" in function_body(
        app,
        "static int motor_app_can_position_submit(const position_setpoint_t *setpoint)",
    )
    assert "motor_control_isr_position_stop();" in function_body(
        app, "static void motor_app_can_position_stop(void)"
    )

    position = function_body(app, "static int32_t motor_app_can_position_mdeg(void)")
    assert "encoder_service_get_control_position_mdeg()" in position
    assert "position_loop_sensor_to_joint_mdeg" in position
    velocity = function_body(
        app, "static int32_t motor_app_can_velocity_mdeg_s(void)"
    )
    assert "encoder_tracker_get_speed_rad_s()" in velocity
    assert "MOTOR_POLE_PAIRS" in velocity
    assert "MOTOR_APP_RAD_S_TO_MDEG_S" in velocity
    assert "INT32_MAX" in velocity and "INT32_MIN" in velocity
    assert "position_loop_control_to_joint_velocity_mdeg_s" in velocity
    vbus = function_body(app, "static uint16_t motor_app_can_vbus_10mv(void)")
    assert "current_sense_at32m412_read_vbus()" in vbus
    assert "UINT16_MAX" in vbus and "100.0f" in vbus
    assert "fault_manager_get();" in function_body(
        app, "static uint32_t motor_app_can_fault_get(void)"
    )
    set_fault = function_body(
        app, "static void motor_app_can_fault_set(uint32_t bits)"
    )
    assert "fault_manager_set_bits(bits);" in set_fault
    assert "__disable_irq" not in set_fault
    clear_fault = function_body(app, "static void motor_app_can_fault_clear(void)")
    assert "fault_manager_clear_bits(FAULT_CAN_TIMEOUT | FAULT_CAN_BUS);" in normalized(
        clear_fault
    )
    assert "__disable_irq" not in clear_fault
    assert "fault_manager_clear_all" not in clear_fault


def assert_first_target_contract(source):
    start = normalized(function_body(
        source,
        "int motor_control_isr_position_start(const position_setpoint_t *setpoint)",
    ))
    required = [
        "if (!position_loop_first_target_safe(encoder.control_position_mdeg, "
        "setpoint->position_mdeg)) { return -5; }",
    ]
    for token in required:
        assert token in start
    check_index = start.index(required[0])
    assert check_index < start.index("position_loop_reset();")
    assert check_index < start.index("mc->state = MOTOR_CONTROL_STATE_ENABLED;")
    assert check_index < start.index("motor_pwm_at32m412_enable_output();")


def assert_can_shell_contract(shell):
    status = normalized(function_body(shell, "static void can_status("))
    reset = normalized(function_body(shell, "static void can_diag_reset("))
    assert "can_motion_service_get_snapshot(&motion)" in status
    assert "can_at32m412_get_diag(&driver);" in status
    assert "fault_manager_get();" in status
    assert "0x43414E31u" in status
    checksum_start = status.index("check =")
    checksum = status[checksum_start:status.index("rt_kprintf", checksum_start)]
    xor_order = [
        "motion.node_id", "motion.state", "motion.session",
        "motion.pending_sequence", "motion.applied_sequence",
        "motion.pending_age_ms", "motion.sync_age_ms", "motion.rx_frames",
        "motion.tx_frames", "motion.protocol_errors", "driver.rx_overflow",
        "driver.bus_off_events", "driver.tx_errors", "fault;",
    ]
    positions = [checksum.index(token) for token in xor_order]
    assert positions == sorted(positions)
    assert (
        '"cs id=%u s=%u se=%u p=%u a=%u pa=%u sa=%u " '
        '"rx=%lu tx=%lu pe=%lu ro=%lu bo=%lu te=%lu " '
        '"f=%08X k=%08X\\n"'
    ) in status
    for mutator in ("can_motion_service_reset_diagnostics",
                    "can_at32m412_reset_diagnostics", "set_joint_config",
                    "force_stop"):
        assert mutator not in status

    assert "motor_shell_reject_if_running()" in reset
    assert "can_motion_service_get_snapshot(&motion)" in reset
    assert "motion.state != CAN_NODE_STATE_READY" in reset
    assert "gpio_input_data_bit_read(PWM_EN_GPIO_PORT, PWM_EN_PIN)" in reset
    assert "can_motion_service_reset_diagnostics();" in reset
    assert "can_at32m412_reset_diagnostics();" in reset
    assert reset.index("motor_shell_reject_if_running") < reset.index(
        "can_motion_service_reset_diagnostics"
    )
    assert reset.index("motion.state != CAN_NODE_STATE_READY") < reset.index(
        "can_motion_service_reset_diagnostics"
    )
    assert reset.index("gpio_input_data_bit_read") < reset.index(
        "can_motion_service_reset_diagnostics"
    )


def assert_reset_api_contract(service_h, service_c, can_h, can_c):
    assert "void can_motion_service_reset_diagnostics(void);" in service_h
    motion = normalized(function_body(
        service_c, "void can_motion_service_reset_diagnostics(void)"
    ))
    for field in ("rx_frames", "tx_frames", "tx_failures", "protocol_errors"):
        assert f"s_service.{field} = 0u;" in motion
    for forbidden in ("state =", "session =", "node_id =", "fault_bits =",
                      "clear_pending", "clear_sequence_window", "memset"):
        assert forbidden not in motion
    assert "can_motion_service_state_lock()" in motion
    assert "can_motion_service_state_unlock(primask)" in motion

    assert "uint32_t bus_off_events;" in can_h
    assert "void can_at32m412_reset_diagnostics(void);" in can_h
    driver = normalized(function_body(
        can_c, "void can_at32m412_reset_diagnostics(void)"
    ))
    for field in ("rx_received", "rx_rejected", "rx_overflow", "tx_queued",
                  "tx_completed", "tx_rejected", "tx_errors", "status_irqs",
                  "error_irqs", "bus_off_events"):
        assert f"s_diag.{field} = 0u;" in driver
    for preserved in ("rec", "tec", "error_passive", "bus_off_latched",
                      "fatal_latched"):
        assert f"s_diag.{preserved} =" not in driver
    assert "__get_PRIMASK()" in driver
    assert "__disable_irq();" in driver


def test_tmr6_is_exactly_1khz_and_isr_is_pure():
    header = read(TIMER_H)
    timer = read(TIMER_C)
    assert "#define CAN_MOTION_TIMER_HZ 1000u" in header
    assert "void can_motion_timer_at32m412_init(void);" in header
    assert_timer_contract(timer)


def test_safe_init_run_callbacks_and_arm_contracts():
    app = read(APP_C)
    service = read(SERVICE_C)
    assert_app_contract(app)
    arm = function_body(service, "static void process_arm(")
    assert "position_start" not in arm
    assert "motor_pwm" not in arm


def test_first_target_distance_is_checked_before_any_start_mutation():
    assert_first_target_contract(read(ISR_C))


def test_checked_can_shell_and_reset_ownership_contracts():
    assert_can_shell_contract(read(SHELL_C))
    assert_reset_api_contract(read(SERVICE_H), read(SERVICE_C),
                              read(CAN_H), read(CAN_C))
    uart = read(USART_DMA_H)
    assert "#define USART1_TX_STAGE_BUF_SIZE 256u" in uart
    worst = (
        "cs id=2 s=5 se=65535 p=65535 a=65535 pa=65535 sa=65535 "
        "rx=4294967295 tx=4294967295 pe=4294967295 ro=4294967295 "
        "bo=4294967295 te=4294967295 f=FFFFFFFF k=FFFFFFFF\r\n"
    )
    assert len(worst.encode("ascii")) < 256


def test_contract_checkers_reject_scoped_mutations():
    timer = read(TIMER_C)
    app = read(APP_C)
    isr = read(ISR_C)
    for signature, needle, replacement in [
        (
            "void can_motion_timer_at32m412_init(void)",
            "tmr_flag_clear(CAN_MOTION_TIMER, TMR_OVF_FLAG);",
            "",
        ),
        (
            "void TMR6_DAC_GLOBAL_IRQHandler(void)",
            "can_motion_service_tick_1ms();",
            "can_motion_service_poll_tx();",
        ),
    ]:
        assert_mutation_rejected(
            assert_timer_contract, timer, signature, needle, replacement
        )
    for signature, needle, replacement in [
        (
            "void motor_app_init(void)",
            "can_motion_service_init(&s_can_motion_ops);",
            "",
        ),
        (
            "void motor_app_run(void)",
            "s_can_init_attempted = true;",
            "s_can_init_attempted = false;",
        ),
        (
            "void motor_app_run(void)",
            "joint_config_service_lock_runtime(&node_id)",
            "joint_config_service_ready()",
        ),
        (
            "void motor_app_run(void)",
            "can_motion_service_poll_tx();",
            "",
        ),
    ]:
        assert_mutation_rejected(
            assert_app_contract, app, signature, needle, replacement
        )
    assert_mutation_rejected(
        assert_first_target_contract,
        isr,
        "int motor_control_isr_position_start(const position_setpoint_t *setpoint)",
        "position_loop_first_target_safe",
        "position_loop_origin_valid",
    )
    shell = read(SHELL_C)
    assert_mutation_rejected(
        assert_can_shell_contract, shell, "static void can_status(",
        "0x43414E31u", "0x43414E30u"
    )
    assert_mutation_rejected(
        assert_can_shell_contract, shell, "static void can_diag_reset(",
        "motion.state != CAN_NODE_STATE_READY",
        "motion.state != CAN_NODE_STATE_ARMED"
    )


def test_all_task_sources_are_linked_once_in_both_builds():
    cmake = read(CMAKE)
    uvproj = read(UVPROJ)
    sources = [
        ("application/motor_control/joint_config.c", "application\\motor_control\\joint_config.c"),
        ("application/motor_control/joint_config_service.c", "application\\motor_control\\joint_config_service.c"),
        ("application/motor_control/can_motion_service.c", "application\\motor_control\\can_motion_service.c"),
        ("platform/at32m412/flash_joint_config_at32m412.c", "platform\\at32m412\\flash_joint_config_at32m412.c"),
        ("platform/at32m412/can_motion_timer_at32m412.c", "platform\\at32m412\\can_motion_timer_at32m412.c"),
        ("communication/can_protocol.c", "communication\\can_protocol.c"),
        ("communication/can_at32m412.c", "communication\\can_at32m412.c"),
    ]
    for cmake_path, uv_path in sources:
        name = Path(cmake_path).name
        assert cmake.count(f"${{CMAKE_SOURCE_DIR}}/{cmake_path}") == 1, cmake_path
        assert uvproj.count(f"<FileName>{name}</FileName>") == 1, name
        assert uvproj.count(f"..\\..\\{uv_path}") == 1, uv_path
    assert "<TextAddressRange>0x08000000</TextAddressRange>" in uvproj


if __name__ == "__main__":
    test_tmr6_is_exactly_1khz_and_isr_is_pure()
    test_safe_init_run_callbacks_and_arm_contracts()
    test_first_target_distance_is_checked_before_any_start_mutation()
    test_checked_can_shell_and_reset_ownership_contracts()
    test_contract_checkers_reject_scoped_mutations()
    test_all_task_sources_are_linked_once_in_both_builds()
    print("CAN motion integration static tests passed")
