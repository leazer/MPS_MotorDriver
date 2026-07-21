from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
PARAMS = ROOT / "application" / "motor_control" / "motor_params.h"
LINKER = ROOT / "AT32M412xB_FLASH.ld"
MDK = ROOT / "project" / "MDK_V5" / "MPS_MotorDriver.uvprojx"
HEADER = ROOT / "platform" / "at32m412" / "flash_joint_config_at32m412.h"
SOURCE = ROOT / "platform" / "at32m412" / "flash_joint_config_at32m412.c"
SERVICE_HEADER = (
    ROOT / "application" / "motor_control" / "joint_config_service.h"
)
SERVICE_SOURCE = (
    ROOT / "application" / "motor_control" / "joint_config_service.c"
)
APP = ROOT / "application" / "motor_app.c"
SHELL = ROOT / "application" / "motor_shell.c"


def read(path):
    assert path.exists(), f"missing {path}"
    return path.read_text(encoding="utf-8")


def normalized_source(source):
    return re.sub(r"\s+", " ", source).strip()


def function_block(source, function_name, next_function_name):
    return source.split(function_name, 1)[1].split(next_function_name, 1)[0]


def function_body(source, signature):
    function_start = source.index(signature)
    opening_brace = source.index("{", function_start)
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
    function_start = source.index(signature)
    opening_brace = source.index("{", function_start)
    body = function_body(source, signature)
    mutation_offset = body.index(needle)
    mutation_start = opening_brace + 1 + mutation_offset
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


def assert_joint_shell_behavior_contract(shell):
    set_body = normalized_source(function_body(
        shell, "static void joint_cfg_set(int argc, char **argv)"
    ))
    show_body = normalized_source(function_body(
        shell, "static void joint_cfg_show(int argc, char **argv)"
    ))
    erase_body = normalized_source(function_body(
        shell, "static void joint_cfg_erase(int argc, char **argv)"
    ))
    parser_guard = (
        "if (!motor_shell_parse_u32(argv[1], &node_id) || "
        "!motor_shell_parse_i32(argv[2], &known_mdeg) || "
        "!motor_shell_parse_i32(argv[3], &direction) || "
        "!motor_shell_parse_i32(argv[4], &min_mdeg) || "
        "!motor_shell_parse_i32(argv[5], &max_mdeg)) {"
    )
    validation_guard = (
        "if (node_id < 1u || node_id > 2u || "
        "(direction != -1 && direction != 1) || "
        "min_mdeg > known_mdeg || known_mdeg > max_mdeg || "
        "width_mdeg >= 360000LL) {"
    )

    assert "if (argc != 6) {" in set_body
    assert "if (argc != 1) {" in show_body
    assert "if (argc != 1) {" in erase_body
    assert parser_guard in set_body
    assert "width_mdeg = (int64_t)max_mdeg - (int64_t)min_mdeg" in set_body
    assert validation_guard in set_body


def assert_service_disabled_contract(service):
    disabled_body = normalized_source(function_body(
        service, "static bool joint_config_service_motor_disabled(void)"
    ))
    capture_body = normalized_source(function_body(
        service,
        "bool joint_config_service_capture(uint8_t node_id,",
    ))
    erase_body = normalized_source(function_body(
        service, "bool joint_config_service_erase(void)"
    ))
    complete_gate = (
        "return control != NULL && "
        "motor_control_get_state(control) == MOTOR_CONTROL_STATE_DISABLED && "
        "!motor_control_isr_open_loop_active() && "
        "!motor_control_isr_align_active() && "
        "!motor_control_isr_current_active() && "
        "!motor_control_isr_speed_active() && "
        "!motor_control_isr_position_active();"
    )

    assert complete_gate in disabled_body
    assert "if (!joint_config_service_motor_disabled()) { return false; }" in capture_body
    assert "if (!joint_config_service_motor_disabled()) { return false; }" in erase_body


def test_contract_checkers_reject_incomplete_mutations():
    shell = read(SHELL)
    service = read(SERVICE_SOURCE)

    assert_joint_shell_behavior_contract(shell)
    assert_service_disabled_contract(service)
    shell_mutations = [
        (
            "static void joint_cfg_set(int argc, char **argv)",
            "if (argc != 6)",
            "if (argc != 5)",
        ),
        (
            "static void joint_cfg_show(int argc, char **argv)",
            "if (argc != 1)",
            "if (argc != 2)",
        ),
        (
            "static void joint_cfg_erase(int argc, char **argv)",
            "if (argc != 1)",
            "if (argc != 2)",
        ),
        (
            "static void joint_cfg_set(int argc, char **argv)",
            "!motor_shell_parse_u32(argv[1], &node_id)",
            "false",
        ),
        (
            "static void joint_cfg_set(int argc, char **argv)",
            "!motor_shell_parse_i32(argv[2], &known_mdeg)",
            "false",
        ),
        (
            "static void joint_cfg_set(int argc, char **argv)",
            "!motor_shell_parse_i32(argv[3], &direction)",
            "false",
        ),
        (
            "static void joint_cfg_set(int argc, char **argv)",
            "!motor_shell_parse_i32(argv[4], &min_mdeg)",
            "false",
        ),
        (
            "static void joint_cfg_set(int argc, char **argv)",
            "!motor_shell_parse_i32(argv[5], &max_mdeg)",
            "false",
        ),
        (
            "static void joint_cfg_set(int argc, char **argv)",
            "node_id < 1u || node_id > 2u",
            "node_id < 0u || node_id > 3u",
        ),
        (
            "static void joint_cfg_set(int argc, char **argv)",
            "direction != -1 && direction != 1",
            "direction != 0",
        ),
        (
            "static void joint_cfg_set(int argc, char **argv)",
            "min_mdeg > known_mdeg || known_mdeg > max_mdeg",
            "min_mdeg > max_mdeg",
        ),
        (
            "static void joint_cfg_set(int argc, char **argv)",
            "width_mdeg >= 360000LL",
            "width_mdeg > 360000LL",
        ),
    ]
    service_mutations = [
        (
            "static bool joint_config_service_motor_disabled(void)",
            "motor_control_get_state(control) == MOTOR_CONTROL_STATE_DISABLED",
            "true",
        ),
        (
            "static bool joint_config_service_motor_disabled(void)",
            "!motor_control_isr_open_loop_active()",
            "true",
        ),
        (
            "static bool joint_config_service_motor_disabled(void)",
            "!motor_control_isr_align_active()",
            "true",
        ),
        (
            "static bool joint_config_service_motor_disabled(void)",
            "!motor_control_isr_current_active()",
            "true",
        ),
        (
            "static bool joint_config_service_motor_disabled(void)",
            "!motor_control_isr_speed_active()",
            "true",
        ),
        (
            "static bool joint_config_service_motor_disabled(void)",
            "!motor_control_isr_position_active()",
            "true",
        ),
        (
            "bool joint_config_service_capture(uint8_t node_id,",
            "if (!joint_config_service_motor_disabled())",
            "if (false)",
        ),
        (
            "bool joint_config_service_erase(void)",
            "if (!joint_config_service_motor_disabled())",
            "if (false)",
        ),
    ]

    for signature, needle, replacement in shell_mutations:
        assert_function_mutation_rejected(
            assert_joint_shell_behavior_contract,
            shell,
            signature,
            needle,
            replacement,
        )
    for signature, needle, replacement in service_mutations:
        assert_function_mutation_rejected(
            assert_service_disabled_contract,
            service,
            signature,
            needle,
            replacement,
        )


def test_flash_layout_reserves_two_joint_pages_before_calibration():
    params = read(PARAMS)
    linker = read(LINKER)
    uvproj = read(MDK)

    assert "#define JOINT_CFG_FLASH_A_ADDR" in params
    assert "0x0801F400u" in params
    assert "#define JOINT_CFG_FLASH_B_ADDR" in params
    assert "0x0801F800u" in params
    assert "#define JOINT_CFG_FLASH_PAGE_SIZE" in params
    assert "1024u" in params
    assert "#define CAL_FLASH_ADDR" in params and "0x0801FC00u" in params
    assert "LENGTH = 125K" in linker
    assert "IROM(0x08000000,0x1F400)" in uvproj
    assert "<Size>0x1F400</Size>" in uvproj
    assert "-FL00020000" in uvproj
    assert "<FileName>joint_config.c</FileName>" in uvproj
    assert "..\\..\\application\\motor_control\\joint_config.c" in uvproj
    assert "<FileName>flash_joint_config_at32m412.c</FileName>" in uvproj
    assert "..\\..\\platform\\at32m412\\flash_joint_config_at32m412.c" in uvproj


def test_joint_storage_contract_uses_inactive_page_and_verified_word_write():
    header = read(HEADER)
    flash_source = read(SOURCE)

    for token in [
        "flash_joint_config_read_latest",
        "flash_joint_config_write_next",
        "flash_joint_config_erase_all",
    ]:
        assert token in header

    assert "flash_sector_erase(target_addr)" in flash_source
    assert "flash_word_program" in flash_source
    assert "joint_config_record_valid" in flash_source
    assert "joint_config_generation_newer" in flash_source
    assert "joint_config_record_t" in flash_source
    assert "sizeof(joint_config_record_t) / sizeof(uint32_t)" in flash_source
    assert "typedef char joint_config_record_size_must_be_36" in flash_source
    assert "typedef char joint_config_word_count_must_be_nine" in flash_source
    assert "_Static_assert" not in flash_source
    assert "flash_lock();" in flash_source
    assert "memcmp" in flash_source

    latest_selection = flash_source.split(
        "static const joint_config_record_t *flash_joint_config_latest", 1
    )[1].split("bool flash_joint_config_read_latest", 1)[0]
    read_latest = flash_source.split("bool flash_joint_config_read_latest", 1)[1]
    write_next = flash_source.split("bool flash_joint_config_write_next", 1)[1]
    erase_all = flash_source.split("bool flash_joint_config_erase_all", 1)[1]

    assert "joint_config_generation_newer" in latest_selection
    assert "JOINT_CFG_FLASH_A_ADDR" in latest_selection
    assert "JOINT_CFG_FLASH_B_ADDR" in latest_selection
    assert "flash_joint_config_latest(&latest_addr)" in read_latest
    assert "latest_addr" in write_next
    assert "target_addr" in write_next
    assert "joint_config_record_valid(record)" in write_next
    assert "flash_sector_erase(target_addr)" in write_next
    assert "flash_lock();" in write_next
    assert "memcmp" in write_next
    assert "joint_config_record_valid(readback)" in write_next
    assert "flash_sector_erase(JOINT_CFG_FLASH_A_ADDR)" in erase_all
    assert "flash_sector_erase(JOINT_CFG_FLASH_B_ADDR)" in erase_all
    assert "flash_lock();" in erase_all


def test_write_next_structurally_preserves_active_page_and_locks_failures():
    flash_source = read(SOURCE)
    write_next = normalized_source(function_block(
        flash_source,
        "bool flash_joint_config_write_next",
        "bool flash_joint_config_erase_all",
    ))

    assert (
        "if (flash_joint_config_latest(&latest_addr) == NULL) { "
        "target_addr = JOINT_CFG_FLASH_A_ADDR; } else if "
        "(latest_addr == JOINT_CFG_FLASH_A_ADDR) { "
        "target_addr = JOINT_CFG_FLASH_B_ADDR; } else { "
        "target_addr = JOINT_CFG_FLASH_A_ADDR; }"
    ) in write_next
    assert (
        "for (word_index = 0u; word_index < JOINT_CFG_WORD_COUNT; "
        "++word_index)"
    ) in write_next

    after_unlock = write_next.split("flash_unlock();", 1)[1]
    erase_failure = re.search(
        r"status = flash_sector_erase\(target_addr\); "
        r"if \(status != FLASH_OPERATE_DONE\) \{ (?P<body>.*?) \}",
        after_unlock,
    )
    word_failure = re.search(
        r"for \(word_index = 0u; word_index < JOINT_CFG_WORD_COUNT; "
        r"\+\+word_index\) \{ .*?"
        r"if \(status != FLASH_OPERATE_DONE\) \{ (?P<body>.*?) \}",
        after_unlock,
    )

    assert erase_failure is not None
    assert "flash_lock(); return false;" in erase_failure.group("body")
    assert word_failure is not None
    assert "flash_lock(); return false;" in word_failure.group("body")

    failure_returns = re.findall(r".*?return false;", after_unlock)
    assert failure_returns
    assert all("flash_lock();" in failure for failure in failure_returns)


def test_joint_config_service_public_contract_and_app_integration():
    header = read(SERVICE_HEADER)
    app = read(APP)

    for token in [
        "joint_config_service_init",
        "joint_config_service_poll",
        "joint_config_service_ready",
        "joint_config_service_node_id",
        "joint_config_service_capture",
        "joint_config_service_erase",
        "joint_config_service_get_status",
    ]:
        assert token in header

    app_init = function_block(app, "void motor_app_init", "void motor_app_run")
    app_run = app.split("void motor_app_run", 1)[1]
    assert app_init.index("position_loop_init();") < app_init.index(
        "joint_config_service_init();"
    )
    assert app_run.index("joint_config_service_poll();") < app_run.index(
        "rt_thread_mdelay(10);"
    )


def test_service_restores_once_from_valid_encoder_and_captures_corrected_raw():
    service = read(SERVICE_SOURCE)
    assert_service_disabled_contract(service)
    init_body = function_block(
        service, "void joint_config_service_init", "void joint_config_service_poll"
    )
    poll_body = function_block(
        service, "void joint_config_service_poll", "bool joint_config_service_ready"
    )
    capture_body = function_block(
        service,
        "bool joint_config_service_capture",
        "bool joint_config_service_erase",
    )

    assert "flash_joint_config_read_latest" in init_body
    assert "s_ready = false" in init_body
    assert "if (s_ready" in poll_body
    assert "encoder_service_get_snapshot" in poll_body
    assert "snapshot.valid" in poll_body
    assert "joint_config_restore_angle" in poll_body
    assert "snapshot.corrected_raw16" in poll_body
    assert (
        "position_loop_set_origin(snapshot.control_position_mdeg, "
        "restored_joint_mdeg)"
    ) in normalized_source(poll_body)

    assert "encoder_service_get_snapshot" in capture_body
    assert "snapshot.valid" in capture_body
    assert "snapshot.corrected_raw16" in capture_body
    assert "joint_config_make" in capture_body
    assert "flash_joint_config_write_next" in capture_body
    assert capture_body.count("flash_joint_config_read_latest") >= 2
    assert "joint_config_record_valid" in capture_body
    assert "memcmp" in capture_body
    assert "joint_config_restore_angle" in capture_body
    assert (
        "position_loop_set_origin(snapshot.control_position_mdeg, "
        "restored_joint_mdeg)"
    ) in normalized_source(capture_body)
    assert capture_body.index("flash_joint_config_write_next") < capture_body.rindex(
        "flash_joint_config_read_latest"
    ) < capture_body.index("position_loop_set_origin")


def test_service_erase_is_disabled_only_and_clears_runtime_origin_after_flash():
    service = read(SERVICE_SOURCE)
    assert_service_disabled_contract(service)
    erase_body = function_block(
        service,
        "bool joint_config_service_erase",
        "bool joint_config_service_get_status",
    )

    assert erase_body.index("flash_joint_config_erase_all") < erase_body.index(
        "position_loop_init"
    )
    assert "s_ready = false" in erase_body
    assert "s_record_present = false" in erase_body


def test_joint_config_shell_commands_have_strict_parsing_and_full_status():
    shell = read(SHELL)
    assert_joint_shell_behavior_contract(shell)
    signed_parser = function_block(
        shell, "static bool motor_shell_parse_i32", "static bool motor_shell_parse_u32"
    )
    unsigned_parser = function_block(
        shell, "static bool motor_shell_parse_u32", "static void pwm_info"
    )

    for command in ["joint_cfg_set", "joint_cfg_show", "joint_cfg_erase"]:
        assert f"static void {command}" in shell
        assert f"MSH_CMD_EXPORT({command}" in shell
    assert (
        "joint_cfg_set <node_id> <known_mdeg> <direction> <min_mdeg> <max_mdeg>"
        in shell
    )
    for token in [
        "errno",
        "ERANGE",
        "end",
        "INT32_MIN",
        "INT32_MAX",
        "joint_config_service_capture",
        "joint_config_service_erase",
        "joint_config_service_get_status",
    ]:
        assert token in shell
    assert "motor_shell_decimal_string(text, true)" in signed_parser
    assert "motor_shell_decimal_string(text, false)" in unsigned_parser
    for label in [
        "version",
        "generation",
        "node",
        "zero_raw",
        "known_mdeg",
        "min_mdeg",
        "max_mdeg",
        "direction",
        "crc_valid",
        "encoder_ready",
        "restored_joint_mdeg",
        "service_ready",
    ]:
        assert label in shell


if __name__ == "__main__":
    test_contract_checkers_reject_incomplete_mutations()
    test_flash_layout_reserves_two_joint_pages_before_calibration()
    test_joint_storage_contract_uses_inactive_page_and_verified_word_write()
    test_write_next_structurally_preserves_active_page_and_locks_failures()
    test_joint_config_service_public_contract_and_app_integration()
    test_service_restores_once_from_valid_encoder_and_captures_corrected_raw()
    test_service_erase_is_disabled_only_and_clears_runtime_origin_after_flash()
    test_joint_config_shell_commands_have_strict_parsing_and_full_status()
    print("joint config static tests passed")
