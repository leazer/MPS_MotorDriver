from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
SCOPE = ROOT / "debug.lksscope"
ISR_SOURCE = ROOT / "application" / "motor_control" / "motor_control_isr.c"
ENCODER_SOURCE = ROOT / "application" / "motor_control" / "encoder_service.c"
POSITION_SOURCE = ROOT / "application" / "motor_control" / "position_loop.c"

REQUIRED_CURVES = {
    "s_snapshot.raw16",
    "s_snapshot.raw_elec_mrad",
    "s_snapshot.elec_mrad",
    "s_dbg_id_ma",
    "s_dbg_iq_ma",
    "s_sampling_debug_snapshot.ia_ma",
    "s_sampling_debug_snapshot.ib_ma",
    "s_sampling_debug_snapshot.ic_ma",
    "s_dbg_uu_mv",
    "s_dbg_uv_mv",
    "s_dbg_uw_mv",
}

REQUIRED_TUNING = {
    "g_motor_tuning.current.id_kp",
    "g_motor_tuning.current.id_ki",
    "g_motor_tuning.current.iq_kp",
    "g_motor_tuning.current.iq_ki",
    "g_motor_tuning.current.id_integral_limit_v",
    "g_motor_tuning.current.iq_integral_limit_v",
    "g_motor_tuning.current.id_output_limit_v",
    "g_motor_tuning.current.iq_output_limit_v",
    "g_motor_tuning.speed.kp",
    "g_motor_tuning.speed.kp_brake",
    "g_motor_tuning.speed.ki",
    "g_motor_tuning.speed.integral_limit_A",
    "g_motor_tuning.speed.output_limit_A",
    "g_motor_tuning.speed.friction_A",
    "g_motor_tuning.speed.ramp_rad_s2",
    "g_motor_tuning.position.kp",
    "g_motor_tuning.position.speed_limit_elec_rad_s",
    "g_motor_tuning.position.max_velocity_mdeg_s",
    "g_motor_tuning.position.max_error_mdeg",
    "g_motor_tuning.position.command_limit_mdeg",
    "g_motor_tuning.position.extrapolation_limit_ms",
    "g_motor_tuning.position.iq_friction_A",
    "g_motor_tuning.position.iq_friction_moving_A",
    "g_motor_tuning.position.iq_friction_error_mdeg",
    "g_motor_tuning.protection.phase_overcurrent_A",
    "g_motor_tuning.protection.overcurrent_debounce_ticks",
    "g_motor_tuning.protection.imbalance_threshold_A",
    "g_motor_tuning.protection.imbalance_debounce_ticks",
    "g_motor_tuning.protection.sample_blanking_ticks",
    "g_motor_tuning.protection.sample_invalid_limit",
}

REQUIRED_LOOP_DEBUG = {
    "g_motor_loop_debug.current.id_ref_A",
    "g_motor_loop_debug.current.iq_ref_A",
    "g_motor_loop_debug.current.id_measured_A",
    "g_motor_loop_debug.current.iq_measured_A",
    "g_motor_loop_debug.current.id_error_A",
    "g_motor_loop_debug.current.iq_error_A",
    "g_motor_loop_debug.current.id_integral_v",
    "g_motor_loop_debug.current.iq_integral_v",
    "g_motor_loop_debug.current.vd_unlimited_v",
    "g_motor_loop_debug.current.vq_unlimited_v",
    "g_motor_loop_debug.current.vd_output_v",
    "g_motor_loop_debug.current.vq_output_v",
    "g_motor_loop_debug.current.id_saturated",
    "g_motor_loop_debug.current.iq_saturated",
    "g_motor_loop_debug.speed.target_rad_s",
    "g_motor_loop_debug.speed.command_rad_s",
    "g_motor_loop_debug.speed.measured_rad_s",
    "g_motor_loop_debug.speed.error_rad_s",
    "g_motor_loop_debug.speed.integral_A",
    "g_motor_loop_debug.speed.active_kp",
    "g_motor_loop_debug.speed.friction_A",
    "g_motor_loop_debug.speed.iq_unlimited_A",
    "g_motor_loop_debug.speed.iq_output_A",
    "g_motor_loop_debug.speed.saturated",
    "g_motor_loop_debug.position.target_position_mdeg",
    "g_motor_loop_debug.position.reference_position_mdeg",
    "g_motor_loop_debug.position.measured_position_mdeg",
    "g_motor_loop_debug.position.error_mdeg",
    "g_motor_loop_debug.position.velocity_ff_mdeg_s",
    "g_motor_loop_debug.position.proportional_velocity_mdeg_s",
    "g_motor_loop_debug.position.speed_unlimited_elec_rad_s",
    "g_motor_loop_debug.position.speed_output_elec_rad_s",
    "g_motor_loop_debug.position.iq_feedforward_A",
    "g_motor_loop_debug.protection.max_phase_current_A",
    "g_motor_loop_debug.protection.imbalance_A",
    "g_motor_loop_debug.protection.overcurrent_consecutive",
    "g_motor_loop_debug.protection.invalid_consecutive",
    "g_motor_loop_debug.protection.imbalance_consecutive",
    "g_motor_loop_debug.protection.invalid_total",
    "g_motor_loop_debug.protection.frame_valid",
    "g_motor_loop_debug.protection.overcurrent_active",
}

REQUIRED_NUMERIC = {
    "s_motor_control.state",
    "s_motor_control.mode",
    "s_motor_control.fault_flags",
    "s_motor_control.iq_ref_ma",
    "s_motor_control.speed_ref_rpm",
    "s_motor_control.position_ref_mdeg",
    "s_snapshot.raw16",
    "s_snapshot.corrected_raw16",
    "s_snapshot.raw_elec_mrad",
    "s_snapshot.elec_mrad",
    "s_snapshot.speed_mech_mrad_s",
    "s_snapshot.speed_elec_mrad_s",
    "s_snapshot.valid",
    "s_snapshot.bus_error_count",
    "s_snapshot.spike_count",
    "s_dbg_id_ma",
    "s_dbg_iq_ma",
    "s_dbg_id_avg_ma",
    "s_dbg_iq_avg_ma",
    "s_sampling_debug_snapshot.ia_ma",
    "s_sampling_debug_snapshot.ib_ma",
    "s_sampling_debug_snapshot.ic_ma",
    "s_dbg_vbus_mv",
    "s_dbg_uu_mv",
    "s_dbg_uv_mv",
    "s_dbg_uw_mv",
    "s_dbg_ta",
    "s_dbg_tb",
    "s_dbg_tc",
    "s_sampling_debug_snapshot.sample_valid_mask",
    "s_sampling_debug_snapshot.reconstructed_phase",
    "s_sampling_debug_snapshot.sample_invalid_total",
    "s_sampling_debug_snapshot.sample_invalid_consecutive",
    "s_sampling_debug_snapshot.sample_overcurrent_consecutive",
    "s_sampling_debug_snapshot.pi_freeze_count",
    "s_dbg_oc_hits",
    "s_dbg_imbal_hits",
    "s_dbg_tick_count",
    "s_dbg_cur_hits",
    "s_dbg_spd_hits",
    "s_dbg_fault_hits",
    "s_dbg_enc_alive",
} | REQUIRED_TUNING | REQUIRED_LOOP_DEBUG


def load_scope():
    assert SCOPE.exists(), f"missing {SCOPE}"
    return ET.parse(SCOPE).getroot()


def form(root, form_type):
    matches = root.findall(f"./forms/form[@type='{form_type}']")
    assert len(matches) == 1, f"expected one form type {form_type}, got {len(matches)}"
    return matches[0]


def test_connection_and_map_settings_are_preserved():
    root = load_scope()
    params = {item.attrib["name"]: item.attrib.get("value", "")
              for item in root.findall("./params/param")}
    assert params["commPort"] == "COM9"
    assert params["jClock"] == "10000000"
    assert params["sampleInterval"] == "10"
    assert form(root, "2").attrib["mapFile"] == "./build/Debug/MPS_MotorDriver.elf"


def test_requested_signals_are_curves():
    curve_form = form(load_scope(), "8")
    variables = {item.attrib["name"]: item.attrib["subPlot"]
                 for item in curve_form.findall("var")}
    names = list(variables)
    assert set(names) == REQUIRED_CURVES
    assert len(names) == len(set(names))
    assert curve_form.attrib["subMode"] == "1"
    assert variables["s_snapshot.raw16"] == "0"
    assert variables["s_snapshot.raw_elec_mrad"] == "1"
    assert variables["s_snapshot.elec_mrad"] == "1"
    assert variables["s_dbg_id_ma"] == "2"
    assert variables["s_dbg_iq_ma"] == "2"
    assert variables["s_sampling_debug_snapshot.ia_ma"] == "3"
    assert variables["s_sampling_debug_snapshot.ib_ma"] == "3"
    assert variables["s_sampling_debug_snapshot.ic_ma"] == "3"
    assert variables["s_dbg_uu_mv"] == "4"
    assert variables["s_dbg_uv_mv"] == "4"
    assert variables["s_dbg_uw_mv"] == "4"


def test_common_monitoring_variables_are_in_numeric_list():
    numeric_form = form(load_scope(), "7")
    names = [item.attrib["name"] for item in numeric_form.findall("var")]
    assert REQUIRED_NUMERIC <= set(names)
    assert len(names) == len(set(names))
    for item in numeric_form.findall("var"):
        address = item.attrib.get("addr", "")
        assert address.startswith("0x2000"), (
            f"{item.attrib['name']} has no indexed SRAM address"
        )


def test_current_loop_phase_voltage_debug_symbols_exist():
    source = ISR_SOURCE.read_text(encoding="utf-8")
    for token in ["s_dbg_uu_mv", "s_dbg_uv_mv", "s_dbg_uw_mv", "foc_inv_clarke"]:
        assert token in source


def test_encoder_snapshot_scope_symbol_is_unambiguous():
    encoder = ENCODER_SOURCE.read_text(encoding="utf-8")
    position = POSITION_SOURCE.read_text(encoding="utf-8")

    assert "static volatile encoder_snapshot_t s_snapshot;" in encoder
    assert "static position_loop_snapshot_t s_snapshot;" not in position


def test_current_loop_phase_voltage_is_cleared_outside_closed_loop():
    source = ISR_SOURCE.read_text(encoding="utf-8")
    assert "static void current_debug_clear_phase_voltage(void)" in source
    assert source.count("current_debug_clear_phase_voltage();") >= 5


if __name__ == "__main__":
    test_connection_and_map_settings_are_preserved()
    test_requested_signals_are_curves()
    test_common_monitoring_variables_are_in_numeric_list()
    test_current_loop_phase_voltage_debug_symbols_exist()
    test_encoder_snapshot_scope_symbol_is_unambiguous()
    test_current_loop_phase_voltage_is_cleared_outside_closed_loop()
    print("lksscope static tests passed")
