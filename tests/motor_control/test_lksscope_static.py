from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
SCOPE = ROOT / "debug.lksscope"
ISR_SOURCE = ROOT / "application" / "motor_control" / "motor_control_isr.c"

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

REQUIRED_NUMERIC = {
    "s_motor_control.state",
    "s_motor_control.mode",
    "s_motor_control.fault_flags",
    "s_motor_control.iq_ref_ma",
    "s_motor_control.speed_ref_rpm",
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
}


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
    assert form(root, "2").attrib["mapFile"] == "./project/MDK_V5/objects/MPS_MotorDriver.axf"


def test_requested_signals_are_curves():
    curve_form = form(load_scope(), "8")
    names = [item.attrib["name"] for item in curve_form.findall("var")]
    assert set(names) == REQUIRED_CURVES
    assert len(names) == len(set(names))
    assert curve_form.attrib["subMode"] == "1"
    assert {item.attrib["subPlot"] for item in curve_form.findall("var")} == {"0", "1", "2", "3"}


def test_common_monitoring_variables_are_in_numeric_list():
    numeric_form = form(load_scope(), "7")
    names = [item.attrib["name"] for item in numeric_form.findall("var")]
    assert REQUIRED_NUMERIC <= set(names)
    assert len(names) == len(set(names))
    for item in numeric_form.findall("var"):
        assert item.attrib.get("addr", "").startswith("0x2000")


def test_current_loop_phase_voltage_debug_symbols_exist():
    source = ISR_SOURCE.read_text(encoding="utf-8")
    for token in ["s_dbg_uu_mv", "s_dbg_uv_mv", "s_dbg_uw_mv", "foc_inv_clarke"]:
        assert token in source


if __name__ == "__main__":
    test_connection_and_map_settings_are_preserved()
    test_requested_signals_are_curves()
    test_common_monitoring_variables_are_in_numeric_list()
    test_current_loop_phase_voltage_debug_symbols_exist()
    print("lksscope static tests passed")
