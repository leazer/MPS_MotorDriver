from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "application" / "motor_control" / "encoder_service.h"
SOURCE = ROOT / "application" / "motor_control" / "encoder_service.c"
TRACKER_HEADER = ROOT / "application" / "motor_control" / "encoder_tracker.h"
TRACKER_SOURCE = ROOT / "application" / "motor_control" / "encoder_tracker.c"
ACQ_TIMER_HEADER = ROOT / "platform" / "at32m412" / "encoder_acq_timer_at32m412.h"
ACQ_TIMER_SOURCE = ROOT / "platform" / "at32m412" / "encoder_acq_timer_at32m412.c"
CMAKE = ROOT / "CMakeLists.txt"
MDK_PROJECT = ROOT / "project" / "MDK_V5" / "MPS_MotorDriver.uvprojx"


def read(path):
    assert path.exists(), f"missing {path}"
    return path.read_text(encoding="utf-8")


def test_encoder_service_public_api_exists():
    header = read(HEADER)
    for token in [
        "encoder_snapshot_t",
        "encoder_service_init",
        "encoder_service_update_from_isr",
        "encoder_service_update_sample",
        "encoder_service_acquire_once",
        "encoder_service_poll_once_thread",
        "encoder_service_get_snapshot",
        "encoder_service_get_electrical_angle_rad",
        "encoder_service_get_raw16",
        "encoder_service_set_zero",
        "encoder_service_get_zero",
        "encoder_service_set_calibration_table",
        "encoder_service_reset_diagnostics",
        "last_rejected_raw16",
        "last_rejected_delta",
    ]:
        assert token in header


def test_snapshot_exposes_raw_and_calibrated_electrical_angles():
    header = read(HEADER)
    source = read(SOURCE)
    assert "int32_t  raw_elec_mrad;" in header
    assert "encoder_elec_mrad_from_position(raw)" in source
    assert "encoder_elec_mrad_from_position(corrected)" in source


def test_encoder_service_source_is_in_build():
    cmake = read(CMAKE)
    assert "application/motor_control/encoder_service.c" in cmake


def test_encoder_service_source_is_in_keil_project():
    mdk = read(MDK_PROJECT)
    assert "<FileName>encoder_service.c</FileName>" in mdk
    assert "..\\..\\application\\motor_control\\encoder_service.c" in mdk


def test_encoder_service_does_not_use_rtthread_api_in_isr_update():
    source = read(SOURCE)
    update_start = source.index("int encoder_service_update_from_isr(void)")
    update_body = source[update_start:source.index("int encoder_service_poll_once_thread", update_start)]
    assert "rt_" not in update_body


def test_encoder_service_can_resync_after_stale_or_repeated_spikes():
    source = read(SOURCE)
    poll_start = source.index("int encoder_service_poll_once_thread(void)")
    poll_body = source[poll_start:source.index("bool encoder_service_get_snapshot", poll_start)]
    reset_start = source.index("void encoder_service_reset_diagnostics(void)")
    reset_body = source[reset_start:]
    assert "s_has_prev = false" in poll_body
    assert "s_has_prev = false" in reset_body
    assert "ENC_CONSEC_ERROR_THRESHOLD" in source
    assert "encoder_accept_sample(raw, speed, 0)" in source


def test_only_platform_adapter_calls_ma600a_read_angle_speed():
    callers = []
    for path in [
        ROOT / "application" / "motor_control" / "encoder_service.c",
        ROOT / "application" / "motor_control" / "motor_calibration.c",
        ROOT / "application" / "motor_control" / "motor_control_isr.c",
        ROOT / "platform" / "at32m412" / "motor_encoder_at32m412.c",
    ]:
        text = read(path)
        if "ma600a_read_angle_and_speed_raw" in text:
            callers.append(path.as_posix())
    expected = (ROOT / "platform" / "at32m412" / "motor_encoder_at32m412.c").as_posix()
    assert callers == [expected]


def test_foc_isr_does_not_read_encoder_spi():
    isr = read(ROOT / "application" / "motor_control" / "motor_control_isr.c")
    assert "encoder_service_update_from_isr" not in isr
    assert "motor_encoder_read_raw_frame" not in isr
    assert "ma600a_read_angle_and_speed_raw" not in isr
    assert "encoder_tracker_get_electrical_angle_rad" in isr


def test_encoder_tracker_module_exists_and_is_built():
    header = read(TRACKER_HEADER)
    source = read(TRACKER_SOURCE)
    cmake = read(CMAKE)
    mdk = read(MDK_PROJECT)
    for token in [
        "encoder_tracker_init",
        "encoder_tracker_update_sample",
        "encoder_tracker_tick",
        "encoder_tracker_get_electrical_angle_rad",
        "encoder_tracker_get_speed_rad_s",
        "encoder_tracker_get_sample_age_ticks",
    ]:
        assert token in header + source
    assert "application/motor_control/encoder_tracker.c" in cmake
    assert "<FileName>encoder_tracker.c</FileName>" in mdk


def test_isr_uses_tracker_sample_age_for_sensor_failures():
    isr = read(ROOT / "application" / "motor_control" / "motor_control_isr.c")
    assert "encoder_tracker_get_sample_age_ticks" in isr
    assert "s_enc_consec_fail" not in isr


def test_encoder_acquisition_uses_hardware_timer_not_thread():
    header = read(ACQ_TIMER_HEADER)
    source = read(ACQ_TIMER_SOURCE)
    app = read(ROOT / "application" / "motor_app.c")
    cmake = read(CMAKE)
    mdk = read(MDK_PROJECT)
    for token in [
        "encoder_acq_timer_at32m412_init",
        "TMR7_GLOBAL_IRQHandler",
        "encoder_service_acquire_once",
        "TMR7_GLOBAL_IRQn",
    ]:
        assert token in header + source
    assert "encoder_acq_timer_at32m412_init" in app
    assert "encoder_acq_thread" not in app
    assert "rt_thread_init(&s_encoder_acq_thread" not in app
    assert "platform/at32m412/encoder_acq_timer_at32m412.c" in cmake
    assert "<FileName>encoder_acq_timer_at32m412.c</FileName>" in mdk


def test_calibration_consumes_encoder_service_snapshots():
    header = read(ROOT / "application" / "motor_control" / "motor_calibration.h")
    source = read(ROOT / "application" / "motor_control" / "motor_calibration.c")
    for token in [
        "cal_mode_t",
        "motor_calibration_quality_t",
        "motor_calibration_start_mode",
        "motor_calibration_stop_manual",
        "motor_calibration_get_quality",
    ]:
        assert token in header
    assert "encoder_service_get_snapshot" in source
    assert "motor_encoder_read_angle_speed" not in source


def test_encoder_shell_commands_exist():
    shell = read(ROOT / "application" / "motor_shell.c")
    for token in [
        "enc_status",
        "enc_diag_reset",
        "enc_zero",
        "enc_cal_start",
        "enc_cal_stop",
        "enc_cal_status",
        "enc_cal_dump",
        "MSH_CMD_EXPORT(enc_status",
        "last_rej",
    ]:
        assert token in shell


def test_enc_status_polls_once_when_motor_is_idle():
    shell = read(ROOT / "application" / "motor_shell.c")
    start = shell.index("static void enc_status")
    body = shell[start:shell.index("MSH_CMD_EXPORT(enc_status", start)]
    assert "encoder_service_poll_once_thread" in body
    assert "motor_control_isr_open_loop_active" in body
    assert "motor_control_isr_align_active" in body
    assert "motor_control_isr_current_active" in body


if __name__ == "__main__":
    test_encoder_service_public_api_exists()
    test_snapshot_exposes_raw_and_calibrated_electrical_angles()
    test_encoder_service_source_is_in_build()
    test_encoder_service_source_is_in_keil_project()
    test_encoder_service_does_not_use_rtthread_api_in_isr_update()
    test_encoder_service_can_resync_after_stale_or_repeated_spikes()
    test_only_platform_adapter_calls_ma600a_read_angle_speed()
    test_foc_isr_does_not_read_encoder_spi()
    test_encoder_tracker_module_exists_and_is_built()
    test_isr_uses_tracker_sample_age_for_sensor_failures()
    test_encoder_acquisition_uses_hardware_timer_not_thread()
    test_calibration_consumes_encoder_service_snapshots()
    test_encoder_shell_commands_exist()
    test_enc_status_polls_once_when_motor_is_idle()
    print("encoder_service static tests passed")
