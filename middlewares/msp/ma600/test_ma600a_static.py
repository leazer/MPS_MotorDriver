from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
MA600_DIR = ROOT / "middlewares" / "msp" / "ma600"
HEADER = MA600_DIR / "ma600a.h"
SOURCE = MA600_DIR / "ma600a.c"
AT32_PORT_HEADER = MA600_DIR / "ma600a_at32_spi2.h"
AT32_PORT_SOURCE = MA600_DIR / "ma600a_at32_spi2.c"
CMAKE = ROOT / "CMakeLists.txt"
MDK_PROJECT = ROOT / "project" / "MDK_V5" / "MPS_MotorDriver.uvprojx"
MAIN = ROOT / "project" / "src" / "main.c"
MOTOR_APP = ROOT / "application" / "motor_app.c"


def read(path):
    assert path.exists(), f"missing {path}"
    return path.read_text(encoding="utf-8")


def test_public_api_and_bus_callbacks():
    header = read(HEADER)

    for token in [
        "typedef struct ma600a_bus",
        "void *user",
        "spi_transfer16",
        "cs_write",
        "delay_us",
        "delay_ms",
        "ma600a_init",
        "ma600a_read_angle_raw",
        "ma600a_read_angle_and_speed_raw",
        "ma600a_read_speed_raw",
        "ma600a_set_mtsp_speed",
        "ma600a_write_bct",
        "ma600a_write_zero",
        "ma600a_write_correction_table",
        "ma600a_store_config_to_nvm",
        "ma600a_store_correction_to_nvm",
        "ma600a_restore_from_nvm",
        "ma600a_speed_raw_to_rpm",
        "ma600a_speed_raw_to_rpm_with_ck100",
    ]:
        assert token in header


def test_protocol_constants_and_registers_are_present():
    source = read(SOURCE)

    for token in [
        "0xD200u",
        "0xEA54u",
        "0xEA55u",
        "0xEA56u",
        "MA600A_REG_ZERO0",
        "MA600A_REG_ZERO1",
        "MA600A_REG_BCT0",
        "MA600A_REG_BCT1",
        "MA600A_REG_PRT",
        "MA600A_PRT_MTSP",
        "MA600A_REG_CORR0",
        "MA600A_CORRECTION_TABLE_SIZE",
    ]:
        assert token in source


def test_speed_read_uses_32bit_frame_and_preserves_prt_bits():
    source = read(SOURCE)

    assert "ma600a_read_angle_and_speed_raw" in source
    assert "ma600a_transfer32_zero" in source
    assert "dev->bus.cs_write(dev->bus.user, MA600A_CS_LOW);" in source
    assert "dev->bus.cs_write(dev->bus.user, MA600A_CS_HIGH);" in source
    assert "dev->bus.spi_transfer16(dev->bus.user, 0x0000u, angle)" in source
    assert "dev->bus.spi_transfer16(dev->bus.user, 0x0000u, data)" in source
    assert "ma600a_update_register_bits(dev, MA600A_REG_PRT, MA600A_PRT_MTSP" in source
    assert "(int16_t)speed_word" in source
    assert "5.722f" in source
    assert "f_ck100_khz / 100.0f" in source


def test_angle_read_does_not_enable_aprt_parity():
    source = read(SOURCE)
    header = read(HEADER)
    platform = read(ROOT / "platform" / "at32m412" / "motor_encoder_at32m412.c")

    assert "ma600a_set_angle_parity" not in header + source + platform
    assert "MA600A_PRT_APRT" not in source
    assert "ma600a_angle_parity_ok" not in source
    assert "& 0xFFFEu" not in source


def test_at32_spi2_uses_datasheet_supported_mode0():
    source = read(ROOT / "platform" / "at32m412" / "motor_encoder_at32m412.c")

    assert "SPI_CLOCK_POLARITY_LOW" in source
    assert "SPI_CLOCK_PHASE_1EDGE" in source
    assert "SPI_CLOCK_PHASE_2EDGE" not in source


def test_driver_is_decoupled_from_at32_spi_gpio():
    source = read(SOURCE)

    forbidden = [
        "SPI2",
        "wk_spi",
        "wk_gpio",
        "gpio_bits",
        "spi_i2s",
        "at32m412_416",
    ]
    for token in forbidden:
        assert token not in source


def test_project_cmake_includes_ma600_driver():
    cmake = read(CMAKE)

    assert "middlewares/msp/ma600/ma600a.c" in cmake
    assert "middlewares/msp/ma600/ma600a_at32_spi2.c" in cmake
    assert "project/src/ma600a_debug.c" not in cmake
    assert "middlewares/msp/ma600" in cmake


def test_at32_spi2_port_uses_callbacks_and_generated_pins():
    header = read(AT32_PORT_HEADER)
    source = read(AT32_PORT_SOURCE)

    for token in [
        "ma600a_at32_spi2_bus_get",
        "ma600a_at32_spi2_transfer16",
        "ma600a_at32_spi2_delay_us",
    ]:
        assert token in header + source

    for token in [
        "SPI2",
        "SPI2_CS_GPIO_PORT",
        "SPI2_CS_PIN",
        "spi_i2s_data_transmit",
        "spi_i2s_data_receive",
        "SPI_I2S_TDBE_FLAG",
        "SPI_I2S_RDBF_FLAG",
    ]:
        assert token in source


def test_main_and_mdk_project_do_not_include_debug_poll_support():
    main = read(MAIN)
    motor_app = read(MOTOR_APP)
    mdk = read(MDK_PROJECT)

    assert '#include "motor_app.h"' in main
    assert "ma600a_debug" not in motor_app
    assert "ma600a.c" in mdk
    assert "ma600a_at32_spi2.c" in mdk
    assert "ma600a_debug.c" not in mdk


if __name__ == "__main__":
    tests = [
        test_public_api_and_bus_callbacks,
        test_protocol_constants_and_registers_are_present,
        test_speed_read_uses_32bit_frame_and_preserves_prt_bits,
        test_angle_read_does_not_enable_aprt_parity,
        test_at32_spi2_uses_datasheet_supported_mode0,
        test_driver_is_decoupled_from_at32_spi_gpio,
        test_project_cmake_includes_ma600_driver,
        test_at32_spi2_port_uses_callbacks_and_generated_pins,
        test_main_and_mdk_project_do_not_include_debug_poll_support,
    ]
    for test in tests:
        test()
    print(f"{len(tests)} ma600a static tests passed")
