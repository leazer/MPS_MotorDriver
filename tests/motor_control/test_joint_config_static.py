from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PARAMS = ROOT / "application" / "motor_control" / "motor_params.h"
LINKER = ROOT / "AT32M412xB_FLASH.ld"
MDK = ROOT / "project" / "MDK_V5" / "MPS_MotorDriver.uvprojx"
HEADER = ROOT / "platform" / "at32m412" / "flash_joint_config_at32m412.h"
SOURCE = ROOT / "platform" / "at32m412" / "flash_joint_config_at32m412.c"


def read(path):
    assert path.exists(), f"missing {path}"
    return path.read_text(encoding="utf-8")


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


if __name__ == "__main__":
    test_flash_layout_reserves_two_joint_pages_before_calibration()
    test_joint_storage_contract_uses_inactive_page_and_verified_word_write()
    print("joint config static tests passed")
