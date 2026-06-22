/*
 * flash_calibration_at32m412.c - FLASH 标定区读写 + 硬件 CRC32 (Stage 4b)
 *
 * 标定区: FLASH 末 1KB (0x0801FC00), 存 motor_calibration_t (528 字节)
 * 链接脚本 AT32M412xB_FLASH.ld 已将 FLASH LENGTH 缩为 127K, 末 1K 不参与链接
 *
 * FLASH 编程约束 (AT32M412):
 *   - 擦除单位: sector (1KB), flash_sector_erase(0x0801FC00) 擦末页
 *   - 编程单位: word (32-bit), flash_word_program(addr, data)
 *   - 编程前必须 flash_unlock, 完成后 flash_lock
 *   - 只能 1->0, 写前必须擦除 (擦除后全 0xFF)
 *
 * CRC: 硬件 CRC 外设, 默认配置 = 标准 CRC-32 (poly 0x04C11DB7,
 *      输入按 word 反转, 输出反转, init 0xFFFFFFFF), 与 zlib/IEEE 一致
 */
#include "flash_calibration_at32m412.h"
#include "motor_params.h"
#include "at32m412_416.h"
#include <string.h>

/* 标定结构体大小 (528 字节). CRC 覆盖范围 CAL_CRC_PAYLOAD_SIZE (516 字节) 在 motor_params.h */
#define CAL_STRUCT_SIZE         sizeof(motor_calibration_t)

static bool s_crc_inited = false;

/* 首次调用时开启 CRC 时钟. 后续调用跳过, 避免重复开时钟开销. */
static void flash_calibration_crc_clock_init(void)
{
    if (!s_crc_inited) {
        crm_periph_clock_enable(CRM_CRC_PERIPH_CLOCK, TRUE);
        s_crc_inited = true;
    }
}

uint32_t flash_calibration_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t words;
    uint32_t remainder;
    uint32_t i;
    uint32_t word_buf;
    uint32_t crc;

    flash_calibration_crc_clock_init();

    /* 整 word 部分直接喂 CRC 外设 (按 byte 重组为 little-endian word, 匹配结构体布局) */
    words = len / 4u;
    crc_data_reset();
    for (i = 0; i < words; i++) {
        word_buf = ((uint32_t)data[i * 4u]) |
                   ((uint32_t)data[i * 4u + 1u] << 8) |
                   ((uint32_t)data[i * 4u + 2u] << 16) |
                   ((uint32_t)data[i * 4u + 3u] << 24);
        crc_one_word_calculate(word_buf);
    }

    /* 尾部不足 4 字节, 补 0 凑成 word (与结构体 padding 行为一致) */
    remainder = len & 3u;
    if (remainder != 0u) {
        word_buf = 0u;
        for (i = 0; i < remainder; i++) {
            word_buf |= ((uint32_t)data[words * 4u + i]) << (i * 8u);
        }
        crc_one_word_calculate(word_buf);
    }

    crc = crc_data_get();
    return crc;
}

bool flash_calibration_read(motor_calibration_t *cal)
{
    const motor_calibration_t *flash_cal;
    uint32_t crc_stored;
    uint32_t crc_calc;
    const uint8_t *payload;

    if (cal == 0) {
        return false;
    }

    /* 1. 直接读 FLASH (内存映射, 无需解锁) */
    flash_cal = (const motor_calibration_t *)CAL_FLASH_ADDR;

    /* 2. 校验 magic / version */
    if (flash_cal->magic != CAL_MAGIC || flash_cal->version != CAL_VERSION) {
        return false;
    }

    /* 3. 校验 CRC32 (覆盖 table + mech_zero_raw + pole_pairs, 516 字节) */
    payload = (const uint8_t *)&flash_cal->table[0];
    crc_stored = flash_cal->crc32;
    crc_calc = flash_calibration_crc32(payload, CAL_CRC_PAYLOAD_SIZE);
    if (crc_calc != crc_stored) {
        return false;
    }

    /* 4. 全部通过, 拷贝到 RAM */
    memcpy(cal, flash_cal, CAL_STRUCT_SIZE);
    return true;
}

bool flash_calibration_write(const motor_calibration_t *cal)
{
    flash_status_type status;
    uint32_t i;
    const uint32_t *src_words;
    const uint32_t *flash_words;
    uint32_t word_count;

    if (cal == 0) {
        return false;
    }

    word_count = CAL_STRUCT_SIZE / 4u;   /* 528/4 = 132 words */
    src_words = (const uint32_t *)cal;

    flash_unlock();

    /* 1. 擦除末页 sector (1KB) */
    status = flash_sector_erase(CAL_FLASH_ADDR);
    if (status != FLASH_OPERATE_DONE) {
        flash_lock();
        return false;
    }

    /* 2. 按 word 编程 (132 次) */
    for (i = 0; i < word_count; i++) {
        status = flash_word_program(CAL_FLASH_ADDR + i * 4u, src_words[i]);
        if (status != FLASH_OPERATE_DONE) {
            flash_lock();
            return false;
        }
    }

    flash_lock();

    /* 3. 回读校验 (memcmp) */
    flash_words = (const uint32_t *)CAL_FLASH_ADDR;
    for (i = 0; i < word_count; i++) {
        if (flash_words[i] != src_words[i]) {
            return false;
        }
    }
    return true;
}

bool flash_calibration_erase(void)
{
    flash_status_type status;

    flash_unlock();
    status = flash_sector_erase(CAL_FLASH_ADDR);
    flash_lock();

    return (status == FLASH_OPERATE_DONE);
}
