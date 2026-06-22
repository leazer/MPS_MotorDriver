/*
 * test_motor_calibration.c - 旁轴标定数据结构与算法单元测试 (Stage 4b)
 *
 * 主机端 gcc 编译运行. 测试内容:
 *   1. motor_calibration_t 结构体大小 = 528 字节 (无 padding, FLASH 布局正确)
 *   2. CRC 覆盖范围 = 516 字节 (table + mech_zero + pole_pairs + reserved2)
 *   3. 软件 CRC32 与 IEEE 标准 CRC-32 一致 (验证硬件 CRC 配置假设)
 *   4. 电角度换算数学: mech_raw + zero + pole_pairs -> theta_e
 *   5. 256 点查表插值数学: raw_16 + table -> corrected_16
 *
 * 注: 产品代码依赖 AT32 硬件 (FLASH/CRC/SPI), 无法直接主机编译.
 *     本测试独立实现等价算法, 验证数学正确性与数据布局.
 */
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ===== 复制产品代码的关键定义 (验证布局一致性) ===== */

#define CAL_TABLE_POINTS        256u
#define CAL_MAGIC               0x304C4143u
#define CAL_VERSION             1u
#define CAL_CRC_PAYLOAD_SIZE    (CAL_TABLE_POINTS * 2u + 2u + 1u + 1u)  /* 516 */
#define MOTOR_POLE_PAIRS        7u

typedef struct {
    uint32_t magic;
    uint8_t  version;
    uint8_t  reserved[3];
    uint32_t timestamp_ms;
    int16_t  table[CAL_TABLE_POINTS];
    uint16_t mech_zero_raw;
    uint8_t  pole_pairs;
    uint8_t  reserved2;
    uint32_t crc32;
} motor_calibration_t;

/* ===== 测试 1: 结构体大小 ===== */
/* spec §4.7.3 标注 528 字节, 但实际布局 (64-bit 与 32-bit 一致):
 *   magic(4@0) version(1@4) reserved[3](3@5) timestamp(4@8) table[256](512@12)
 *   mech_zero(2@524) pole_pairs(1@526) reserved2(1@527) crc32(4@528) = 532
 * spec 数字有误, 产品代码用 sizeof 处理, 不硬编码. */
static void test_struct_size(void)
{
    assert(sizeof(motor_calibration_t) == 532);
    printf("[PASS] struct size = %u bytes (spec said 528, actual 532 due to layout)\n",
           (unsigned)sizeof(motor_calibration_t));
}

/* ===== 测试 2: CRC 覆盖范围 ===== */
static void test_crc_payload_size(void)
{
    /* CRC 覆盖 table(512) + mech_zero(2) + pole_pairs(1) + reserved2(1) */
    assert(CAL_CRC_PAYLOAD_SIZE == 516);
    /* 验证 table 字段在结构体中的偏移 = 12 */
    assert((size_t)((uint8_t *)&((motor_calibration_t*)0)->table[0] -
                    (uint8_t *)0) == 12);
    /* 验证 crc32 字段在结构体末尾 (偏移 528) */
    assert((size_t)((uint8_t *)&((motor_calibration_t*)0)->crc32 -
                    (uint8_t *)0) == 528);
    printf("[PASS] CRC payload size = %u, table offset = 12, crc32 offset = 528\n",
           (unsigned)CAL_CRC_PAYLOAD_SIZE);
}

/* ===== 测试 3: 软件 CRC32 (IEEE, 与 AT32 硬件 CRC 默认配置一致) ===== */
/* AT32 CRC 默认: poly 0x04C11DB7, 输入按 word 反转, 输出反转, init 0xFFFFFFFF
 * 这等价于标准 IEEE CRC-32 (zlib crc32). */
static uint32_t sw_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;
    for (i = 0; i < len; i++) {
        uint32_t j;
        crc ^= (uint32_t)data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 1u) {
                crc = (crc >> 1) ^ 0xEDB88320u;   /* 反转多项式 */
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

static void test_crc32_known_vectors(void)
{
    /* 已知 CRC-32 测试向量 (zlib/IEEE) */
    assert(sw_crc32((const uint8_t *)"", 0) == 0x00000000u);
    assert(sw_crc32((const uint8_t *)"a", 1) == 0xE8B7BE43u);
    assert(sw_crc32((const uint8_t *)"123456789", 9) == 0xCBF43926u);
    assert(sw_crc32((const uint8_t *)"The quick brown fox jumps over the lazy dog", 43) == 0x414FA339u);
    printf("[PASS] CRC32 known vectors match IEEE/zlib standard\n");
}

/* ===== 测试 4: 电角度换算数学 (spec §4.5.2) ===== */
/* theta_e = (mech_diff * POLE_PAIRS * 2π) / 65536, mod 2π
 * mech_diff = (raw_corrected - zero) & 0xFFFF */
static float compute_theta_e(uint16_t raw_corrected, uint16_t zero)
{
    uint16_t mech_diff = (uint16_t)(raw_corrected - zero);
    float theta_e = ((float)mech_diff * (float)MOTOR_POLE_PAIRS * 2.0f * 3.14159265f) / 65536.0f;
    theta_e = fmodf(theta_e, 2.0f * 3.14159265f);
    if (theta_e < 0.0f) theta_e += 2.0f * 3.14159265f;
    return theta_e;
}

static void test_electrical_angle(void)
{
    /* zero=0, raw=0 -> theta_e=0 */
    assert(fabsf(compute_theta_e(0, 0)) < 1e-4f);

    /* zero=0, raw=65536/7 (一圈电角度) -> theta_e≈2π (mod 后≈0) */
    {
        uint16_t raw = (uint16_t)(65536u / 7u);
        float theta = compute_theta_e(raw, 0);
        assert(theta < 1e-2f || theta > (2.0f * 3.14159265f - 1e-2f));
    }

    /* zero=1000, raw=1000 -> theta_e=0 (减零点后为 0) */
    assert(fabsf(compute_theta_e(1000, 1000)) < 1e-4f);

    /* zero=1000, raw=1000+65536/7 -> theta_e≈0 (一圈电角度) */
    {
        uint16_t raw = (uint16_t)(1000u + 65536u / 7u);
        float theta = compute_theta_e(raw, 1000);
        assert(theta < 1e-2f || theta > (2.0f * 3.14159265f - 1e-2f));
    }

    /* 方向: raw 增加 -> theta_e 增加 (正方向) */
    {
        float t1 = compute_theta_e(100, 0);
        float t2 = compute_theta_e(200, 0);
        assert(t2 > t1);
    }
    printf("[PASS] electrical angle conversion (pole_pairs=%u)\n", MOTOR_POLE_PAIRS);
}

/* ===== 测试 5: 256 点查表插值数学 (spec §4.7.6) ===== */
/* idx = raw_16 >> 8 (0..255), frac = (raw_16 * 256) & 0xFFFF
 * off_mdeg = table[idx] + (table[idx+1] - table[idx]) * frac / 65536
 * off_raw = off_mdeg * 65536 / 360000
 * corrected = raw_16 - off_raw */
static uint16_t apply_correction(uint16_t raw_16, const int16_t table[256])
{
    uint32_t idx_frac_q24 = (uint32_t)raw_16 * 256u;
    uint16_t idx  = (uint16_t)(idx_frac_q24 >> 16);
    uint16_t frac = (uint16_t)(idx_frac_q24 & 0xFFFFu);
    int16_t  off0 = table[idx];
    int16_t  off1 = table[(idx + 1u) & 0xFFu];
    int32_t  off_mdeg = (int32_t)off0 + (((int32_t)(off1 - off0) * (int32_t)frac) >> 16);
    int32_t  off_raw = (off_mdeg * 65536) / 360000;
    return (uint16_t)((int32_t)raw_16 - off_raw);
}

static void test_correction_table(void)
{
    int16_t table[256];
    uint16_t i;

    /* 全零表: 校正后 = 原值 */
    memset(table, 0, sizeof(table));
    assert(apply_correction(0, table) == 0);
    assert(apply_correction(1000, table) == 1000);
    assert(apply_correction(65535, table) == 65535);

    /* 常数偏移表: 校正后 = 原值 - 常数偏移 (1° = 1000 mdeg) */
    /* off_raw = 1000 * 65536 / 360000 = 182 LSB */
    for (i = 0; i < 256; i++) table[i] = 1000;
    {
        uint16_t raw = 10000;
        uint16_t corrected = apply_correction(raw, table);
        assert(corrected == (uint16_t)(10000 - 182));   /* 9818 */
    }

    /* 线性渐变表: 插值应平滑 */
    for (i = 0; i < 256; i++) table[i] = (int16_t)(i * 10);  /* 0..2550 mdeg */
    {
        /* raw=32768 (idx=128, frac=0): off_mdeg=1280, off_raw=1280*65536/360000=233 */
        uint16_t raw = 32768u;
        uint16_t corrected = apply_correction(raw, table);
        int32_t expected_off_raw = (1280 * 65536) / 360000;
        assert(corrected == (uint16_t)(32768 - expected_off_raw));
    }

    printf("[PASS] 256-point correction table interpolation\n");
}

/* ===== 测试 6: 标定数据结构 CRC 完整性 ===== */
static void test_calibration_struct_crc(void)
{
    motor_calibration_t cal;
    uint32_t crc;
    memset(&cal, 0, sizeof(cal));
    cal.magic = CAL_MAGIC;
    cal.version = CAL_VERSION;
    cal.mech_zero_raw = 12345;
    cal.pole_pairs = MOTOR_POLE_PAIRS;
    /* 填一个已知模式的表 */
    {
        uint16_t i;
        for (i = 0; i < CAL_TABLE_POINTS; i++) {
            cal.table[i] = (int16_t)(i % 100 - 50);
        }
    }
    /* 计算 CRC (覆盖 table+mech_zero+pole_pairs+reserved2, 516 字节) */
    crc = sw_crc32((const uint8_t *)&cal.table[0], CAL_CRC_PAYLOAD_SIZE);
    cal.crc32 = crc;

    /* 验证: 回读校验通过 */
    {
        uint32_t crc2 = sw_crc32((const uint8_t *)&cal.table[0], CAL_CRC_PAYLOAD_SIZE);
        assert(crc2 == cal.crc32);
    }

    /* 篡改 1 字节后 CRC 不匹配 */
    cal.table[100] += 1;
    {
        uint32_t crc3 = sw_crc32((const uint8_t *)&cal.table[0], CAL_CRC_PAYLOAD_SIZE);
        assert(crc3 != cal.crc32);
    }
    printf("[PASS] calibration struct CRC integrity check\n");
}

int main(void)
{
    test_struct_size();
    test_crc_payload_size();
    test_crc32_known_vectors();
    test_electrical_angle();
    test_correction_table();
    test_calibration_struct_crc();
    printf("\nAll motor_calibration tests passed.\n");
    return 0;
}
