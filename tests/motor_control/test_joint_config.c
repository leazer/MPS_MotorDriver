#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "joint_config.h"

typedef char record_size_must_be_36[
    sizeof(joint_config_record_t) == 36u ? 1 : -1];

static void test_record_and_basic_restore(void)
{
    joint_config_record_t cfg;
    int32_t joint_mdeg;

    joint_config_make(&cfg, 7u, 1u, 1000u, 0, 1, -90000, 90000);
    assert(joint_config_record_valid(&cfg));
    assert(joint_config_restore_angle(&cfg, 900u, &joint_mdeg));
    assert(joint_mdeg >= -550 && joint_mdeg <= -548);
    assert(!joint_config_restore_angle(&cfg, 32768u, &joint_mdeg));
}

static void test_crc32_known_vector(void)
{
    static const uint8_t text[] = "123456789";

    assert(joint_config_crc32(text, 9u) == 0xCBF43926u);
}

static void test_node_2_negative_direction_and_wraps(void)
{
    joint_config_record_t cfg;
    int32_t joint_mdeg;

    joint_config_make(&cfg, 8u, 2u, 65500u, 1500, -1, -10000, 10000);
    assert(joint_config_record_valid(&cfg));

    assert(joint_config_restore_angle(&cfg, 100u, &joint_mdeg));
    assert(joint_mdeg >= 752 && joint_mdeg <= 754);

    joint_config_make(&cfg, 9u, 2u, 100u, -1500, -1, -10000, 10000);
    assert(joint_config_restore_angle(&cfg, 65500u, &joint_mdeg));
    assert(joint_mdeg >= -754 && joint_mdeg <= -752);
}

static void test_restore_nonzero_known_and_inclusive_limits(void)
{
    joint_config_record_t cfg;
    int32_t joint_mdeg;

    joint_config_make(&cfg, 10u, 1u, 0u, 12345, 1, 12000, 13000);
    assert(joint_config_restore_angle(&cfg, 0u, &joint_mdeg));
    assert(joint_mdeg == 12345);

    joint_config_make(&cfg, 11u, 1u, 0u, 0, 1, -180000, 0);
    assert(joint_config_restore_angle(&cfg, 32768u, &joint_mdeg));
    assert(joint_mdeg == -180000);
    assert(joint_config_restore_angle(&cfg, 0u, &joint_mdeg));
    assert(joint_mdeg == 0);
}

static void test_record_validation_rejects_bad_fields(void)
{
    joint_config_record_t cfg;

    joint_config_make(&cfg, 1u, 1u, 0u, 0, 1, -1, 1);
    assert(joint_config_record_valid(&cfg));

    cfg.magic ^= 1u;
    assert(!joint_config_record_valid(&cfg));
    cfg.magic ^= 1u;

    cfg.version++;
    assert(!joint_config_record_valid(&cfg));
    cfg.version--;

    cfg.size--;
    assert(!joint_config_record_valid(&cfg));
    cfg.size++;

    cfg.node_id = 3u;
    assert(!joint_config_record_valid(&cfg));
    cfg.node_id = 1u;

    cfg.joint_direction = 0;
    assert(!joint_config_record_valid(&cfg));
    cfg.joint_direction = 1;

    cfg.min_joint_position_mdeg = 2;
    assert(!joint_config_record_valid(&cfg));
    cfg.min_joint_position_mdeg = -1;

    cfg.known_joint_position_mdeg = 2;
    assert(!joint_config_record_valid(&cfg));
    cfg.known_joint_position_mdeg = 0;

    joint_config_make(&cfg, 1u, 1u, 0u, 0, 1, -1, 359998);
    assert(joint_config_record_valid(&cfg));
    joint_config_make(&cfg, 1u, 1u, 0u, 0, 1, -1, 359999);
    assert(!joint_config_record_valid(&cfg));
}

static void test_record_validation_rejects_bad_crc(void)
{
    joint_config_record_t cfg;

    joint_config_make(&cfg, 1u, 1u, 0u, 0, 1, -1, 1);
    cfg.crc32 ^= 0x80000000u;
    assert(!joint_config_record_valid(&cfg));
}

static void test_generation_wrap_ordering(void)
{
    assert(joint_config_generation_newer(2u, 1u));
    assert(!joint_config_generation_newer(1u, 2u));
    assert(!joint_config_generation_newer(5u, 5u));
    assert(joint_config_generation_newer(0u, 0xffffffffu));
    assert(!joint_config_generation_newer(0xffffffffu, 0u));
    assert(!joint_config_generation_newer(0x80000000u, 0u));
}

static void test_select_latest(void)
{
    joint_config_record_t older;
    joint_config_record_t newer;
    joint_config_record_t selected;

    memset(&older, 0, sizeof(older));
    memset(&newer, 0, sizeof(newer));
    assert(!joint_config_select_latest(&older, &newer, &selected));

    joint_config_make(&older, 0xffffffffu, 1u, 0u, 0, 1, -1, 1);
    joint_config_make(&newer, 0u, 2u, 0u, 0, -1, -1, 1);
    assert(joint_config_select_latest(&older, &newer, &selected));
    assert(selected.generation == 0u);
    assert(selected.node_id == 2u);

    newer.crc32 ^= 1u;
    assert(joint_config_select_latest(&older, &newer, &selected));
    assert(selected.generation == 0xffffffffu);
}

int main(void)
{
    test_record_and_basic_restore();
    test_crc32_known_vector();
    test_node_2_negative_direction_and_wraps();
    test_restore_nonzero_known_and_inclusive_limits();
    test_record_validation_rejects_bad_fields();
    test_record_validation_rejects_bad_crc();
    test_generation_wrap_ordering();
    test_select_latest();
    puts("joint config: PASS");
    return 0;
}
