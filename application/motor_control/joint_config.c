#include "joint_config.h"

#include <stddef.h>
#include <string.h>

#define JOINT_CONFIG_FULL_TURN_MDEG 360000LL
#define JOINT_CONFIG_RAW16_TURN     65536LL

typedef char joint_config_record_size_must_be_36[
    sizeof(joint_config_record_t) == 36u ? 1 : -1];
typedef char joint_config_crc32_offset_must_be_32[
    offsetof(joint_config_record_t, crc32) == 32u ? 1 : -1];

uint32_t joint_config_crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xffffffffu;
    uint32_t index;

    for (index = 0u; index < length; ++index) {
        uint32_t bit;

        crc ^= data[index];
        for (bit = 0u; bit < 8u; ++bit) {
            crc = (crc >> 1u) ^ ((crc & 1u) != 0u ? 0xedb88320u : 0u);
        }
    }

    return crc ^ 0xffffffffu;
}

void joint_config_make(joint_config_record_t *record, uint32_t generation,
                       uint8_t node_id, uint16_t zero_corrected_raw16,
                       int32_t known_joint_position_mdeg,
                       int8_t joint_direction,
                       int32_t min_joint_position_mdeg,
                       int32_t max_joint_position_mdeg)
{
    if (record == NULL) {
        return;
    }

    memset(record, 0, sizeof(*record));
    record->magic = JOINT_CONFIG_MAGIC;
    record->version = JOINT_CONFIG_VERSION;
    record->size = (uint16_t)sizeof(*record);
    record->generation = generation;
    record->node_id = node_id;
    record->joint_direction = joint_direction;
    record->zero_corrected_raw16 = zero_corrected_raw16;
    record->known_joint_position_mdeg = known_joint_position_mdeg;
    record->min_joint_position_mdeg = min_joint_position_mdeg;
    record->max_joint_position_mdeg = max_joint_position_mdeg;
    record->crc32 = joint_config_crc32((const uint8_t *)record,
                                       (uint32_t)offsetof(joint_config_record_t,
                                                          crc32));
}

bool joint_config_record_valid(const joint_config_record_t *record)
{
    int64_t range_width;

    if (record == NULL || record->magic != JOINT_CONFIG_MAGIC ||
        record->version != JOINT_CONFIG_VERSION ||
        record->size != sizeof(*record) ||
        (record->node_id != 1u && record->node_id != 2u) ||
        (record->joint_direction != 1 && record->joint_direction != -1) ||
        record->min_joint_position_mdeg > record->known_joint_position_mdeg ||
        record->known_joint_position_mdeg > record->max_joint_position_mdeg) {
        return false;
    }

    range_width = (int64_t)record->max_joint_position_mdeg -
                  (int64_t)record->min_joint_position_mdeg;
    if (range_width >= JOINT_CONFIG_FULL_TURN_MDEG) {
        return false;
    }

    return record->crc32 == joint_config_crc32(
        (const uint8_t *)record,
        (uint32_t)offsetof(joint_config_record_t, crc32));
}

bool joint_config_generation_newer(uint32_t candidate, uint32_t reference)
{
    const uint32_t delta = candidate - reference;

    return delta != 0u && delta <= 0x7fffffffu;
}

bool joint_config_select_latest(const joint_config_record_t *first,
                                const joint_config_record_t *second,
                                joint_config_record_t *selected)
{
    const bool first_valid = joint_config_record_valid(first);
    const bool second_valid = joint_config_record_valid(second);

    if (selected == NULL || (!first_valid && !second_valid)) {
        return false;
    }

    if (!first_valid ||
        (second_valid && joint_config_generation_newer(second->generation,
                                                        first->generation))) {
        *selected = *second;
    } else {
        *selected = *first;
    }

    return true;
}

bool joint_config_restore_angle(const joint_config_record_t *record,
                                uint16_t corrected_raw16,
                                int32_t *joint_position_mdeg)
{
    uint16_t raw_delta;
    int32_t signed_raw_delta;
    int64_t base_angle;
    int64_t candidate;
    int64_t result = 0;
    int32_t candidate_count = 0;
    int32_t k;

    if (joint_position_mdeg == NULL || !joint_config_record_valid(record)) {
        return false;
    }

    raw_delta = (uint16_t)(corrected_raw16 - record->zero_corrected_raw16);
    signed_raw_delta = raw_delta <= 32767u ? (int32_t)raw_delta :
                       (int32_t)raw_delta - 65536;
    base_angle = (int64_t)record->known_joint_position_mdeg +
                 ((int64_t)signed_raw_delta * record->joint_direction *
                  JOINT_CONFIG_FULL_TURN_MDEG) / JOINT_CONFIG_RAW16_TURN;

    for (k = -2; k <= 2; ++k) {
        candidate = base_angle + ((int64_t)k * JOINT_CONFIG_FULL_TURN_MDEG);
        if (candidate >= record->min_joint_position_mdeg &&
            candidate <= record->max_joint_position_mdeg) {
            result = candidate;
            ++candidate_count;
        }
    }

    if (candidate_count != 1) {
        return false;
    }

    *joint_position_mdeg = (int32_t)result;
    return true;
}
