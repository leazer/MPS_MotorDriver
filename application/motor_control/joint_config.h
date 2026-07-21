#ifndef JOINT_CONFIG_H
#define JOINT_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define JOINT_CONFIG_MAGIC   0x4746434Au
#define JOINT_CONFIG_VERSION 1u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t generation;
    uint8_t node_id;
    int8_t joint_direction;
    uint16_t zero_corrected_raw16;
    int32_t known_joint_position_mdeg;
    int32_t min_joint_position_mdeg;
    int32_t max_joint_position_mdeg;
    uint32_t reserved;
    uint32_t crc32;
} joint_config_record_t;

uint32_t joint_config_crc32(const uint8_t *data, uint32_t length);
void joint_config_make(joint_config_record_t *record, uint32_t generation,
                       uint8_t node_id, uint16_t zero_corrected_raw16,
                       int32_t known_joint_position_mdeg,
                       int8_t joint_direction,
                       int32_t min_joint_position_mdeg,
                       int32_t max_joint_position_mdeg);
bool joint_config_record_valid(const joint_config_record_t *record);
bool joint_config_generation_newer(uint32_t candidate, uint32_t reference);
bool joint_config_select_latest(const joint_config_record_t *first,
                                const joint_config_record_t *second,
                                joint_config_record_t *selected);
bool joint_config_restore_angle(const joint_config_record_t *record,
                                uint16_t corrected_raw16,
                                int32_t *joint_position_mdeg);

#endif
