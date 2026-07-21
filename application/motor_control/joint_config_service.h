#ifndef JOINT_CONFIG_SERVICE_H
#define JOINT_CONFIG_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "joint_config.h"

typedef struct {
    joint_config_record_t record;
    bool record_present;
    bool crc_valid;
    bool encoder_ready;
    bool restored_joint_valid;
    int32_t restored_joint_mdeg;
    bool service_ready;
} joint_config_service_status_t;

void joint_config_service_init(void);
void joint_config_service_poll(void);
bool joint_config_service_ready(void);
uint8_t joint_config_service_node_id(void);
bool joint_config_service_capture(uint8_t node_id,
                                  int32_t known_mdeg,
                                  int8_t direction,
                                  int32_t min_mdeg,
                                  int32_t max_mdeg);
bool joint_config_service_erase(void);
bool joint_config_service_get_status(joint_config_service_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* JOINT_CONFIG_SERVICE_H */
