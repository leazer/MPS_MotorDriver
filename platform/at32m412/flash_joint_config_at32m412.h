#ifndef FLASH_JOINT_CONFIG_AT32M412_H
#define FLASH_JOINT_CONFIG_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "joint_config.h"

/* Reads the newest valid joint record from the two reserved Flash pages. */
bool flash_joint_config_read_latest(joint_config_record_t *record);

/* Writes an already-finalized valid record to the inactive Flash page. */
bool flash_joint_config_write_next(const joint_config_record_t *record);

/* Erases both joint configuration pages; the caller must disable the motor. */
bool flash_joint_config_erase_all(void);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_JOINT_CONFIG_AT32M412_H */
