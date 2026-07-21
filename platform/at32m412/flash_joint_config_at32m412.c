#include "flash_joint_config_at32m412.h"

#include <string.h>

#include "at32m412_416.h"
#include "motor_params.h"

#define JOINT_CFG_WORD_COUNT \
    (sizeof(joint_config_record_t) / sizeof(uint32_t))

_Static_assert(sizeof(joint_config_record_t) == 36u,
               "joint config record must occupy nine words");
_Static_assert(JOINT_CFG_WORD_COUNT == 9u,
               "joint config Flash write must program nine words");

static const joint_config_record_t *flash_joint_config_latest(uint32_t *latest_addr)
{
    const joint_config_record_t *record_a =
        (const joint_config_record_t *)JOINT_CFG_FLASH_A_ADDR;
    const joint_config_record_t *record_b =
        (const joint_config_record_t *)JOINT_CFG_FLASH_B_ADDR;
    const bool record_a_valid = joint_config_record_valid(record_a);
    const bool record_b_valid = joint_config_record_valid(record_b);

    if (!record_a_valid && !record_b_valid) {
        return NULL;
    }

    if (record_a_valid &&
        (!record_b_valid || !joint_config_generation_newer(record_b->generation,
                                                             record_a->generation))) {
        *latest_addr = JOINT_CFG_FLASH_A_ADDR;
        return record_a;
    }

    *latest_addr = JOINT_CFG_FLASH_B_ADDR;
    return record_b;
}

bool flash_joint_config_read_latest(joint_config_record_t *record)
{
    uint32_t latest_addr;
    const joint_config_record_t *latest;

    if (record == NULL) {
        return false;
    }

    latest = flash_joint_config_latest(&latest_addr);
    if (latest == NULL) {
        return false;
    }

    *record = *latest;
    return true;
}

bool flash_joint_config_write_next(const joint_config_record_t *record)
{
    const uint32_t *source_words;
    const joint_config_record_t *readback;
    uint32_t latest_addr;
    uint32_t target_addr;
    uint32_t word_index;
    flash_status_type status;

    if (!joint_config_record_valid(record)) {
        return false;
    }

    if (flash_joint_config_latest(&latest_addr) == NULL) {
        target_addr = JOINT_CFG_FLASH_A_ADDR;
    } else if (latest_addr == JOINT_CFG_FLASH_A_ADDR) {
        target_addr = JOINT_CFG_FLASH_B_ADDR;
    } else {
        target_addr = JOINT_CFG_FLASH_A_ADDR;
    }

    source_words = (const uint32_t *)record;
    flash_unlock();

    status = flash_sector_erase(target_addr);
    if (status != FLASH_OPERATE_DONE) {
        flash_lock();
        return false;
    }

    for (word_index = 0u; word_index < JOINT_CFG_WORD_COUNT; ++word_index) {
        status = flash_word_program(target_addr + word_index * sizeof(uint32_t),
                                    source_words[word_index]);
        if (status != FLASH_OPERATE_DONE) {
            flash_lock();
            return false;
        }
    }

    flash_lock();

    readback = (const joint_config_record_t *)target_addr;
    if (memcmp(readback, record, sizeof(*record)) != 0 ||
        !joint_config_record_valid(readback)) {
        return false;
    }

    return true;
}

bool flash_joint_config_erase_all(void)
{
    flash_status_type status;

    flash_unlock();
    status = flash_sector_erase(JOINT_CFG_FLASH_A_ADDR);
    if (status == FLASH_OPERATE_DONE) {
        status = flash_sector_erase(JOINT_CFG_FLASH_B_ADDR);
    }
    flash_lock();

    return status == FLASH_OPERATE_DONE;
}
