#include "joint_config_service.h"

#include <string.h>

#include "encoder_service.h"
#include "flash_joint_config_at32m412.h"
#include "motor_app.h"
#include "motor_control.h"
#include "motor_control_isr.h"
#include "position_loop.h"

static joint_config_record_t s_record;
static bool s_record_present;
static bool s_ready;
static bool s_restored_joint_valid;
static int32_t s_restored_joint_mdeg;

static bool joint_config_service_motor_disabled(void)
{
    const motor_control_t *control;

    control = motor_app_get_control();
    return control != NULL &&
           motor_control_get_state(control) == MOTOR_CONTROL_STATE_DISABLED &&
           !motor_control_isr_open_loop_active() &&
           !motor_control_isr_align_active() &&
           !motor_control_isr_current_active() &&
           !motor_control_isr_speed_active() &&
           !motor_control_isr_position_active();
}

void joint_config_service_init(void)
{
    joint_config_record_t latest;

    memset(&s_record, 0, sizeof(s_record));
    s_record_present = false;
    s_ready = false;
    s_restored_joint_valid = false;
    s_restored_joint_mdeg = 0;

    if (flash_joint_config_read_latest(&latest) &&
        joint_config_record_valid(&latest)) {
        s_record = latest;
        s_record_present = true;
    }
}

void joint_config_service_poll(void)
{
    encoder_snapshot_t snapshot;
    int32_t restored_joint_mdeg;

    if (s_ready || !s_record_present) {
        return;
    }
    if (!encoder_service_get_snapshot(&snapshot) || !snapshot.valid) {
        return;
    }
    if (!joint_config_restore_angle(&s_record,
                                    snapshot.corrected_raw16,
                                    &restored_joint_mdeg)) {
        return;
    }

    position_loop_set_origin(snapshot.control_position_mdeg,
                             restored_joint_mdeg);
    s_restored_joint_mdeg = restored_joint_mdeg;
    s_restored_joint_valid = true;
    s_ready = true;
}

bool joint_config_service_ready(void)
{
    return s_ready;
}

uint8_t joint_config_service_node_id(void)
{
    return s_ready ? s_record.node_id : 0u;
}

bool joint_config_service_capture(uint8_t node_id,
                                  int32_t known_mdeg,
                                  int8_t direction,
                                  int32_t min_mdeg,
                                  int32_t max_mdeg)
{
    encoder_snapshot_t snapshot;
    joint_config_record_t latest;
    joint_config_record_t candidate;
    joint_config_record_t verified;
    uint32_t generation;
    int32_t restored_joint_mdeg;

    if (!joint_config_service_motor_disabled()) {
        return false;
    }
    if (!encoder_service_get_snapshot(&snapshot) || !snapshot.valid) {
        return false;
    }

    generation = 0u;
    if (flash_joint_config_read_latest(&latest)) {
        if (!joint_config_record_valid(&latest)) {
            return false;
        }
        generation = latest.generation + 1u;
    }

    joint_config_make(&candidate,
                      generation,
                      node_id,
                      snapshot.corrected_raw16,
                      known_mdeg,
                      direction,
                      min_mdeg,
                      max_mdeg);
    if (!joint_config_record_valid(&candidate)) {
        return false;
    }
    if (!flash_joint_config_write_next(&candidate)) {
        return false;
    }
    if (!flash_joint_config_read_latest(&verified) ||
        !joint_config_record_valid(&verified) ||
        memcmp(&verified, &candidate, sizeof(candidate)) != 0) {
        return false;
    }
    if (!joint_config_restore_angle(&verified,
                                    snapshot.corrected_raw16,
                                    &restored_joint_mdeg)) {
        return false;
    }

    position_loop_set_origin(snapshot.control_position_mdeg,
                             restored_joint_mdeg);
    s_record = verified;
    s_record_present = true;
    s_restored_joint_mdeg = restored_joint_mdeg;
    s_restored_joint_valid = true;
    s_ready = true;
    return true;
}

bool joint_config_service_erase(void)
{
    if (!joint_config_service_motor_disabled()) {
        return false;
    }
    if (!flash_joint_config_erase_all()) {
        return false;
    }

    memset(&s_record, 0, sizeof(s_record));
    s_record_present = false;
    s_ready = false;
    s_restored_joint_valid = false;
    s_restored_joint_mdeg = 0;
    position_loop_init();
    return true;
}

bool joint_config_service_get_status(joint_config_service_status_t *status)
{
    encoder_snapshot_t snapshot;

    if (status == NULL) {
        return false;
    }

    memset(status, 0, sizeof(*status));
    status->record = s_record;
    status->record_present = s_record_present;
    status->crc_valid = s_record_present &&
                        joint_config_record_valid(&s_record);
    status->encoder_ready = encoder_service_get_snapshot(&snapshot) &&
                            snapshot.valid;
    status->restored_joint_valid = s_restored_joint_valid;
    status->restored_joint_mdeg = s_restored_joint_mdeg;
    status->service_ready = s_ready;
    return true;
}
