#include "joint_config_service.h"

#include <string.h>

#if defined(__CC_ARM) || defined(__arm__) || defined(__thumb__)
#include "at32m412_416.h"
#else
#include <stdatomic.h>
#endif

#include "can_motion_service.h"
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
static volatile bool s_mutation_busy;
static volatile bool s_runtime_locked;
static volatile bool s_reboot_required;

#if !defined(__CC_ARM) && !defined(__arm__) && !defined(__thumb__)
static atomic_flag s_joint_config_host_lock = ATOMIC_FLAG_INIT;
#endif

static uint32_t joint_config_service_state_lock(void)
{
#if defined(__CC_ARM) || defined(__arm__) || defined(__thumb__)
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    return primask;
#else
    while (atomic_flag_test_and_set_explicit(&s_joint_config_host_lock,
                                             memory_order_acquire)) {
    }
    return 0u;
#endif
}

static void joint_config_service_state_unlock(uint32_t primask)
{
#if defined(__CC_ARM) || defined(__arm__) || defined(__thumb__)
    __DMB();
    if (primask == 0u) {
        __enable_irq();
    }
#else
    (void)primask;
    atomic_flag_clear_explicit(&s_joint_config_host_lock,
                               memory_order_release);
#endif
}

static bool joint_config_service_begin_mutation(bool allow_runtime_locked)
{
    uint32_t primask;
    bool claimed;

    claimed = false;
    primask = joint_config_service_state_lock();
    if (!s_mutation_busy && !s_reboot_required &&
        (allow_runtime_locked || !s_runtime_locked)) {
        s_mutation_busy = true;
        claimed = true;
    }
    joint_config_service_state_unlock(primask);
    return claimed;
}

static void joint_config_service_end_mutation(void)
{
    uint32_t primask;

    primask = joint_config_service_state_lock();
    s_mutation_busy = false;
    joint_config_service_state_unlock(primask);
}

static void joint_config_service_fail_closed_for_reboot(void)
{
    uint32_t primask;

    primask = joint_config_service_state_lock();
    s_ready = false;
    s_restored_joint_valid = false;
    s_reboot_required = true;
    s_mutation_busy = false;
    joint_config_service_state_unlock(primask);
}

static void joint_config_service_quiesce_motion(void)
{
    uint32_t primask;

    primask = joint_config_service_state_lock();
    s_ready = false;
    s_restored_joint_valid = false;
    s_reboot_required = true;
    can_motion_service_set_joint_config(false, 0u);
    joint_config_service_state_unlock(primask);
}

static void joint_config_service_publish_ready(
    const joint_config_record_t *record,
    int32_t restored_joint_mdeg)
{
    uint32_t primask;

    primask = joint_config_service_state_lock();
    s_record = *record;
    s_record_present = true;
    s_restored_joint_mdeg = restored_joint_mdeg;
    s_restored_joint_valid = true;
    s_ready = true;
    s_mutation_busy = false;
    joint_config_service_state_unlock(primask);
}

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
    s_mutation_busy = false;
    s_runtime_locked = false;
    s_reboot_required = false;

    if (flash_joint_config_read_latest(&latest) &&
        joint_config_record_valid(&latest)) {
        s_record = latest;
        s_record_present = true;
    }
}

void joint_config_service_poll(void)
{
    encoder_snapshot_t snapshot;
    joint_config_record_t record;
    int32_t restored_joint_mdeg;
    uint32_t primask;
    bool restore_needed;

    if (!joint_config_service_begin_mutation(false)) {
        return;
    }

    primask = joint_config_service_state_lock();
    restore_needed = true;
    if (s_ready || !s_record_present) {
        restore_needed = false;
    } else {
        record = s_record;
    }
    joint_config_service_state_unlock(primask);
    if (!restore_needed) {
        joint_config_service_end_mutation();
        return;
    }
    if (!encoder_service_get_snapshot(&snapshot) || !snapshot.valid) {
        joint_config_service_end_mutation();
        return;
    }
    if (!joint_config_restore_angle(&record,
                                    snapshot.corrected_raw16,
                                    &restored_joint_mdeg)) {
        joint_config_service_end_mutation();
        return;
    }

    if (!position_loop_set_joint_origin(snapshot.control_position_mdeg,
                                        restored_joint_mdeg,
                                        record.joint_direction)) {
        joint_config_service_end_mutation();
        return;
    }
    joint_config_service_publish_ready(&record, restored_joint_mdeg);
}

bool joint_config_service_ready(void)
{
    uint32_t primask;
    bool ready;

    primask = joint_config_service_state_lock();
    ready = s_ready;
    joint_config_service_state_unlock(primask);
    return ready;
}

uint8_t joint_config_service_node_id(void)
{
    uint32_t primask;
    uint8_t node_id;

    primask = joint_config_service_state_lock();
    node_id = s_ready ? s_record.node_id : 0u;
    joint_config_service_state_unlock(primask);
    return node_id;
}

bool joint_config_service_lock_runtime(uint8_t *node_id)
{
    uint32_t primask;
    bool locked;

    if (node_id == NULL) {
        return false;
    }

    locked = false;
    primask = joint_config_service_state_lock();
    if (!s_mutation_busy && !s_runtime_locked && !s_reboot_required &&
        s_ready && s_record_present) {
        *node_id = s_record.node_id;
        s_runtime_locked = true;
        locked = true;
    }
    joint_config_service_state_unlock(primask);
    return locked;
}

bool joint_config_service_set_runtime_origin(int32_t sensor_mdeg,
                                             int32_t joint_mdeg)
{
    uint32_t primask;
    bool available;

    if (!joint_config_service_motor_disabled()) {
        return false;
    }
    if (!joint_config_service_begin_mutation(false)) {
        return false;
    }

    primask = joint_config_service_state_lock();
    available = !s_record_present && !s_ready;
    joint_config_service_state_unlock(primask);
    if (!available) {
        joint_config_service_end_mutation();
        return false;
    }
    if (!joint_config_service_motor_disabled()) {
        joint_config_service_end_mutation();
        return false;
    }
    if (!position_loop_set_joint_origin(sensor_mdeg, joint_mdeg, 1)) {
        joint_config_service_end_mutation();
        return false;
    }
    joint_config_service_end_mutation();
    return true;
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
    if (!joint_config_service_begin_mutation(false)) {
        return false;
    }
    if (!encoder_service_get_snapshot(&snapshot) || !snapshot.valid) {
        joint_config_service_end_mutation();
        return false;
    }

    generation = 0u;
    if (flash_joint_config_read_latest(&latest)) {
        if (!joint_config_record_valid(&latest)) {
            joint_config_service_end_mutation();
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
        joint_config_service_end_mutation();
        return false;
    }
    if (!flash_joint_config_write_next(&candidate)) {
        joint_config_service_end_mutation();
        return false;
    }
    if (!flash_joint_config_read_latest(&verified) ||
        !joint_config_record_valid(&verified) ||
        memcmp(&verified, &candidate, sizeof(candidate)) != 0) {
        joint_config_service_fail_closed_for_reboot();
        return false;
    }
    if (!joint_config_restore_angle(&verified,
                                    snapshot.corrected_raw16,
                                    &restored_joint_mdeg)) {
        joint_config_service_fail_closed_for_reboot();
        return false;
    }
    if (!joint_config_service_motor_disabled()) {
        joint_config_service_fail_closed_for_reboot();
        return false;
    }

    if (!position_loop_set_joint_origin(snapshot.control_position_mdeg,
                                        restored_joint_mdeg,
                                        verified.joint_direction)) {
        joint_config_service_fail_closed_for_reboot();
        return false;
    }
    joint_config_service_publish_ready(&verified, restored_joint_mdeg);
    return true;
}

bool joint_config_service_erase(void)
{
    uint32_t primask;
    bool erased;

    if (!joint_config_service_motor_disabled()) {
        return false;
    }
    if (!joint_config_service_begin_mutation(true)) {
        return false;
    }

    joint_config_service_quiesce_motion();
    erased = flash_joint_config_erase_all();
    position_loop_init();

    if (erased) {
        primask = joint_config_service_state_lock();
        memset(&s_record, 0, sizeof(s_record));
        s_record_present = false;
        s_restored_joint_valid = false;
        s_restored_joint_mdeg = 0;
        joint_config_service_state_unlock(primask);
    }
    joint_config_service_end_mutation();
    return erased;
}

bool joint_config_service_get_status(joint_config_service_status_t *status)
{
    encoder_snapshot_t snapshot;
    uint32_t primask;

    if (status == NULL) {
        return false;
    }

    memset(status, 0, sizeof(*status));
    primask = joint_config_service_state_lock();
    status->record = s_record;
    status->record_present = s_record_present;
    status->restored_joint_valid = s_restored_joint_valid;
    status->restored_joint_mdeg = s_restored_joint_mdeg;
    status->service_ready = s_ready;
    status->mutation_busy = s_mutation_busy;
    status->runtime_locked = s_runtime_locked;
    status->reboot_required = s_reboot_required;
    joint_config_service_state_unlock(primask);

    status->crc_valid = status->record_present &&
                        joint_config_record_valid(&status->record);
    status->encoder_ready = encoder_service_get_snapshot(&snapshot) &&
                            snapshot.valid;
    return true;
}
