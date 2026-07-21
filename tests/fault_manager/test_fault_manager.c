/* tests/fault_manager/test_fault_manager.c */
#include "fault_manager.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    fault_manager_init();
    assert(fault_manager_get() == FAULT_NONE);
    assert(!fault_manager_any());

    fault_manager_set_bits(FAULT_OVERCURRENT);
    assert(fault_manager_get() == FAULT_OVERCURRENT);
    assert(fault_manager_any());

    fault_manager_set_bits(FAULT_SENSOR);
    assert(fault_manager_get() == (FAULT_OVERCURRENT | FAULT_SENSOR));

    fault_manager_clear_bits(FAULT_OVERCURRENT);
    assert(fault_manager_get() == FAULT_SENSOR);
    assert(fault_manager_any());

    fault_manager_clear_all();
    assert(fault_manager_get() == FAULT_NONE);
    assert(!fault_manager_any());

    fault_manager_set_bits(FAULT_POSITION_TRACKING);
    assert(fault_manager_any_fatal());
    assert(fault_manager_get() == FAULT_POSITION_TRACKING);
    fault_manager_clear_bits(FAULT_POSITION_TRACKING);
    assert(!fault_manager_any_fatal());
    assert(fault_manager_get() == FAULT_NONE);

    fault_manager_set_bits(FAULT_CAN_TIMEOUT);
    assert(fault_manager_any_fatal());
    fault_manager_clear_bits(FAULT_CAN_TIMEOUT);
    assert(!fault_manager_any_fatal());

    fault_manager_set_bits(FAULT_CAN_BUS);
    assert(fault_manager_any_fatal());
    assert(fault_manager_get() == FAULT_CAN_BUS);
    fault_manager_clear_bits(FAULT_CAN_BUS);
    assert(!fault_manager_any_fatal());

    /* A non-CAN writer clearing its own bit must preserve an asynchronously
     * latched CAN fatal bit. Manager-owned fetch/critical updates make this
     * interleaving-safe instead of publishing a stale full-word snapshot. */
    fault_manager_set_bits(FAULT_CAN_BUS | FAULT_CAL_INVALID);
    fault_manager_clear_bits(FAULT_CAL_INVALID);
    assert(fault_manager_get() == FAULT_CAN_BUS);
    fault_manager_set_bits(FAULT_SENSOR);
    fault_manager_clear_bits(FAULT_SENSOR);
    assert(fault_manager_get() == FAULT_CAN_BUS);

    printf("test_fault_manager: 8 tests passed\n");
    return 0;
}
