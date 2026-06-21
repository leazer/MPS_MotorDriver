/* tests/fault_manager/test_fault_manager.c */
#include "fault_manager.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    fault_manager_init();
    assert(fault_manager_get() == FAULT_NONE);
    assert(!fault_manager_any());

    fault_manager_set(FAULT_OVERCURRENT);
    assert(fault_manager_get() == FAULT_OVERCURRENT);
    assert(fault_manager_any());

    fault_manager_set(FAULT_SENSOR);
    assert(fault_manager_get() == (FAULT_OVERCURRENT | FAULT_SENSOR));

    fault_manager_clear(FAULT_OVERCURRENT);
    assert(fault_manager_get() == FAULT_SENSOR);
    assert(fault_manager_any());

    fault_manager_clear_all();
    assert(fault_manager_get() == FAULT_NONE);
    assert(!fault_manager_any());

    printf("test_fault_manager: 5 tests passed\n");
    return 0;
}
