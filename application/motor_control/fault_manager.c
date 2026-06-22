#include "fault_manager.h"
#include <stdbool.h>

static uint32_t s_fault_flags = 0u;

void fault_manager_init(void) { s_fault_flags = 0u; }
void fault_manager_set(uint32_t fault) { s_fault_flags |= fault; }
void fault_manager_clear(uint32_t fault) { s_fault_flags &= ~fault; }
void fault_manager_clear_all(void) { s_fault_flags = 0u; }
uint32_t fault_manager_get(void) { return s_fault_flags; }
bool fault_manager_any(void) { return s_fault_flags != 0u; }
bool fault_manager_any_fatal(void) { return (s_fault_flags & FAULT_FATAL_MASK) != 0u; }
