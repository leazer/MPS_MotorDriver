#include "fault_manager.h"
#include <stdbool.h>

#if defined(__CC_ARM) || defined(__arm__) || defined(__thumb__)
#include "at32m412_416.h"

static volatile uint32_t s_fault_flags;

static uint32_t fault_manager_irq_lock(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void fault_manager_irq_unlock(uint32_t primask)
{
    __DMB();
    if (primask == 0u) {
        __enable_irq();
    }
}
#else
#include <stdatomic.h>

static atomic_uint s_fault_flags = ATOMIC_VAR_INIT(0u);
#endif

void fault_manager_init(void)
{
#if defined(__CC_ARM) || defined(__arm__) || defined(__thumb__)
    s_fault_flags = 0u;
#else
    atomic_store_explicit(&s_fault_flags, 0u, memory_order_seq_cst);
#endif
}

void fault_manager_set_bits(uint32_t bits)
{
#if defined(__CC_ARM) || defined(__arm__) || defined(__thumb__)
    uint32_t primask;

    primask = fault_manager_irq_lock();
    s_fault_flags |= bits;
    fault_manager_irq_unlock(primask);
#else
    (void)atomic_fetch_or_explicit(&s_fault_flags, bits,
                                   memory_order_seq_cst);
#endif
}

void fault_manager_clear_bits(uint32_t bits)
{
#if defined(__CC_ARM) || defined(__arm__) || defined(__thumb__)
    uint32_t primask;

    primask = fault_manager_irq_lock();
    s_fault_flags &= ~bits;
    fault_manager_irq_unlock(primask);
#else
    (void)atomic_fetch_and_explicit(&s_fault_flags, ~bits,
                                    memory_order_seq_cst);
#endif
}

void fault_manager_clear_all(void)
{
#if defined(__CC_ARM) || defined(__arm__) || defined(__thumb__)
    uint32_t primask;

    primask = fault_manager_irq_lock();
    s_fault_flags = 0u;
    fault_manager_irq_unlock(primask);
#else
    atomic_store_explicit(&s_fault_flags, 0u, memory_order_seq_cst);
#endif
}

uint32_t fault_manager_get(void)
{
#if defined(__CC_ARM) || defined(__arm__) || defined(__thumb__)
    return s_fault_flags;
#else
    return (uint32_t)atomic_load_explicit(&s_fault_flags,
                                          memory_order_seq_cst);
#endif
}

bool fault_manager_any(void) { return fault_manager_get() != 0u; }
bool fault_manager_any_fatal(void)
{
    return (fault_manager_get() & FAULT_FATAL_MASK) != 0u;
}
