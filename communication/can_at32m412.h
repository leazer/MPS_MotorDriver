#ifndef CAN_AT32M412_H
#define CAN_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

/* Counters saturate at UINT32_MAX. Latched fields clear only on init. */
typedef struct {
    uint8_t rec;
    uint8_t tec;
    bool error_passive;
    bool bus_off_latched;
    bool fatal_latched;
    uint32_t rx_received;
    uint32_t rx_rejected;
    uint32_t rx_overflow;
    uint32_t tx_queued;
    uint32_t tx_completed;
    uint32_t tx_rejected;
    uint32_t tx_errors;
    uint32_t status_irqs;
    uint32_t error_irqs;
} can_at32m412_diag_t;

bool can_at32m412_init(uint8_t node_id);
bool can_at32m412_rx_pop(can_frame_t *out);
bool can_at32m412_tx_push(const can_frame_t *frame);
void can_at32m412_tx_kick(void);
void can_at32m412_get_diag(can_at32m412_diag_t *out);
bool can_at32m412_fatal_bus_error(void);

void can_at32m412_irq_rx(void);
void can_at32m412_irq_tx(void);
void can_at32m412_irq_status(void);
void can_at32m412_irq_error(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_AT32M412_H */
