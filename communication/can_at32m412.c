#include "can_at32m412.h"

void can_at32m412_init(void) { /* stub, Plan 5 */ }
bool can_at32m412_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    (void)id; (void)data; (void)len;
    return false;
}
void can_at32m412_register_rx(can_rx_callback_t cb) { (void)cb; }
