#include "can_protocol.h"
#include "motor_params.h"
#include <string.h>

void can_protocol_handle_control(const uint8_t *data, uint8_t len)
{
    (void)data; (void)len;  /* stub, Plan 5 */
}

void can_protocol_build_status(uint8_t *data, uint8_t *len)
{
    memset(data, 0, 8);
    *len = 8;  /* stub, Plan 5 */
}

void can_protocol_build_ext_status(uint8_t *data, uint8_t *len)
{
    memset(data, 0, 8);
    *len = 8;  /* stub, Plan 5 */
}
