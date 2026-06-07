#ifndef MA600A_AT32_SPI2_H
#define MA600A_AT32_SPI2_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ma600a.h"

const ma600a_bus_t *ma600a_at32_spi2_bus_get(void);

ma600a_status_t ma600a_at32_spi2_transfer16(void *user, uint16_t tx, uint16_t *rx);
ma600a_status_t ma600a_at32_spi2_transfer8(void *user, uint8_t tx, uint8_t *rx);
void ma600a_at32_spi2_cs_write(void *user, uint8_t level);
void ma600a_at32_spi2_delay_us(void *user, uint32_t us);
void ma600a_at32_spi2_delay_ms(void *user, uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif
