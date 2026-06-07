#ifndef MA600A_H
#define MA600A_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MA600A_CORRECTION_TABLE_SIZE 32u

typedef enum
{
    MA600A_OK = 0,
    MA600A_ERR_NULL = -1,
    MA600A_ERR_BUS = -2,
    MA600A_ERR_RANGE = -3,
    MA600A_ERR_VERIFY = -4
} ma600a_status_t;

typedef enum
{
    MA600A_BCT_AXIS_NONE = 0,
    MA600A_BCT_AXIS_X,
    MA600A_BCT_AXIS_Y,
    MA600A_BCT_AXIS_XY
} ma600a_bct_axis_t;

typedef struct ma600a_bus
{
    void *user;
    ma600a_status_t (*spi_transfer16)(void *user, uint16_t tx, uint16_t *rx);
    ma600a_status_t (*spi_transfer8)(void *user, uint8_t tx, uint8_t *rx);
    void (*cs_write)(void *user, uint8_t level);
    void (*delay_us)(void *user, uint32_t us);
    void (*delay_ms)(void *user, uint32_t ms);
} ma600a_bus_t;

typedef struct
{
    ma600a_bus_t bus;
} ma600a_t;

ma600a_status_t ma600a_init(ma600a_t *dev, const ma600a_bus_t *bus);

ma600a_status_t ma600a_read_angle_raw(ma600a_t *dev, uint16_t *angle);
ma600a_status_t ma600a_read_angle_deg(ma600a_t *dev, float *deg);
ma600a_status_t ma600a_read_angle_and_speed_raw(ma600a_t *dev, uint16_t *angle, int16_t *speed);
ma600a_status_t ma600a_read_speed_raw(ma600a_t *dev, int16_t *speed);
ma600a_status_t ma600a_set_mtsp_speed(ma600a_t *dev, uint8_t enable);

ma600a_status_t ma600a_write_bct(ma600a_t *dev, uint8_t bct, ma600a_bct_axis_t axis);
ma600a_status_t ma600a_read_bct(ma600a_t *dev, uint8_t *bct, ma600a_bct_axis_t *axis);

ma600a_status_t ma600a_write_zero(ma600a_t *dev, uint16_t zero);
ma600a_status_t ma600a_read_zero(ma600a_t *dev, uint16_t *zero);
ma600a_status_t ma600a_set_current_angle_as_zero(ma600a_t *dev, uint16_t *zero);

ma600a_status_t ma600a_write_correction_table(ma600a_t *dev, const int8_t corr[MA600A_CORRECTION_TABLE_SIZE]);
ma600a_status_t ma600a_read_correction_table(ma600a_t *dev, int8_t corr[MA600A_CORRECTION_TABLE_SIZE]);
ma600a_status_t ma600a_clear_correction_table(ma600a_t *dev);

ma600a_status_t ma600a_store_config_to_nvm(ma600a_t *dev);
ma600a_status_t ma600a_store_correction_to_nvm(ma600a_t *dev);
ma600a_status_t ma600a_restore_from_nvm(ma600a_t *dev);

int8_t ma600a_corr_deg_to_reg(float corr_deg);
uint8_t ma600a_calc_bct_from_k(float k);
float ma600a_speed_raw_to_rpm(int16_t speed);
float ma600a_speed_raw_to_rpm_with_ck100(int16_t speed, float f_ck100_khz);

#ifdef __cplusplus
}
#endif

#endif
