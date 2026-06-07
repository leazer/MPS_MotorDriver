#include "ma600a_debug.h"

#include "ma600a.h"
#include "ma600a_at32_spi2.h"

volatile uint16_t g_ma600a_raw_angle = 0u;
volatile float g_ma600a_angle_deg = 0.0f;
volatile int32_t g_ma600a_status = MA600A_ERR_NULL;
volatile uint32_t g_ma600a_sample_count = 0u;
volatile uint32_t g_ma600a_error_count = 0u;
volatile uint8_t g_ma600a_bct = 0u;
volatile uint8_t g_ma600a_axis = 0u;
volatile int16_t g_ma600a_speed_raw = 0;
volatile float g_ma600a_speed_rpm = 0.0f;

static ma600a_t s_ma600a;

void ma600a_debug_init(void)
{
    ma600a_status_t status;
    uint8_t bct = 0u;
    ma600a_bct_axis_t axis = MA600A_BCT_AXIS_NONE;

    status = ma600a_init(&s_ma600a, ma600a_at32_spi2_bus_get());
    g_ma600a_status = status;
    if(status != MA600A_OK)
    {
        g_ma600a_error_count++;
        return;
    }

    status = ma600a_read_bct(&s_ma600a, &bct, &axis);
    g_ma600a_status = status;
    if(status == MA600A_OK)
    {
        g_ma600a_bct = bct;
        g_ma600a_axis = (uint8_t)axis;
    }
    else
    {
        g_ma600a_error_count++;
    }

    status = ma600a_set_mtsp_speed(&s_ma600a, 1u);
    g_ma600a_status = status;
    if(status != MA600A_OK)
    {
        g_ma600a_error_count++;
        return;
    }

    ma600a_debug_poll();
}

void ma600a_debug_poll(void)
{
    ma600a_status_t status;
    uint16_t raw_angle = 0u;
    int16_t speed = 0;

    status = ma600a_read_angle_and_speed_raw(&s_ma600a, &raw_angle, &speed);
    g_ma600a_status = status;
    if(status == MA600A_OK)
    {
        g_ma600a_raw_angle = raw_angle;
        g_ma600a_angle_deg = ((float)raw_angle * 360.0f) / 65536.0f;
        g_ma600a_speed_raw = speed;
        g_ma600a_speed_rpm = ma600a_speed_raw_to_rpm(speed);
        g_ma600a_sample_count++;
    }
    else
    {
        g_ma600a_error_count++;
    }
}
