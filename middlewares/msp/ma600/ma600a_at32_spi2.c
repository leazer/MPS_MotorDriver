#include "ma600a_at32_spi2.h"

#include "at32m412_416_wk_config.h"
#include "rtthread.h"

#define MA600A_AT32_SPI2_TIMEOUT 100000u

static ma600a_status_t ma600a_at32_spi2_wait_flag(uint32_t flag)
{
    uint32_t timeout = MA600A_AT32_SPI2_TIMEOUT;

    while(spi_i2s_flag_get(SPI2, flag) == RESET)
    {
        if(timeout == 0u)
        {
            return MA600A_ERR_BUS;
        }
        timeout--;
    }

    return MA600A_OK;
}

ma600a_status_t ma600a_at32_spi2_transfer8(void *user, uint8_t tx, uint8_t *rx)
{
    ma600a_status_t status;

    (void)user;

    status = ma600a_at32_spi2_wait_flag(SPI_I2S_TDBE_FLAG);
    if(status != MA600A_OK)
    {
        return status;
    }

    spi_i2s_data_transmit(SPI2, tx);

    status = ma600a_at32_spi2_wait_flag(SPI_I2S_RDBF_FLAG);
    if(status != MA600A_OK)
    {
        return status;
    }

    if(rx != 0)
    {
        *rx = (uint8_t)(spi_i2s_data_receive(SPI2) & 0x00FFu);
    }
    else
    {
        (void)spi_i2s_data_receive(SPI2);
    }

    return MA600A_OK;
}

ma600a_status_t ma600a_at32_spi2_transfer16(void *user, uint16_t tx, uint16_t *rx)
{
    ma600a_status_t status;

    (void)user;

    /* SPI2 配置为 SPI_FRAME_16BIT, 硬件按 16 位移位收发.
     * 必须一次 spi_i2s_data_transmit 写 16-bit, 不能拆成两次 transfer8
     * (拆分会在 16-bit 帧模式下产生两个独立 16-bit 帧, 时序错乱, MA600A 返回 0).
     * 对齐 AT 官方 AS5047P SPI_ReadWriteByte 实现 (单次 16-bit 收发). */
    status = ma600a_at32_spi2_wait_flag(SPI_I2S_TDBE_FLAG);
    if(status != MA600A_OK)
    {
        return status;
    }

    spi_i2s_data_transmit(SPI2, tx);

    status = ma600a_at32_spi2_wait_flag(SPI_I2S_RDBF_FLAG);
    if(status != MA600A_OK)
    {
        return status;
    }

    if(rx != 0)
    {
        *rx = spi_i2s_data_receive(SPI2);
    }
    else
    {
        (void)spi_i2s_data_receive(SPI2);
    }

    return MA600A_OK;
}

void ma600a_at32_spi2_cs_write(void *user, uint8_t level)
{
    (void)user;

    if(level != 0u)
    {
        gpio_bits_set(SPI2_CS_GPIO_PORT, SPI2_CS_PIN);
    }
    else
    {
        gpio_bits_reset(SPI2_CS_GPIO_PORT, SPI2_CS_PIN);
    }
}

void ma600a_at32_spi2_delay_us(void *user, uint32_t us)
{
    volatile uint32_t cycles;

    (void)user;

    while(us > 0u)
    {
        cycles = 120u;
        while(cycles > 0u)
        {
            cycles--;
        }
        us--;
    }
}

void ma600a_at32_spi2_delay_ms(void *user, uint32_t ms)
{
    (void)user;

    rt_thread_mdelay(ms);
}

const ma600a_bus_t *ma600a_at32_spi2_bus_get(void)
{
    static const ma600a_bus_t bus = {
        0,
        ma600a_at32_spi2_transfer16,
        ma600a_at32_spi2_transfer8,
        ma600a_at32_spi2_cs_write,
        ma600a_at32_spi2_delay_us,
        ma600a_at32_spi2_delay_ms,
    };

    return &bus;
}
