#include "ma600a.h"

#define MA600A_CS_LOW  0u
#define MA600A_CS_HIGH 1u

#define MA600A_READ_REG_COMMAND        0xD200u
#define MA600A_WRITE_REG_COMMAND       0xEA54u
#define MA600A_STORE_BLOCK_COMMAND_1   0xEA55u
#define MA600A_STORE_BLOCK_COMMAND_2   0xEA00u
#define MA600A_RESTORE_BLOCKS_COMMAND  0xEA56u

#define MA600A_REG_ZERO0 0x00u
#define MA600A_REG_ZERO1 0x01u
#define MA600A_REG_BCT0  0x02u
#define MA600A_REG_BCT1  0x03u
#define MA600A_REG_PRT   0x1Cu
#define MA600A_REG_CORR0 0x20u

#define MA600A_BCT1_ETX 0x01u
#define MA600A_BCT1_ETY 0x02u
#define MA600A_PRT_MTSP 0x80u

static ma600a_status_t ma600a_validate(const ma600a_t *dev)
{
    if(dev == 0)
    {
        return MA600A_ERR_NULL;
    }

    if((dev->bus.spi_transfer16 == 0) || (dev->bus.cs_write == 0))
    {
        return MA600A_ERR_NULL;
    }

    return MA600A_OK;
}

static void ma600a_delay_us(ma600a_t *dev, uint32_t us)
{
    if(dev->bus.delay_us != 0)
    {
        dev->bus.delay_us(dev->bus.user, us);
    }
}

static void ma600a_delay_ms(ma600a_t *dev, uint32_t ms)
{
    if(dev->bus.delay_ms != 0)
    {
        dev->bus.delay_ms(dev->bus.user, ms);
    }
}

static ma600a_status_t ma600a_transfer16(ma600a_t *dev, uint16_t tx, uint16_t *rx)
{
    ma600a_status_t status;

    dev->bus.cs_write(dev->bus.user, MA600A_CS_LOW);
    status = dev->bus.spi_transfer16(dev->bus.user, tx, rx);
    dev->bus.cs_write(dev->bus.user, MA600A_CS_HIGH);

    if(status != MA600A_OK)
    {
        return MA600A_ERR_BUS;
    }

    return MA600A_OK;
}

static ma600a_status_t ma600a_transfer32_zero(ma600a_t *dev, uint16_t *angle, uint16_t *data)
{
    ma600a_status_t status;

    dev->bus.cs_write(dev->bus.user, MA600A_CS_LOW);

    status = dev->bus.spi_transfer16(dev->bus.user, 0x0000u, angle);
    if(status == MA600A_OK)
    {
        status = dev->bus.spi_transfer16(dev->bus.user, 0x0000u, data);
    }

    dev->bus.cs_write(dev->bus.user, MA600A_CS_HIGH);

    if(status != MA600A_OK)
    {
        return MA600A_ERR_BUS;
    }

    return MA600A_OK;
}

static ma600a_status_t ma600a_read_register(ma600a_t *dev, uint8_t address, uint8_t *value)
{
    ma600a_status_t status;
    uint16_t rx = 0u;

    if(value == 0)
    {
        return MA600A_ERR_NULL;
    }

    status = ma600a_transfer16(dev, (uint16_t)(MA600A_READ_REG_COMMAND | address), &rx);
    if(status != MA600A_OK)
    {
        return status;
    }

    ma600a_delay_us(dev, 1u);

    status = ma600a_transfer16(dev, 0x0000u, &rx);
    if(status != MA600A_OK)
    {
        return status;
    }

    ma600a_delay_us(dev, 1u);
    *value = (uint8_t)(rx & 0x00FFu);
    return MA600A_OK;
}

static ma600a_status_t ma600a_write_register(ma600a_t *dev, uint8_t address, uint8_t value)
{
    ma600a_status_t status;
    uint16_t rx = 0u;

    status = ma600a_transfer16(dev, MA600A_WRITE_REG_COMMAND, &rx);
    if(status != MA600A_OK)
    {
        return status;
    }

    ma600a_delay_us(dev, 1u);

    status = ma600a_transfer16(dev, (uint16_t)(((uint16_t)address << 8) | value), &rx);
    if(status != MA600A_OK)
    {
        return status;
    }

    ma600a_delay_us(dev, 1u);

    status = ma600a_transfer16(dev, 0x0000u, &rx);
    if(status != MA600A_OK)
    {
        return status;
    }

    ma600a_delay_us(dev, 1u);
    if((uint8_t)(rx & 0x00FFu) != value)
    {
        return MA600A_ERR_VERIFY;
    }

    return MA600A_OK;
}

static ma600a_status_t ma600a_update_register_bits(ma600a_t *dev, uint8_t address, uint8_t mask, uint8_t value)
{
    ma600a_status_t status;
    uint8_t reg = 0u;

    status = ma600a_read_register(dev, address, &reg);
    if(status != MA600A_OK)
    {
        return status;
    }

    reg = (uint8_t)((reg & (uint8_t)~mask) | (value & mask));
    return ma600a_write_register(dev, address, reg);
}

static uint8_t ma600a_axis_to_bits(ma600a_bct_axis_t axis)
{
    switch(axis)
    {
        case MA600A_BCT_AXIS_X:
            return MA600A_BCT1_ETX;
        case MA600A_BCT_AXIS_Y:
            return MA600A_BCT1_ETY;
        case MA600A_BCT_AXIS_XY:
            return (uint8_t)(MA600A_BCT1_ETX | MA600A_BCT1_ETY);
        case MA600A_BCT_AXIS_NONE:
        default:
            return 0u;
    }
}

static ma600a_bct_axis_t ma600a_bits_to_axis(uint8_t bits)
{
    bits &= (uint8_t)(MA600A_BCT1_ETX | MA600A_BCT1_ETY);

    if(bits == MA600A_BCT1_ETX)
    {
        return MA600A_BCT_AXIS_X;
    }

    if(bits == MA600A_BCT1_ETY)
    {
        return MA600A_BCT_AXIS_Y;
    }

    if(bits == (uint8_t)(MA600A_BCT1_ETX | MA600A_BCT1_ETY))
    {
        return MA600A_BCT_AXIS_XY;
    }

    return MA600A_BCT_AXIS_NONE;
}

ma600a_status_t ma600a_init(ma600a_t *dev, const ma600a_bus_t *bus)
{
    if((dev == 0) || (bus == 0))
    {
        return MA600A_ERR_NULL;
    }

    if((bus->spi_transfer16 == 0) || (bus->cs_write == 0))
    {
        return MA600A_ERR_NULL;
    }

    dev->bus = *bus;
    dev->bus.cs_write(dev->bus.user, MA600A_CS_HIGH);
    return MA600A_OK;
}

ma600a_status_t ma600a_read_angle_raw(ma600a_t *dev, uint16_t *angle)
{
    ma600a_status_t status;

    if(angle == 0)
    {
        return MA600A_ERR_NULL;
    }

    status = ma600a_validate(dev);
    if(status != MA600A_OK)
    {
        return status;
    }

    return ma600a_transfer16(dev, 0x0000u, angle);
}

ma600a_status_t ma600a_read_angle_deg(ma600a_t *dev, float *deg)
{
    ma600a_status_t status;
    uint16_t angle = 0u;

    if(deg == 0)
    {
        return MA600A_ERR_NULL;
    }

    status = ma600a_read_angle_raw(dev, &angle);
    if(status != MA600A_OK)
    {
        return status;
    }

    *deg = ((float)angle * 360.0f) / 65536.0f;
    return MA600A_OK;
}

ma600a_status_t ma600a_read_angle_and_speed_raw(ma600a_t *dev, uint16_t *angle, int16_t *speed)
{
    ma600a_status_t status;
    uint16_t speed_word = 0u;

    if((angle == 0) || (speed == 0))
    {
        return MA600A_ERR_NULL;
    }

    status = ma600a_validate(dev);
    if(status != MA600A_OK)
    {
        return status;
    }

    status = ma600a_transfer32_zero(dev, angle, &speed_word);
    if(status != MA600A_OK)
    {
        return status;
    }

    *speed = (int16_t)speed_word;
    return MA600A_OK;
}

ma600a_status_t ma600a_read_speed_raw(ma600a_t *dev, int16_t *speed)
{
    uint16_t angle = 0u;

    return ma600a_read_angle_and_speed_raw(dev, &angle, speed);
}

ma600a_status_t ma600a_set_mtsp_speed(ma600a_t *dev, uint8_t enable)
{
    ma600a_status_t status;
    uint8_t value = 0u;

    status = ma600a_validate(dev);
    if(status != MA600A_OK)
    {
        return status;
    }

    if(enable != 0u)
    {
        value = MA600A_PRT_MTSP;
    }

    return ma600a_update_register_bits(dev, MA600A_REG_PRT, MA600A_PRT_MTSP, value);
}

ma600a_status_t ma600a_write_bct(ma600a_t *dev, uint8_t bct, ma600a_bct_axis_t axis)
{
    ma600a_status_t status;
    uint8_t readback = 0u;
    uint8_t axis_bits;

    status = ma600a_validate(dev);
    if(status != MA600A_OK)
    {
        return status;
    }

    axis_bits = ma600a_axis_to_bits(axis);
    if((axis > MA600A_BCT_AXIS_XY) || ((bct != 0u) && (axis == MA600A_BCT_AXIS_NONE)))
    {
        return MA600A_ERR_RANGE;
    }

    status = ma600a_write_register(dev, MA600A_REG_BCT0, bct);
    if(status != MA600A_OK)
    {
        return status;
    }

    status = ma600a_write_register(dev, MA600A_REG_BCT1, axis_bits);
    if(status != MA600A_OK)
    {
        return status;
    }

    status = ma600a_read_register(dev, MA600A_REG_BCT1, &readback);
    if(status != MA600A_OK)
    {
        return status;
    }

    if((readback & (uint8_t)(MA600A_BCT1_ETX | MA600A_BCT1_ETY)) != axis_bits)
    {
        return MA600A_ERR_VERIFY;
    }

    return MA600A_OK;
}

ma600a_status_t ma600a_read_bct(ma600a_t *dev, uint8_t *bct, ma600a_bct_axis_t *axis)
{
    ma600a_status_t status;
    uint8_t axis_bits = 0u;

    if((bct == 0) || (axis == 0))
    {
        return MA600A_ERR_NULL;
    }

    status = ma600a_validate(dev);
    if(status != MA600A_OK)
    {
        return status;
    }

    status = ma600a_read_register(dev, MA600A_REG_BCT0, bct);
    if(status != MA600A_OK)
    {
        return status;
    }

    status = ma600a_read_register(dev, MA600A_REG_BCT1, &axis_bits);
    if(status != MA600A_OK)
    {
        return status;
    }

    *axis = ma600a_bits_to_axis(axis_bits);
    return MA600A_OK;
}

ma600a_status_t ma600a_write_zero(ma600a_t *dev, uint16_t zero)
{
    ma600a_status_t status;
    uint16_t readback = 0u;

    status = ma600a_validate(dev);
    if(status != MA600A_OK)
    {
        return status;
    }

    status = ma600a_write_register(dev, MA600A_REG_ZERO0, (uint8_t)(zero & 0x00FFu));
    if(status != MA600A_OK)
    {
        return status;
    }

    status = ma600a_write_register(dev, MA600A_REG_ZERO1, (uint8_t)(zero >> 8));
    if(status != MA600A_OK)
    {
        return status;
    }

    status = ma600a_read_zero(dev, &readback);
    if(status != MA600A_OK)
    {
        return status;
    }

    if(readback != zero)
    {
        return MA600A_ERR_VERIFY;
    }

    return MA600A_OK;
}

ma600a_status_t ma600a_read_zero(ma600a_t *dev, uint16_t *zero)
{
    ma600a_status_t status;
    uint8_t low = 0u;
    uint8_t high = 0u;

    if(zero == 0)
    {
        return MA600A_ERR_NULL;
    }

    status = ma600a_validate(dev);
    if(status != MA600A_OK)
    {
        return status;
    }

    status = ma600a_read_register(dev, MA600A_REG_ZERO0, &low);
    if(status != MA600A_OK)
    {
        return status;
    }

    status = ma600a_read_register(dev, MA600A_REG_ZERO1, &high);
    if(status != MA600A_OK)
    {
        return status;
    }

    *zero = (uint16_t)(((uint16_t)high << 8) | low);
    return MA600A_OK;
}

ma600a_status_t ma600a_set_current_angle_as_zero(ma600a_t *dev, uint16_t *zero)
{
    ma600a_status_t status;
    uint16_t angle = 0u;

    status = ma600a_write_zero(dev, 0u);
    if(status != MA600A_OK)
    {
        return status;
    }

    status = ma600a_read_angle_raw(dev, &angle);
    if(status != MA600A_OK)
    {
        return status;
    }

    status = ma600a_write_zero(dev, angle);
    if(status != MA600A_OK)
    {
        return status;
    }

    if(zero != 0)
    {
        *zero = angle;
    }

    return MA600A_OK;
}

ma600a_status_t ma600a_write_correction_table(ma600a_t *dev, const int8_t corr[MA600A_CORRECTION_TABLE_SIZE])
{
    ma600a_status_t status;
    uint8_t i;

    if(corr == 0)
    {
        return MA600A_ERR_NULL;
    }

    status = ma600a_validate(dev);
    if(status != MA600A_OK)
    {
        return status;
    }

    for(i = 0u; i < MA600A_CORRECTION_TABLE_SIZE; i++)
    {
        status = ma600a_write_register(dev, (uint8_t)(MA600A_REG_CORR0 + i), (uint8_t)corr[i]);
        if(status != MA600A_OK)
        {
            return status;
        }
    }

    return MA600A_OK;
}

ma600a_status_t ma600a_read_correction_table(ma600a_t *dev, int8_t corr[MA600A_CORRECTION_TABLE_SIZE])
{
    ma600a_status_t status;
    uint8_t i;
    uint8_t value = 0u;

    if(corr == 0)
    {
        return MA600A_ERR_NULL;
    }

    status = ma600a_validate(dev);
    if(status != MA600A_OK)
    {
        return status;
    }

    for(i = 0u; i < MA600A_CORRECTION_TABLE_SIZE; i++)
    {
        status = ma600a_read_register(dev, (uint8_t)(MA600A_REG_CORR0 + i), &value);
        if(status != MA600A_OK)
        {
            return status;
        }
        corr[i] = (int8_t)value;
    }

    return MA600A_OK;
}

ma600a_status_t ma600a_clear_correction_table(ma600a_t *dev)
{
    int8_t corr[MA600A_CORRECTION_TABLE_SIZE] = {0};

    return ma600a_write_correction_table(dev, corr);
}

ma600a_status_t ma600a_store_config_to_nvm(ma600a_t *dev)
{
    ma600a_status_t status;
    uint16_t rx = 0u;

    status = ma600a_validate(dev);
    if(status != MA600A_OK)
    {
        return status;
    }

    status = ma600a_transfer16(dev, MA600A_STORE_BLOCK_COMMAND_1, &rx);
    if(status != MA600A_OK)
    {
        return status;
    }
   
    ma600a_delay_us(dev, 1u);

    status = ma600a_transfer16(dev, MA600A_STORE_BLOCK_COMMAND_2, &rx);
    if(status != MA600A_OK)
    {
        return status;
    }

    ma600a_delay_ms(dev, 600u);
    return MA600A_OK;
}

ma600a_status_t ma600a_store_correction_to_nvm(ma600a_t *dev)
{
    ma600a_status_t status;
    uint16_t rx = 0u;

    status = ma600a_validate(dev);
    if(status != MA600A_OK)
    {
        return status;
    }

    status = ma600a_transfer16(dev, MA600A_STORE_BLOCK_COMMAND_1, &rx);
    if(status != MA600A_OK)
    {
        return status;
    }

    ma600a_delay_us(dev, 1u);

    status = ma600a_transfer16(dev, (uint16_t)(MA600A_STORE_BLOCK_COMMAND_2 | 0x0001u), &rx);
    if(status != MA600A_OK)
    {
        return status;
    }

    ma600a_delay_ms(dev, 600u);
    return MA600A_OK;
}

ma600a_status_t ma600a_restore_from_nvm(ma600a_t *dev)
{
    ma600a_status_t status;
    uint16_t rx = 0u;

    status = ma600a_validate(dev);
    if(status != MA600A_OK)
    {
        return status;
    }

    status = ma600a_transfer16(dev, MA600A_RESTORE_BLOCKS_COMMAND, &rx);
    if(status != MA600A_OK)
    {
        return status;
    }

    ma600a_delay_us(dev, 240u);
    return MA600A_OK;
}

int8_t ma600a_corr_deg_to_reg(float corr_deg)
{
    float scaled;
    int32_t value;

    if(corr_deg > 11.25f)
    {
        corr_deg = 11.25f;
    }
    else if(corr_deg < -11.25f)
    {
        corr_deg = -11.25f;
    }

    scaled = (corr_deg * 32.0f * 128.0f) / 360.0f;
    if(scaled >= 0.0f)
    {
        value = (int32_t)(scaled + 0.5f);
    }
    else
    {
        value = (int32_t)(scaled - 0.5f);
    }

    if(value > 127)
    {
        value = 127;
    }
    else if(value < -128)
    {
        value = -128;
    }

    return (int8_t)value;
}

uint8_t ma600a_calc_bct_from_k(float k)
{
    float bct;

    if(k <= 1.0f)
    {
        return 0u;
    }

    bct = 258.0f * (1.0f - (1.0f / k));
    if(bct >= 255.0f)
    {
        return 255u;
    }

    return (uint8_t)(bct + 0.5f);
}

float ma600a_speed_raw_to_rpm(int16_t speed)
{
    return (float)speed * 5.722f;
}

float ma600a_speed_raw_to_rpm_with_ck100(int16_t speed, float f_ck100_khz)
{
    return (float)speed * 5.722f * f_ck100_khz / 100.0f;
}
