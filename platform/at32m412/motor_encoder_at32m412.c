/*
 * motor_encoder_at32m412.c - MA600A 磁编码器 AT32M412 适配 (Stage 4)
 *
 * 职责:
 *   1. SPI2 硬件初始化 (Stage 0 清空了 wk_spi2_init, 此处重建)
 *   2. 封装 middlewares/msp/ma600 驱动, 提供角度+速度读取
 *   3. 机械角 -> 电角度换算 (含旁轴标定查表 + 零点修正, spec §4.5.2 + §4.7.6)
 *
 * SPI2 配置 (参考 AT 官方 AS5047P.c, 适配 MA600A):
 *   - Mode 1 (CPOL=0, CPHA=1), MSB first, 16-bit frame
 *   - 主机模式, 软件 CS (PA15)
 *   - DIV_64 = 180MHz/64 = 2.8125MHz (MA600A 上限 4.16MHz, 留余量)
 *   - 引脚 PB3(SCK)/PB4(MISO)/PB5(MOSI) MUX_3
 *
 * ISR 安全: ma600a_read_angle_and_speed_raw 只用 delay_us 忙等 (~1µs),
 * 不触发 RT-Thread API, 可在 FOC ISR (16kHz) 内直接调用.
 */
#include "motor_encoder_at32m412.h"
#include "board_motor_pins.h"
#include "ma600a.h"
#include "ma600a_at32_spi2.h"
#include "motor_calibration.h"
#include "motor_params.h"
#include "at32m412_416.h"
#include <math.h>

/* 编码器存活判定窗口: 最近 N 次读取中至少 1 次成功 */
#define ENC_ALIVE_WINDOW        32u
#define TWO_PI_F                6.28318530718f

/* ===== 内部状态 ===== */
static ma600a_t s_ma600a;
static uint16_t  s_last_raw16 = 0u;       /* 最近一次成功读取的 16-bit 角度 */
static uint16_t  s_error_count = 0u;      /* 读取失败累计 (uint16 环绕) */
static uint32_t  s_last_success_tick = 0u;/* 最近成功读取的 tick 序号 (用静态递增计数) */
static uint32_t  s_tick_seq = 0u;         /* 内部 tick 序号 (每次 read_angle_speed 递增) */

/* 零点: 由标定或手动 mc_zero 设置, 定义在 motor_calibration.c */
extern uint16_t g_motor_zero_raw;

/* ===== SPI2 硬件初始化 (wk_spi2_init 等价, 适配 PB3/4/5 MUX_3) ===== */
static void motor_encoder_spi2_init(void)
{
    gpio_init_type gpio_init_struct;
    spi_init_type  spi_init_struct;

    /* 1. CRM 时钟: SPI2 + GPIOB(SCK/MISO/MOSI). GPIOA(CS) 已在 board_clock_init 开 */
    crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    /* 2. GPIO MUX: PB3=SCK, PB4=MISO, PB5=MOSI (AF3) */
    gpio_pin_mux_config(SPI2_SCK_GPIO_PORT,  SPI2_SCK_PIN_SOURCE,  SPI2_SCK_IOMUX);
    gpio_pin_mux_config(SPI2_MISO_GPIO_PORT, SPI2_MISO_PIN_SOURCE, SPI2_MISO_IOMUX);
    gpio_pin_mux_config(SPI2_MOSI_GPIO_PORT, SPI2_MOSI_PIN_SOURCE, SPI2_MOSI_IOMUX);

    /* 3. GPIO 配置: SCK/MOSI/MISO 复用推挽, 下拉 (参考 AS5047P.c) */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode           = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull           = GPIO_PULL_DOWN;
    gpio_init_struct.gpio_pins           = SPI2_SCK_PIN | SPI2_MISO_PIN | SPI2_MOSI_PIN;
    gpio_init(GPIOB, &gpio_init_struct);

    /* CS (PA15): board_gpio_init 已配为推挽输出并置高. 此处幂等再设一次确保状态 */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode           = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pull           = GPIO_PULL_NONE;
    gpio_init_struct.gpio_pins           = SPI2_CS_PIN;
    gpio_init(SPI2_CS_GPIO_PORT, &gpio_init_struct);
    gpio_bits_set(SPI2_CS_GPIO_PORT, SPI2_CS_PIN);   /* CS=高, 释放 */

    /* 4. SPI2 参数 (Mode 0, MSB, 16-bit, 主机, 软件 CS, DIV_64) */
    spi_i2s_reset(SPI2);
    spi_default_para_init(&spi_init_struct);
    spi_init_struct.transmission_mode      = SPI_TRANSMIT_FULL_DUPLEX;
    spi_init_struct.master_slave_mode      = SPI_MODE_MASTER;
    spi_init_struct.mclk_freq_division     = SPI_MCLK_DIV_64;   /* 2.8125MHz */
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi_init_struct.frame_bit_num          = SPI_FRAME_16BIT;
    spi_init_struct.clock_polarity         = SPI_CLOCK_POLARITY_LOW;   /* CPOL=0 */
    spi_init_struct.clock_phase            = SPI_CLOCK_PHASE_1EDGE;    /* CPHA=0 -> Mode 0 */
    spi_init_struct.cs_mode_selection      = SPI_CS_SOFTWARE_MODE;
    spi_init(SPI2, &spi_init_struct);

    spi_enable(SPI2, TRUE);
}

void motor_encoder_at32m412_init(void)
{
    motor_encoder_spi2_init();
    ma600a_init(&s_ma600a, ma600a_at32_spi2_bus_get());
}

int motor_encoder_read_raw_frame(uint16_t *raw_angle_16, int16_t *raw_speed)
{
    if ((raw_angle_16 == 0) || (raw_speed == 0)) {
        s_error_count++;
        return -1;
    }

    if (ma600a_read_angle_and_speed_raw(&s_ma600a, raw_angle_16, raw_speed) != 0) {
        s_error_count++;
        return -1;
    }

    return 0;
}

int motor_encoder_read_angle_speed(uint16_t *raw_angle_16, int16_t *raw_speed)
{
    s_tick_seq++;

    if (motor_encoder_read_raw_frame(raw_angle_16, raw_speed) != 0) {
        return -1;
    }

    s_last_raw16 = *raw_angle_16;
    s_last_success_tick = s_tick_seq;
    return 0;
}

int motor_encoder_read_angle_raw(uint16_t *raw_angle_16)
{
    s_tick_seq++;

    if (raw_angle_16 == 0) {
        s_error_count++;
        return -1;
    }

    if (ma600a_read_angle_raw(&s_ma600a, raw_angle_16) != 0) {
        s_error_count++;
        return -1;
    }

    s_last_raw16 = *raw_angle_16;
    s_last_success_tick = s_tick_seq;
    return 0;
}

float motor_encoder_to_electrical_angle(uint16_t raw_angle_16)
{
    uint16_t raw_corrected;
    uint16_t mech_diff;
    float    theta_e;

    /* 1. 旁轴查表校正 (spec §4.7.6), 若标定有效 */
    if (motor_calibration_is_valid()) {
        const motor_calibration_t *cal = motor_calibration_get();
        /* 256 点线性插值 (Q16 定点, spec §4.7.6) */
        uint32_t idx_frac_q24 = (uint32_t)raw_angle_16 * 256u;
        uint16_t idx  = (uint16_t)(idx_frac_q24 >> 16);          /* 0..255 */
        uint16_t frac = (uint16_t)(idx_frac_q24 & 0xFFFFu);      /* Q16 小数 */
        int16_t  off0 = cal->table[idx];
        int16_t  off1 = cal->table[(uint16_t)((idx + 1u) & 0xFFu)];
        int32_t  off_mdeg = (int32_t)off0 + (((int32_t)(off1 - off0) * (int32_t)frac) >> 16);
        /* 0.001° -> raw16 LSB: 65536 / 360000 = 0.1820444... */
        int32_t  off_raw = (off_mdeg * 65536) / 360000;
        raw_corrected = (uint16_t)((int32_t)raw_angle_16 - off_raw);
    } else {
        raw_corrected = raw_angle_16;
    }

    /* 2. 减零点 (16-bit 环绕) */
    mech_diff = (uint16_t)(raw_corrected - g_motor_zero_raw);

    /* 3. 机械角 -> 电角度 (spec §4.5.2) */
    theta_e = ((float)mech_diff * (float)MOTOR_POLE_PAIRS * TWO_PI_F) / 65536.0f;
    /* 对 2π 取模, 归一化到 [0, 2π) */
    theta_e = fmodf(theta_e, TWO_PI_F);
    if (theta_e < 0.0f) {
        theta_e += TWO_PI_F;
    }
    return theta_e;
}

uint16_t motor_encoder_get_last_raw(void) { return s_last_raw16; }
uint16_t motor_encoder_get_error_count(void) { return s_error_count; }

bool motor_encoder_is_alive(void)
{
    /* 最近 ENC_ALIVE_WINDOW 次 tick 内有成功读取即视为存活 */
    return (s_tick_seq - s_last_success_tick) < ENC_ALIVE_WINDOW;
}

void motor_encoder_set_zero(uint16_t raw) { g_motor_zero_raw = raw; }
uint16_t motor_encoder_get_zero(void) { return g_motor_zero_raw; }

