#ifndef BOARD_MOTOR_PINS_H
#define BOARD_MOTOR_PINS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "at32m412_416.h"

/* ===== TMR1 三相 PWM (高边输入到 MP6540H) ===== */
#define PWMA_GPIO_PORT           GPIOA
#define PWMA_PIN                 GPIO_PINS_8
#define PWMA_PIN_SOURCE          GPIO_PINS_SOURCE8
#define PWMA_IOMUX               GPIO_MUX_1

#define PWMB_GPIO_PORT           GPIOA
#define PWMB_PIN                 GPIO_PINS_9
#define PWMB_PIN_SOURCE          GPIO_PINS_SOURCE9
#define PWMB_IOMUX               GPIO_MUX_1

#define PWMC_GPIO_PORT           GPIOA
#define PWMC_PIN                 GPIO_PINS_10
#define PWMC_PIN_SOURCE          GPIO_PINS_SOURCE10
#define PWMC_IOMUX               GPIO_MUX_1

/* ===== MP6540H 使能 / 故障 ===== */
#define PWM_EN_GPIO_PORT         GPIOB
#define PWM_EN_PIN               GPIO_PINS_10

#define nFAULT_GPIO_PORT         GPIOB
#define nFAULT_PIN               GPIO_PINS_2
#define nFAULT_EXINT_LINE        EXINT_LINE_2
#define nFAULT_EXINT_IRQn        EXINT2_IRQn

/* ===== ADC2 电流采样 (MP6540H 电流镜) ===== */
#define SOA_GPIO_PORT            GPIOB
#define SOA_PIN                  GPIO_PINS_1
#define SOA_ADC_CHANNEL          ADC_CHANNEL_9

#define SOB_GPIO_PORT            GPIOB
#define SOB_PIN                  GPIO_PINS_0
#define SOB_ADC_CHANNEL          ADC_CHANNEL_8

#define SOC_GPIO_PORT            GPIOA
#define SOC_PIN                  GPIO_PINS_7
#define SOC_ADC_CHANNEL          ADC_CHANNEL_7

/* ===== VBUS 母线电压 (分压比 1/6) ===== */
#define VBUS_GPIO_PORT           GPIOA
#define VBUS_PIN                 GPIO_PINS_6
#define VBUS_ADC_CHANNEL         ADC_CHANNEL_6

/* ===== SPI2 (MA600A) ===== */
#define SPI2_SCK_GPIO_PORT       GPIOB
#define SPI2_SCK_PIN             GPIO_PINS_3
#define SPI2_SCK_PIN_SOURCE      GPIO_PINS_SOURCE3
#define SPI2_SCK_IOMUX           GPIO_MUX_3

#define SPI2_MISO_GPIO_PORT      GPIOB
#define SPI2_MISO_PIN            GPIO_PINS_4
#define SPI2_MISO_PIN_SOURCE     GPIO_PINS_SOURCE4
#define SPI2_MISO_IOMUX          GPIO_MUX_3

#define SPI2_MOSI_GPIO_PORT      GPIOB
#define SPI2_MOSI_PIN            GPIO_PINS_5
#define SPI2_MOSI_PIN_SOURCE     GPIO_PINS_SOURCE5
#define SPI2_MOSI_IOMUX          GPIO_MUX_3

#define SPI2_CS_GPIO_PORT        GPIOA
#define SPI2_CS_PIN              GPIO_PINS_15

/* ===== CAN1 (TJA1051) ===== */
#define CAN1_TX_GPIO_PORT        GPIOA
#define CAN1_TX_PIN              GPIO_PINS_12
#define CAN1_TX_PIN_SOURCE       GPIO_PINS_SOURCE12
#define CAN1_TX_IOMUX            GPIO_MUX_9

#define CAN1_RX_GPIO_PORT        GPIOA
#define CAN1_RX_PIN              GPIO_PINS_11
#define CAN1_RX_PIN_SOURCE       GPIO_PINS_SOURCE11
#define CAN1_RX_IOMUX            GPIO_MUX_9

/* ===== USART1 (finsh shell, 已从 PA9 移走避免冲突) ===== */
#define USART1_TX_GPIO_PORT      GPIOB
#define USART1_TX_PIN            GPIO_PINS_6
#define USART1_TX_PIN_SOURCE     GPIO_PINS_SOURCE6
#define USART1_TX_IOMUX          GPIO_MUX_7

#define USART1_RX_GPIO_PORT      GPIOB
#define USART1_RX_PIN            GPIO_PINS_7
#define USART1_RX_PIN_SOURCE     GPIO_PINS_SOURCE7
#define USART1_RX_IOMUX          GPIO_MUX_7

/* ===== LED ===== */
#define LED_GPIO_PORT            GPIOA
#define LED_PIN                  GPIO_PINS_0

/* ===== PWM 时序常数 (spec §1.3, 适配 180MHz sclk) ===== */
#define PWM_FREQUENCY_HZ         16000u
#define TMR1_CLOCK_HZ            180000000u   /* APB2 = sclk = 180MHz (APB2_DIV_1, 定时器不翻倍) */
#define TMR1_ARR                 5624u   /* 180MHz / (2 * 16kHz) - 1, 中心对齐 TWO_WAY_3 */
#define PWM_DUTY_MAX             (uint16_t)(TMR1_ARR * 0.95f)  /* 硬限幅 95% = 5342 */

#ifdef __cplusplus
}
#endif

#endif /* BOARD_MOTOR_PINS_H */
