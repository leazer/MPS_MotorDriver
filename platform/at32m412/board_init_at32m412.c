#include "board_init_at32m412.h"
#include "board_motor_pins.h"
#include "at32m412_416.h"

/* spec §3.5 中断优先级 (NVIC_PRIORITY_GROUP_4: 4 位抢占 / 0 位子) */
#define PRIO_FOC_ISR      0   /* TMR1_OVF_TMR10, 最高 */
#define PRIO_NFAULT       1   /* EXINT2 */
#define PRIO_ADC          2   /* ADC1_2 */
#define PRIO_CAN_RX       3   /* CAN1_RX */
#define PRIO_SYSTICK      14  /* RT-Thread 节拍 */
#define PRIO_PENDSV       15  /* RT-Thread 调度, 最低 */

void board_clock_init(void)
{
    /* GPIO 端口 */
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    /* GPIOF 不用 (board_motor_pins.h 无 GPIOF 引脚) */

    /* 高级定时器 (PWM) */
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

    /* USART1 (finsh console) */
    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);

    /* SCFG: PA15(SPI2_CS) 等 GPIO 复用配置需要 */
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);

    /* PWC: 系统时钟 (wk_system_clock_config 内部已开, 此处冗余开一次无害) */
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);

    /* SPI2 / CAN1 / USART1 / ADC2 时钟由各模块 Stage 初始化时自行开启 */
}

void board_gpio_init(void)
{
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);

    /* 先写电平再配模式, 避免上电毛刺导致 MP6540H 误使能 */
    gpio_bits_reset(LED_GPIO_PORT, LED_PIN);           /* LED 灭 */
    gpio_bits_reset(PWM_EN_GPIO_PORT, PWM_EN_PIN);     /* MP6540H EN=低, 禁用 */
    gpio_bits_set(SPI2_CS_GPIO_PORT, SPI2_CS_PIN);     /* SPI CS=高, 片选无效 */

    /* nFAULT (PB2): 输入上拉 */
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = nFAULT_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(nFAULT_GPIO_PORT, &gpio_init_struct);

    /* LED (PA0) + SPI2_CS (PA15): 推挽输出 */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = LED_PIN | SPI2_CS_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio_init_struct);

    /* PWM_EN (PB10): 推挽输出, 默认低 (MP6540H 禁用) */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = PWM_EN_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(PWM_EN_GPIO_PORT, &gpio_init_struct);
}

void board_nvic_init(void)
{
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    /* 系统异常: 最高优先级 0 */
    NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_SetPriority(BusFault_IRQn,          NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_SetPriority(UsageFault_IRQn,        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_SetPriority(SVCall_IRQn,            NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_SetPriority(DebugMonitor_IRQn,      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

    /* RT-Thread 调度: SysTick=14, PendSV=15 (最低, 给 FOC/nFAULT/CAN 留优先级空间) */
    NVIC_SetPriority(SysTick_IRQn,  NVIC_EncodePriority(NVIC_GetPriorityGrouping(), PRIO_SYSTICK, 0));
    NVIC_SetPriority(PendSV_IRQn,   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), PRIO_PENDSV, 0));

    /* 外设中断: 只设优先级, 不使能 (各 Stage 实现 ISR 后再 nvic_irq_enable, 避免空 ISR 死循环) */
    NVIC_SetPriority(TMR1_OVF_TMR10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), PRIO_FOC_ISR, 0));
    NVIC_SetPriority(EXINT2_IRQn,         NVIC_EncodePriority(NVIC_GetPriorityGrouping(), PRIO_NFAULT, 0));
    NVIC_SetPriority(ADC1_2_IRQn,         NVIC_EncodePriority(NVIC_GetPriorityGrouping(), PRIO_ADC, 0));
    NVIC_SetPriority(CAN1_RX_IRQn,        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), PRIO_CAN_RX, 0));
}

void board_usart1_init(void)
{
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);

    /* PB6 (TX) 复用 MUX_7 */
    gpio_pin_mux_config(USART1_TX_GPIO_PORT, USART1_TX_PIN_SOURCE, USART1_TX_IOMUX);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = USART1_TX_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(USART1_TX_GPIO_PORT, &gpio_init_struct);

    /* PB7 (RX) 复用 MUX_7 */
    gpio_pin_mux_config(USART1_RX_GPIO_PORT, USART1_RX_PIN_SOURCE, USART1_RX_IOMUX);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = USART1_RX_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(USART1_RX_GPIO_PORT, &gpio_init_struct);

    /* 115200 8N1, 无流控, 使能收发 */
    usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART1, TRUE);
    usart_receiver_enable(USART1, TRUE);
    usart_parity_selection_config(USART1, USART_PARITY_NONE);
    usart_hardware_flow_control_set(USART1, USART_HARDWARE_FLOW_NONE);
    usart_enable(USART1, TRUE);
}
