/* add user code begin Header */
/**
  **************************************************************************
  * @file     at32f402_405_wk_config.c
  * @brief    work bench config program
  **************************************************************************
  * Copyright (c) 2025, Artery Technology, All rights reserved.
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* add user code end Header */

#include "at32m412_416_wk_config.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */

/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */

/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief  system clock config program
  * @note   the system clock is configured as follow:
  *         system clock (sclk)   = (hext * pll_ns)/(pll_ms * pll_fr)
  *         system clock source   = HEXT_VALUE
  *         - lick                = on
  *         - lext                = off
  *         - hick                = on
  *         - hext                = off
  *         - hext                = HEXT_VALUE
  *         - sclk                = 180000000
  *         - ahbdiv              = 1
  *         - ahbclk              = 180000000
  *         - apb1div             = 1
  *         - apb1clk             = 180000000
  *         - apb2div             = 1
  *         - apb2clk             = 180000000
  *         - apb3div             = 4
  *         - apb3clk             = 45000000
  *         - pll_ns              = 90
  *         - pll_ms              = 1
  *         - pll_fr              = 4
  *         - flash_wtcyc         = 5 cycle
  * @param  none
  * @retval none
  */
void wk_system_clock_config(void)
{
  /* reset crm */
  crm_reset();

  /* config flash psr register */
  flash_psr_set(FLASH_WAIT_CYCLE_5);

  /* enable pwc periph clock */
  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
  
  /* config ldo voltage */
  pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);

  /* enable lick */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_LICK, TRUE);

  /* wait till lick is ready */
  while(crm_flag_get(CRM_LICK_STABLE_FLAG) != SET)
  {
  }

  /* enable hext */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);

  /* wait till hext is ready */
  while(crm_hext_stable_wait() == ERROR)
  {
  }

  /* enable hick */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);

  /* wait till hick is ready */
  while(crm_flag_get(CRM_HICK_STABLE_FLAG) != SET)
  {
  }

  /* config pll clock resource */
  crm_pll_config(CRM_PLL_SOURCE_HEXT, 90, 1, CRM_PLL_FR_4);
  /* enable pll */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
  /* wait till pll is ready */
  while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET)
  {
  }

  /* config ahbclk */
  crm_ahb_div_set(CRM_AHB_DIV_1);

  /* config apb3clk, the maximum frequency of APB3 clock is 90 MHz */
  crm_apb3_div_set(CRM_APB3_DIV_4);

  /* config apb2clk, the maximum frequency of APB2 clock is 180 MHz */
  crm_apb2_div_set(CRM_APB2_DIV_1);

  /* config apb1clk, the maximum frequency of APB1 clock is 180 MHz */
  crm_apb1_div_set(CRM_APB1_DIV_1);

  /* enable auto step mode */
  crm_auto_step_mode_enable(TRUE);

  /* select pll as system clock source */
  crm_sysclk_switch(CRM_SCLK_PLL);

  /* wait till pll is used as system clock source */
  while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL)
  {
  }

  /* disable auto step mode */
  crm_auto_step_mode_enable(FALSE);

  /* update system_core_clock global variable */
  system_core_clock_update();
}

/**
  * @brief  config periph clock
  * @param  none
  * @retval none
  */
void wk_periph_clock_config(void)
{
  /* enable gpioa periph clock */
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

  /* enable gpiob periph clock */
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

  /* enable gpiof periph clock */
  crm_periph_clock_enable(CRM_GPIOF_PERIPH_CLOCK, TRUE);

  /* enable spi2 periph clock */
  crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);

  /* enable can1 periph clock */
  crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);

  /* enable tmr1 periph clock */
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

  /* enable usart1 periph clock */
  crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);

  /* enable adc2 periph clock */
  crm_periph_clock_enable(CRM_ADC2_PERIPH_CLOCK, TRUE);

  /* enable scfg periph clock */
  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
}

/**
  * @brief  nvic config
  * @param  none
  * @retval none
  */
void wk_nvic_config(void)
{
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
 
  NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
  nvic_irq_enable(ADC1_2_IRQn, 0, 0);
  nvic_irq_enable(CAN1_RX_IRQn, 0, 0);
}

/**
  * @brief  init gpio_input/gpio_output/gpio_analog/eventout function.
  * @param  none
  * @retval none
  */
void wk_gpio_config(void)
{
  /* add user code begin gpio_config 0 */

  /* add user code end gpio_config 0 */

  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin gpio_config 1 */

  /* add user code end gpio_config 1 */

  /* gpio input config */
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = nFAULT_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(nFAULT_GPIO_PORT, &gpio_init_struct);

  /* gpio output config */
  gpio_bits_reset(LED_GPIO_PORT, LED_PIN);
  gpio_bits_reset(PWM_EN_GPIO_PORT, PWM_EN_PIN);
  gpio_bits_set(SPI2_CS_GPIO_PORT, SPI2_CS_PIN);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = LED_PIN | SPI2_CS_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = PWM_EN_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(PWM_EN_GPIO_PORT, &gpio_init_struct);

  /* add user code begin gpio_config 2 */

  /* add user code end gpio_config 2 */
}

/**
  * @brief  init usart1 function
  * @param  none
  * @retval none
  */
void wk_usart1_init(void)
{
  /* add user code begin usart1_init 0 */

  /* add user code end usart1_init 0 */

  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin usart1_init 1 */

  /* add user code end usart1_init 1 */

  /* configure the TX pin */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE6, GPIO_MUX_7);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_6;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure the RX pin */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE7, GPIO_MUX_7);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_7;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure param */
  usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART1, TRUE);
  usart_receiver_enable(USART1, TRUE);
  usart_parity_selection_config(USART1, USART_PARITY_NONE);

  usart_hardware_flow_control_set(USART1, USART_HARDWARE_FLOW_NONE);

  /* add user code begin usart1_init 2 */

  /* add user code end usart1_init 2 */

  usart_enable(USART1, TRUE);

  /* add user code begin usart1_init 3 */

  /* add user code end usart1_init 3 */
}

/**
  * @brief  init can1 function.
  * @param  none
  * @retval none
  */
void wk_can1_init(void)
{
  /* add user code begin can1_init 0 */

  /* add user code end can1_init 0 */

  gpio_init_type gpio_init_struct;
  can_bittime_type can_bittime_struct;
  can_filter_config_type can_filter_struct;

  /* add user code begin can1_init 1 */

  /* add user code end can1_init 1 */

  /*gpio-----------------------------------------------------------------------------*/ 
  gpio_default_para_init(&gpio_init_struct);

  /* configure the CAN1 TX pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE12, GPIO_MUX_9);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_12;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure the CAN1 RX pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE11, GPIO_MUX_9);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_11;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  crm_can_clock_select(CRM_CAN1, CRM_CAN_CLOCK_SOURCE_PLL);

  can_software_reset(CAN1, TRUE);

  /*can_bit_time_setting-------------------------------------------------------------*/
  can_bittime_default_para_init(&can_bittime_struct);

  /*set boudrate = pclk/(bittime_div *(bts1_size + bts2_size))-----------------------*/
  can_bittime_struct.bittime_div = 1;
  can_bittime_struct.ac_bts1_size = 135;
  can_bittime_struct.ac_bts2_size = 45;
  can_bittime_struct.ac_rsaw_size = 45;
  can_bittime_set(CAN1, &can_bittime_struct);

  /*can_filter_0_config--------------------------------------------------------------*/
  can_filter_default_para_init(&can_filter_struct);

  can_filter_struct.mask_para.id_type = TRUE;
  can_filter_struct.code_para.id_type = CAN_ID_STANDARD;
  can_filter_struct.mask_para.id = 0x7FF;
  can_filter_struct.code_para.id = 0x0;
  can_filter_struct.mask_para.data_length = 0xF;
  can_filter_struct.code_para.data_length = 0x0;
  can_filter_struct.mask_para.frame_type = TRUE;
  can_filter_struct.code_para.frame_type = CAN_FRAME_DATA;
  can_filter_struct.mask_para.recv_frame = TRUE;
  can_filter_struct.code_para.recv_frame = CAN_RECV_NORMAL;
  can_filter_set(CAN1, CAN_FILTER_NUM_0, &can_filter_struct);

  can_software_reset(CAN1, FALSE);

  can_filter_enable(CAN1, CAN_FILTER_NUM_0, TRUE);

  /*can_base_config------------------------------------------------------------------*/
  can_retransmission_limit_set(CAN1, CAN_RE_TRANS_TIMES_UNLIMIT);
  can_rearbitration_limit_set(CAN1, CAN_RE_ARBI_TIMES_UNLIMIT);
  can_mode_set(CAN1, CAN_MODE_COMMUNICATE);
  can_stb_transmit_mode_set(CAN1, CAN_STB_TRANSMIT_BY_FIFO);
  can_rxbuf_warning_set(CAN1, 1);
  can_rxbuf_overflow_mode_set(CAN1, CAN_RXBUF_OVERFLOW_BE_OVWR);
  can_error_warning_set(CAN1, 11);
  can_restricted_operation_enable(CAN1, FALSE);
  can_receive_all_enable(CAN1, FALSE);

  /* add user code begin can1_init 2 */

  /* add user code end can1_init 2 */
}

/**
  * @brief  init adc-common function.
  * @param  none
  * @retval none
  */
void wk_adc_common_init(void)
{
  /* add user code begin adc_common_init 0 */

  /* add user code end adc_common_init 0 */

  adc_common_config_type adc_common_struct;

  /* add user code begin adc_common_init 1 */

  /* add user code end adc_common_init 1 */

  adc_reset();

  /* adc_common_settings------------------------------------------------------------ */
  adc_common_default_para_init(&adc_common_struct);
  adc_common_struct.combine_mode = ADC_ORDINARY_SMLT_PREEMPT_SMLT_ONESLAVE_MODE;
  adc_common_struct.div = ADC_HCLK_DIV_6;
  adc_common_struct.common_dma_mode = ADC_COMMON_DMAMODE_DISABLE;
  adc_common_struct.common_dma_request_repeat_state = FALSE;
  adc_common_struct.sampling_interval = ADC_SAMPLING_INTERVAL_4CYCLES;
  adc_common_struct.tempervintrv_state = FALSE;
  adc_common_config(&adc_common_struct);
  
  /* add user code begin adc_common_init 2 */

  /* add user code end adc_common_init 2 */
}

/**
  * @brief  init adc2 function.
  * @param  none
  * @retval none
  */
void wk_adc2_init(void)
{
  /* add user code begin adc2_init 0 */

  /* add user code end adc2_init 0 */

  gpio_init_type gpio_init_struct;
  adc_base_config_type adc_base_struct;

  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin adc2_init 1 */

  /* add user code end adc2_init 1 */

  /*gpio--------------------------------------------------------------------*/ 
  /* configure the IN6 pin */
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = VBUS_PIN;
  gpio_init(VBUS_GPIO_PORT, &gpio_init_struct);

  /* configure the IN7 pin */
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = SOC_PIN;
  gpio_init(SOC_GPIO_PORT, &gpio_init_struct);

  /* configure the IN8 pin */
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = SOB_PIN;
  gpio_init(SOB_GPIO_PORT, &gpio_init_struct);

  /* configure the IN9 pin */
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = SOA_PIN;
  gpio_init(SOA_GPIO_PORT, &gpio_init_struct);

  /* adc_settings------------------------------------------------------------------- */
  adc_base_default_para_init(&adc_base_struct);
  adc_base_struct.sequence_mode = FALSE;
  adc_base_struct.repeat_mode = FALSE;
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = 1;
  adc_base_config(ADC2, &adc_base_struct);

  adc_resolution_set(ADC2, ADC_RESOLUTION_12B);

  /* adc_ordinary_conversionmode---------------------------------------------------- */
  adc_ordinary_channel_set(ADC2, ADC_CHANNEL_6, 1, ADC_SAMPLETIME_1_5);

  /* When "ADC_ORDINARY_TRIG_EDGE_NONE" is selected, the external trigger source is invalid, and user can only use software trigger. \
  The software trigger function is adc_ordinary_software_trigger_enable(ADCx, TRUE); */
  adc_ordinary_conversion_trigger_set(ADC2, ADC_ORDINARY_TRIG_TMR1CH1, ADC_ORDINARY_TRIG_EDGE_NONE);

  /* adc_preempt_conversionmode----------------------------------------------------- */
  adc_preempt_channel_length_set(ADC2, 1);

  adc_preempt_channel_set(ADC2, ADC_CHANNEL_6, 1, ADC_SAMPLETIME_1_5);
  adc_preempt_offset_value_set(ADC2, ADC_PREEMPT_CHANNEL_1, 0x0);

  /* When "ADC_PREEMPT_TRIG_EDGE_NONE" is selected, the external trigger source is invalid, and user can only use software trigger. \
  The software trigger function is adc_preempt_software_trigger_enable(ADCx, TRUE); */
  adc_preempt_conversion_trigger_set(ADC2, ADC_PREEMPT_TRIG_TMR1CH4, ADC_PREEMPT_TRIG_EDGE_NONE);

  /* add user code begin adc2_init 2 */

  /* add user code end adc2_init 2 */

  adc_enable(ADC2, TRUE);
  while(adc_flag_get(ADC2, ADC_RDY_FLAG) == RESET);

  /* adc calibration---------------------------------------------------------------- */
  adc_calibration_init(ADC2);
  while(adc_calibration_init_status_get(ADC2));
  adc_calibration_start(ADC2);
  while(adc_calibration_status_get(ADC2));

  /* add user code begin adc2_init 3 */

  /* add user code end adc2_init 3 */
}

/**
  * @brief  init tmr1 function.
  * @param  none
  * @retval none
  */
void wk_tmr1_init(void)
{
  /* add user code begin tmr1_init 0 */

  /* add user code end tmr1_init 0 */

  gpio_init_type gpio_init_struct;
  tmr_output_config_type tmr_output_struct;
  tmr_brkdt_config_type tmr_brkdt_struct;

  tmr_brk2_config_type tmr_brk2_struct;


  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin tmr1_init 1 */

  /* add user code end tmr1_init 1 */

  /* configure the tmr1 CH1 pin */
  gpio_pin_mux_config(PWMA_GPIO_PORT, GPIO_PINS_SOURCE8, GPIO_MUX_1);
  gpio_init_struct.gpio_pins = PWMA_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(PWMA_GPIO_PORT, &gpio_init_struct);

  /* configure the tmr1 CH2 pin */
  gpio_pin_mux_config(PWMB_GPIO_PORT, GPIO_PINS_SOURCE9, GPIO_MUX_1);
  gpio_init_struct.gpio_pins = PWMB_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(PWMB_GPIO_PORT, &gpio_init_struct);

  /* configure the tmr1 CH3 pin */
  gpio_pin_mux_config(PWMC_GPIO_PORT, GPIO_PINS_SOURCE10, GPIO_MUX_1);
  gpio_init_struct.gpio_pins = PWMC_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(PWMC_GPIO_PORT, &gpio_init_struct);

  /* configure counter settings */
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);
  tmr_repetition_counter_set(TMR1, 0);
  tmr_period_buffer_enable(TMR1, FALSE);
  tmr_base_init(TMR1, 65535, 0);

  /* configure primary mode settings */
  tmr_sub_sync_mode_set(TMR1, FALSE);
  tmr_primary_mode_select(TMR1, TMR_PRIMARY_SEL_RESET);
  tmr_primary_mode2_select(TMR1, TMR_PRIMARY_SEL_RESET);

  /* configure channel 1 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, 0);
  tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_1, FALSE);

  tmr_output_channel_immediately_set(TMR1, TMR_SELECT_CHANNEL_1, FALSE);

  /* configure channel 2 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, 0);
  tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_2, FALSE);

  tmr_output_channel_immediately_set(TMR1, TMR_SELECT_CHANNEL_2, FALSE);

  /* configure channel 3 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, 0);
  tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_3, FALSE);

  tmr_output_channel_immediately_set(TMR1, TMR_SELECT_CHANNEL_3, FALSE);

  /* configure break and dead-time settings */
  tmr_brkdt_struct.brk_enable = FALSE;
  tmr_brkdt_struct.auto_output_enable = FALSE;
  tmr_brkdt_struct.brk_polarity = TMR_BRK_INPUT_ACTIVE_LOW;
  tmr_brkdt_struct.fcsoen_state = FALSE;
  tmr_brkdt_struct.fcsodis_state = FALSE;
  tmr_brkdt_struct.wp_level = TMR_WP_OFF;
  tmr_brkdt_struct.deadtime = 0;
  tmr_brkdt_config(TMR1, &tmr_brkdt_struct);
  tmr_brk_filter_value_set(TMR1, 0);

  /* break 2 config */
  tmr_brk2_struct.brk2_enable = FALSE;
  tmr_brk2_struct.brk2_polarity = TMR_BRK_INPUT_ACTIVE_LOW;
  tmr_brk2_struct.brk2_filter = 0;
  tmr_brk2_config(TMR1, &tmr_brk2_struct);

  tmr_output_enable(TMR1, TRUE);

  tmr_counter_enable(TMR1, TRUE);

  /* add user code begin tmr1_init 2 */

  /* add user code end tmr1_init 2 */
}

/**
  * @brief  init spi2 function
  * @param  none
  * @retval none
  */
void wk_spi2_init(void)
{
  /* add user code begin spi2_init 0 */

  /* add user code end spi2_init 0 */

  gpio_init_type gpio_init_struct;
  spi_init_type spi_init_struct;

  gpio_default_para_init(&gpio_init_struct);
  spi_default_para_init(&spi_init_struct);

  /* add user code begin spi2_init 1 */

  /* add user code end spi2_init 1 */

  /* configure the SCK pin */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE3, GPIO_MUX_6);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_3;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure the MISO pin */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE4, GPIO_MUX_6);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_4;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure the MOSI pin */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE5, GPIO_MUX_6);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_5;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure param */
  spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
  spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
  spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
  spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
  spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_8;
  spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
  spi_init_struct.clock_phase = SPI_CLOCK_PHASE_1EDGE;
  spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
  spi_init(SPI2, &spi_init_struct);

  /* add user code begin spi2_init 2 */

  /* add user code end spi2_init 2 */

  spi_enable(SPI2, TRUE);

  /* add user code begin spi2_init 3 */

  /* add user code end spi2_init 3 */
}

/**
  * @brief  init wdt function.
  * @param  none
  * @retval none
  */
void wk_wdt_init(void)
{
  /* add user code begin wdt_init 0 */

  /* add user code end wdt_init 0 */

  wdt_register_write_enable(TRUE);
  wdt_divider_set(WDT_CLK_DIV_8);
  wdt_reload_value_set(0xFFF);
  wdt_counter_reload();

  /* if enabled, please feed the dog through wdt_counter_reload() function */
  //wdt_enable();

  /* add user code begin wdt_init 1 */

  /* add user code end wdt_init 1 */
}

/* add user code begin 1 */

/* add user code end 1 */
