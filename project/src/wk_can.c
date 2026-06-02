/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_can.c
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

/* Includes ------------------------------------------------------------------*/
#include "wk_can.h"

/* add user code begin 0 */

/* add user code end 0 */

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

/* add user code begin 1 */

/* add user code end 1 */
