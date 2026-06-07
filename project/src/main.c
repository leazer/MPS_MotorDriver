/* add user code begin Header */
/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
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
#include "at32m412_416_wk_config.h"
#include "wk_adc.h"
#include "wk_can.h"
#include "wk_spi.h"
#include "wk_tmr.h"
#include "wk_usart.h"
#include "wk_wdt.h"
#include "wk_gpio.h"
#include "rtthread_app.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include "ma600a_debug.h"

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
  * @brief main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  /* add user code begin 1 */

  /* add user code end 1 */

  /* init gpio function. */
  wk_gpio_config();

  /* init adc-common function. */
  wk_adc_common_init();

  /* init adc2 function. */
  wk_adc2_init();

  /* init usart1 function. */
  wk_usart1_init();

  /* init can1 function. */
  wk_can1_init();

  /* init spi2 function. */
  wk_spi2_init();

  ma600a_debug_init();

  /* init wdt function. */
  wk_wdt_init();

  /* init tmr1 function. */
  wk_tmr1_init();

  /* add user code begin 2 */

  /* add user code end 2 */

  /* init rtthread function. */
  wk_rtthread_init();

  while(1)
  {
    rt_thread_mdelay(10);

    /* add user code begin 3 */
    ma600a_debug_poll();
    gpio_bits_toggle(LED_GPIO_PORT, LED_PIN);
    /* add user code end 3 */
  }
}

  /* add user code begin 4 */

  /* add user code end 4 */
