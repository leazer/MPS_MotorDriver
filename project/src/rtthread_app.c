/* add user code begin Header */
/**
  ******************************************************************************
  * File Name          : rtthread_app.c
  * Description        : Code for rtthread applications
  */
/* add user code end Header */

/* Includes ------------------------------------------------------------------*/
#include "rtthread_app.h"

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

/* add user code begin 1 */

/* add user code end 1 */

/* if there is not enable heap, we should use static thread and stack. */
ALIGN(8)
static rt_uint8_t my_task01_stack[256];
struct rt_thread my_task01;

/**
  * @brief  initializes all tasks.
  * @param  none
  * @retval none
  */
void rtthread_task_create(void)
{
  rt_err_t result;

  /* initializes my_task01 */
  result = rt_thread_init(&my_task01,
                          "my_task01",
                          my_task01_func,
                          RT_NULL,
                          my_task01_stack,
                          sizeof(my_task01_stack),
                          0,
                          20);
  RT_ASSERT(result == RT_EOK);

  rt_thread_startup(&my_task01);

  /* using to eliminate the warning */
  (void)result;
}

/**
  * @brief  rtthread init and begin run.
  * @param  none
  * @retval none
  */
void wk_rtthread_init(void)
{
  /* add user code begin rtthread_init 0 */

  /* add user code end rtthread_init 0 */

  rtthread_task_create();

  /* add user code begin rtthread_init 1 */

  /* add user code end rtthread_init 1 */
}

/**
  * @brief my_task01 function.
  * @param  none
  * @retval none
  */
void my_task01_func(void *parameter)
{
  /* add user code begin my_task01_func 0 */

  /* add user code end my_task01_func 0 */

  /* add user code begin my_task01_func 1 */

  /* add user code end my_task01_func 1 */

  /* Infinite loop */
  while(1)
  {
    /* add user code begin my_task01_func 2 */

    rt_thread_mdelay(1);

    /* add user code end my_task01_func 2 */
  }
}

/* add user code begin 2 */

/* add user code end 2 */

