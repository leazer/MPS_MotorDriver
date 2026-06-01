/* add user code begin Header */
/**
  ******************************************************************************
  * File Name          : rtthread_app.h
  * Description        : Code for rtthread applications
  */
/* add user code end Header */
  
#ifndef RTTHREAD_APP_H
#define RTTHREAD_APP_H

/* Includes ------------------------------------------------------------------*/
#include <rthw.h>
#include <rtthread.h>
#include "at32m412_416_wk_config.h"

/* private includes -------------------------------------------------------------*/
/* add user code begin private includes */

/* add user code end private includes */

/* exported types -------------------------------------------------------------*/
/* add user code begin exported types */

/* add user code end exported types */

/* exported constants --------------------------------------------------------*/
/* add user code begin exported constants */

/* add user code end exported constants */

/* exported macro ------------------------------------------------------------*/
/* add user code begin exported macro */

/* add user code end exported macro */

/* exported task macro */
extern struct rt_thread my_task01;

/* declaration for task function */
void my_task01_func(void *parameter);

/* add user code begin 0 */

/* add user code end 0 */

void wk_rtthread_init(void);

/* add user code begin 1 */

/* add user code end 1 */

#endif /* RTTHREAD_APP_H */
