#include "can_motion_timer_at32m412.h"

#include "at32m412_416.h"
#include "can_motion_service.h"

#define CAN_MOTION_TIMER             TMR6
#define CAN_MOTION_TIMER_IRQn        TMR6_DAC_GLOBAL_IRQn
#define CAN_MOTION_TIMER_CLOCK       CRM_TMR6_PERIPH_CLOCK
#define CAN_MOTION_TIMER_PRIO        4u
#define CAN_MOTION_TIMER_PRESCALER   179u
#define CAN_MOTION_TIMER_PERIOD      999u

void can_motion_timer_at32m412_init(void)
{
    crm_periph_clock_enable(CAN_MOTION_TIMER_CLOCK, TRUE);
    tmr_reset(CAN_MOTION_TIMER);

    tmr_base_init(CAN_MOTION_TIMER, CAN_MOTION_TIMER_PERIOD,
                  CAN_MOTION_TIMER_PRESCALER);
    tmr_cnt_dir_set(CAN_MOTION_TIMER, TMR_COUNT_UP);
    tmr_period_buffer_enable(CAN_MOTION_TIMER, TRUE);
    tmr_flag_clear(CAN_MOTION_TIMER, TMR_OVF_FLAG);
    tmr_interrupt_enable(CAN_MOTION_TIMER, TMR_OVF_INT, TRUE);

    nvic_irq_enable(CAN_MOTION_TIMER_IRQn, CAN_MOTION_TIMER_PRIO, 0);
    tmr_counter_enable(CAN_MOTION_TIMER, TRUE);
}

void TMR6_DAC_GLOBAL_IRQHandler(void)
{
    if (tmr_flag_get(CAN_MOTION_TIMER, TMR_OVF_FLAG) != RESET) {
        tmr_flag_clear(CAN_MOTION_TIMER, TMR_OVF_FLAG);
        can_motion_service_tick_1ms();
    }
}
