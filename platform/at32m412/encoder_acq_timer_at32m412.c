#include "encoder_acq_timer_at32m412.h"
#include "encoder_service.h"
#include "at32m412_416.h"

#define ENCODER_ACQ_TIMER          TMR7
#define ENCODER_ACQ_TIMER_IRQn     TMR7_GLOBAL_IRQn
#define ENCODER_ACQ_TIMER_CLOCK    CRM_TMR7_PERIPH_CLOCK
#define ENCODER_ACQ_TIMER_PRIO     4u
#define ENCODER_ACQ_TIMER_CLK_HZ   180000000u
#define ENCODER_ACQ_TIMER_DIV      44u
#define ENCODER_ACQ_TIMER_PERIOD   ((ENCODER_ACQ_TIMER_CLK_HZ / ((ENCODER_ACQ_TIMER_DIV + 1u) * ENCODER_ACQ_TIMER_HZ)) - 1u)

void encoder_acq_timer_at32m412_init(void)
{
    crm_periph_clock_enable(ENCODER_ACQ_TIMER_CLOCK, TRUE);
    tmr_reset(ENCODER_ACQ_TIMER);

    tmr_base_init(ENCODER_ACQ_TIMER, ENCODER_ACQ_TIMER_PERIOD, ENCODER_ACQ_TIMER_DIV);
    tmr_cnt_dir_set(ENCODER_ACQ_TIMER, TMR_COUNT_UP);
    tmr_period_buffer_enable(ENCODER_ACQ_TIMER, TRUE);
    tmr_flag_clear(ENCODER_ACQ_TIMER, TMR_OVF_FLAG);
    tmr_interrupt_enable(ENCODER_ACQ_TIMER, TMR_OVF_INT, TRUE);

    nvic_irq_enable(ENCODER_ACQ_TIMER_IRQn, ENCODER_ACQ_TIMER_PRIO, 0);
    tmr_counter_enable(ENCODER_ACQ_TIMER, TRUE);
}

void TMR7_GLOBAL_IRQHandler(void)
{
    if (tmr_flag_get(ENCODER_ACQ_TIMER, TMR_OVF_FLAG) != RESET) {
        tmr_flag_clear(ENCODER_ACQ_TIMER, TMR_OVF_FLAG);
        (void)encoder_service_acquire_once();
    }
}
