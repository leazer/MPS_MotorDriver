#include "can_at32m412.h"

#include <stddef.h>
#include <string.h>

#include "at32m412_416.h"

#define CAN_RX_QUEUE_CAPACITY 8u
#define CAN_TX_QUEUE_CAPACITY 8u
#define CAN_RX_QUEUE_MASK     (CAN_RX_QUEUE_CAPACITY - 1u)
#define CAN_TX_QUEUE_MASK     (CAN_TX_QUEUE_CAPACITY - 1u)
#define CAN_HW_RX_DRAIN_LIMIT 8u
#define CAN_IRQ_PRIORITY      3u

static can_frame_t s_rx_queue[CAN_RX_QUEUE_CAPACITY];
static can_frame_t s_tx_queue[CAN_TX_QUEUE_CAPACITY];
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;
static volatile uint8_t s_tx_head;
static volatile uint8_t s_tx_tail;
static volatile bool s_tx_active;
static volatile uint8_t s_tx_active_handle;
static uint8_t s_tx_next_handle;
static uint8_t s_node_id;
static volatile can_at32m412_diag_t s_diag;

static void can_sat_increment(volatile uint32_t *counter)
{
    if (*counter != 0xffffffffu) {
        ++(*counter);
    }
}

static bool can_rx_queue_full(void)
{
    return (uint8_t)(s_rx_head - s_rx_tail) >= CAN_RX_QUEUE_CAPACITY;
}

static bool can_tx_queue_full(void)
{
    return (uint8_t)(s_tx_head - s_tx_tail) >= CAN_TX_QUEUE_CAPACITY;
}

static void can_diag_reset(void)
{
    s_diag.rec = 0u;
    s_diag.tec = 0u;
    s_diag.error_passive = false;
    s_diag.bus_off_latched = false;
    s_diag.fatal_latched = false;
    s_diag.rx_received = 0u;
    s_diag.rx_rejected = 0u;
    s_diag.rx_overflow = 0u;
    s_diag.tx_queued = 0u;
    s_diag.tx_completed = 0u;
    s_diag.tx_rejected = 0u;
    s_diag.tx_errors = 0u;
    s_diag.status_irqs = 0u;
    s_diag.error_irqs = 0u;
}

static void can_configure_exact_filter(can_filter_type filter_number, uint16_t id)
{
    can_filter_config_type filter;

    can_filter_default_para_init(&filter);
    filter.code_para.id = id;
    filter.code_para.id_type = CAN_ID_STANDARD;
    filter.code_para.frame_type = CAN_FRAME_DATA;
    filter.code_para.data_length = CAN_DLC_BYTES_8;
    filter.mask_para.id = 0x7ffu;
    filter.mask_para.id_type = TRUE;
    filter.mask_para.frame_type = TRUE;
    filter.mask_para.data_length = 0x0fu;
    can_filter_set(CAN1, filter_number, &filter);
    can_filter_enable(CAN1, filter_number, TRUE);
}

static bool can_rx_frame_valid(const can_rxbuf_type *rxbuf)
{
    if (rxbuf->id_type != CAN_ID_STANDARD) {
        return false;
    }
    if (rxbuf->frame_type != CAN_FRAME_DATA) {
        return false;
    }
    if (rxbuf->data_length != CAN_DLC_BYTES_8) {
        return false;
    }
    return (rxbuf->id == 0x080u) ||
           (rxbuf->id == (uint32_t)(0x100u + s_node_id));
}

static void can_snapshot_error_state(void)
{
    s_diag.rec = can_receive_error_counter_get(CAN1);
    s_diag.tec = can_transmit_error_counter_get(CAN1);
    s_diag.error_passive = (can_flag_get(CAN1, CAN_EPASS_FLAG) != RESET);
    if (can_busoff_get(CAN1) != RESET) {
        s_diag.bus_off_latched = true;
        s_diag.fatal_latched = true;
    }
}

static void can_tx_start_next(void)
{
    can_txbuf_type txbuf;
    const can_frame_t *frame;
    uint8_t handle;

    if (s_tx_active || s_diag.bus_off_latched || (s_tx_head == s_tx_tail)) {
        return;
    }

    frame = &s_tx_queue[s_tx_tail & CAN_TX_QUEUE_MASK];
    txbuf.id = frame->id;
    txbuf.id_type = CAN_ID_STANDARD;
    txbuf.frame_type = CAN_FRAME_DATA;
    txbuf.data_length = CAN_DLC_BYTES_8;
    memcpy(txbuf.data, frame->data, 8u);
    txbuf.tx_timestamp = FALSE;
    handle = s_tx_next_handle++;
    txbuf.handle = handle;

    if (can_txbuf_write(CAN1, CAN_TXBUF_PTB, &txbuf) != SUCCESS) {
        can_sat_increment(&s_diag.tx_errors);
        return;
    }

    s_tx_active_handle = handle;
    __DMB();
    s_tx_active = true;
    if (can_txbuf_transmit(CAN1, CAN_TRANSMIT_PTB) != SUCCESS) {
        s_tx_active = false;
        can_sat_increment(&s_diag.tx_errors);
        return;
    }
}

bool can_at32m412_init(uint8_t node_id)
{
    gpio_init_type gpio;
    can_bittime_type bittime;
    uint8_t filter_number;

    if ((node_id != 1u) && (node_id != 2u)) {
        return false;
    }

    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);
    crm_can_clock_select(CRM_CAN1, CRM_CAN_CLOCK_SOURCE_PLL);

    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE12, GPIO_MUX_9);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE11, GPIO_MUX_9);
    gpio_default_para_init(&gpio);
    gpio.gpio_pins = GPIO_PINS_12 | GPIO_PINS_11;
    gpio.gpio_mode = GPIO_MODE_MUX;
    gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio.gpio_pull = GPIO_PULL_NONE;
    gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init(GPIOA, &gpio);

    nvic_irq_disable(CAN1_RX_IRQn);
    nvic_irq_disable(CAN1_TX_IRQn);
    nvic_irq_disable(CAN1_STAT_IRQn);
    nvic_irq_disable(CAN1_ERR_IRQn);

    s_node_id = node_id;
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_tx_head = 0u;
    s_tx_tail = 0u;
    s_tx_active = false;
    s_tx_active_handle = 0u;
    s_tx_next_handle = 0u;
    can_diag_reset();

    can_reset(CAN1);
    can_software_reset(CAN1, TRUE);
    can_bittime_default_para_init(&bittime);
    bittime.bittime_div = 10u;
    bittime.ac_bts1_size = 14u;
    bittime.ac_bts2_size = 4u;
    bittime.ac_rsaw_size = 2u;
    can_bittime_set(CAN1, &bittime);
    can_mode_set(CAN1, CAN_MODE_COMMUNICATE);
    can_retransmission_limit_set(CAN1, CAN_RE_TRANS_TIMES_UNLIMIT);
    can_rearbitration_limit_set(CAN1, CAN_RE_ARBI_TIMES_UNLIMIT);
    can_rxbuf_overflow_mode_set(CAN1, CAN_RXBUF_OVERFLOW_BE_LOSE);
    can_receive_all_enable(CAN1, FALSE);

    for (filter_number = 0u; filter_number < 16u; ++filter_number) {
        can_filter_enable(CAN1, (can_filter_type)filter_number, FALSE);
    }
    can_configure_exact_filter(CAN_FILTER_NUM_0, 0x080u);
    can_configure_exact_filter(CAN_FILTER_NUM_1,
                               (uint16_t)(0x100u + node_id));

    can_flag_clear(CAN1, CAN_ALL_FLAG);
    can_interrupt_enable(CAN1, CAN_RIE_INT | CAN_TPIE_INT |
                               CAN_EPIE_INT | CAN_EIE_INT |
                               CAN_BEIE_INT | CAN_ROIE_INT, TRUE);
    can_software_reset(CAN1, FALSE);

    nvic_irq_enable(CAN1_RX_IRQn, 3u, 0u);
    nvic_irq_enable(CAN1_TX_IRQn, 3u, 0u);
    nvic_irq_enable(CAN1_STAT_IRQn, 3u, 0u);
    nvic_irq_enable(CAN1_ERR_IRQn, 3u, 0u);
    return true;
}

bool can_at32m412_rx_pop(can_frame_t *out)
{
    if (out == NULL) {
        return false;
    }
    if (s_rx_head == s_rx_tail) {
        return false;
    }

    __DMB();
    *out = s_rx_queue[s_rx_tail & CAN_RX_QUEUE_MASK];
    __DMB();
    ++s_rx_tail;
    return true;
}

bool can_at32m412_tx_push(const can_frame_t *frame)
{
    if ((frame == NULL) || (frame->id > 0x7ffu) || (frame->dlc != 8u)) {
        can_sat_increment(&s_diag.tx_rejected);
        return false;
    }
    if (can_tx_queue_full()) {
        can_sat_increment(&s_diag.tx_rejected);
        return false;
    }

    s_tx_queue[s_tx_head & CAN_TX_QUEUE_MASK] = *frame;
    __DMB();
    ++s_tx_head;
    can_sat_increment(&s_diag.tx_queued);
    return true;
}

void can_at32m412_tx_kick(void)
{
    nvic_irq_disable(CAN1_TX_IRQn);
    can_tx_start_next();
    nvic_irq_enable(CAN1_TX_IRQn, CAN_IRQ_PRIORITY, 0u);
}

void can_at32m412_get_diag(can_at32m412_diag_t *out)
{
    if (out == NULL) {
        return;
    }

    can_snapshot_error_state();
    out->rec = s_diag.rec;
    out->tec = s_diag.tec;
    out->error_passive = s_diag.error_passive;
    out->bus_off_latched = s_diag.bus_off_latched;
    out->fatal_latched = s_diag.fatal_latched;
    out->rx_received = s_diag.rx_received;
    out->rx_rejected = s_diag.rx_rejected;
    out->rx_overflow = s_diag.rx_overflow;
    out->tx_queued = s_diag.tx_queued;
    out->tx_completed = s_diag.tx_completed;
    out->tx_rejected = s_diag.tx_rejected;
    out->tx_errors = s_diag.tx_errors;
    out->status_irqs = s_diag.status_irqs;
    out->error_irqs = s_diag.error_irqs;
}

bool can_at32m412_fatal_bus_error(void)
{
    return s_diag.fatal_latched;
}

void can_at32m412_irq_rx(void)
{
    can_rxbuf_type rxbuf;
    can_rxbuf_status_type hw_status;
    can_frame_t frame;
    uint8_t drained;

    for (drained = 0u; drained < CAN_HW_RX_DRAIN_LIMIT; ++drained) {
        hw_status = can_rxbuf_status_get(CAN1);
        if (hw_status == CAN_RXBUF_STATUS_EMPTY) {
            break;
        }
        if (hw_status == CAN_RXBUF_STATUS_OVERFLOW) {
            can_sat_increment(&s_diag.rx_overflow);
            s_diag.fatal_latched = true;
            can_flag_clear(CAN1, CAN_ROIF_FLAG);
        }
        if (can_rxbuf_read(CAN1, &rxbuf) != SUCCESS) {
            (void)can_rxbuf_release(CAN1);
            can_sat_increment(&s_diag.rx_rejected);
            continue;
        }
        if (!can_rx_frame_valid(&rxbuf)) {
            can_sat_increment(&s_diag.rx_rejected);
            continue;
        }
        if (can_rx_queue_full()) {
            can_sat_increment(&s_diag.rx_overflow);
            s_diag.fatal_latched = true;
            continue;
        }

        frame.id = (uint16_t)rxbuf.id;
        frame.dlc = 8u;
        memcpy(frame.data, rxbuf.data, 8u);
        s_rx_queue[s_rx_head & CAN_RX_QUEUE_MASK] = frame;
        __DMB();
        ++s_rx_head;
        can_sat_increment(&s_diag.rx_received);
    }

    can_flag_clear(CAN1, CAN_RIF_FLAG | CAN_RFIF_FLAG | CAN_RAFIF_FLAG);
}

void can_at32m412_irq_tx(void)
{
    can_transmit_status_type status;

    if (can_flag_get(CAN1, CAN_TPIF_FLAG) != RESET) {
        can_flag_clear(CAN1, CAN_TPIF_FLAG);
        can_transmit_status_get(CAN1, &status);
        if (s_tx_active &&
            (status.final_handle == s_tx_active_handle) &&
            (status.final_tstat == CAN_TSTAT_TRANSMITTED)) {
            s_tx_tail++;
            can_sat_increment(&s_diag.tx_completed);
        } else if (s_tx_active) {
            can_sat_increment(&s_diag.tx_errors);
        }
        s_tx_active = false;
    }
    can_tx_start_next();
}

void can_at32m412_irq_status(void)
{
    can_sat_increment(&s_diag.status_irqs);
    can_snapshot_error_state();
    can_flag_clear(CAN1, CAN_EPIF_FLAG);
}

void can_at32m412_irq_error(void)
{
    can_sat_increment(&s_diag.error_irqs);
    if (can_flag_get(CAN1, CAN_ROIF_FLAG) != RESET) {
        can_sat_increment(&s_diag.rx_overflow);
        s_diag.fatal_latched = true;
    }
    can_snapshot_error_state();
    can_flag_clear(CAN1, CAN_EIF_FLAG | CAN_BEIF_FLAG | CAN_ROIF_FLAG);
}
