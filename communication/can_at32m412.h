#ifndef CAN_AT32M412_H
#define CAN_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* CAN1 初始化 (500kbps, spec §5.1) */
void can_at32m412_init(void);

/* 发送一帧 (阻塞, 超时 100ms) */
bool can_at32m412_send(uint32_t id, const uint8_t *data, uint8_t len);

/* 注册接收回调 (CAN RX 中断内调用) */
typedef void (*can_rx_callback_t)(uint32_t id, const uint8_t *data, uint8_t len);
void can_at32m412_register_rx(can_rx_callback_t cb);

#ifdef __cplusplus
}
#endif

#endif /* CAN_AT32M412_H */
