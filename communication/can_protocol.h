#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "motor_params.h"

/* CAN ID 编码 (spec §5.1): (function_code << 7) | node_id */
#define CAN_ID_CONTROL           ((0x02u << 7) | MOTOR_NODE_ID)   /* 0x101 */
#define CAN_ID_STATUS            ((0x03u << 7) | MOTOR_NODE_ID)   /* 0x181 */
#define CAN_ID_EXT_STATUS        ((0x05u << 7) | MOTOR_NODE_ID)   /* 0x281 */

/* 控制模式 (spec §3.4) */
typedef enum {
    CAN_MODE_OPEN_LOOP  = 0,
    CAN_MODE_CURRENT    = 1,
    CAN_MODE_SPEED      = 2,
    CAN_MODE_POSITION   = 3,
    CAN_MODE_ALIGN      = 4,
    CAN_MODE_CALIBRATE  = 5,
} can_mode_t;

/* 解析收到的控制帧 (8 bytes), 更新 motor_control 状态 */
void can_protocol_handle_control(const uint8_t *data, uint8_t len);

/* 组装状态帧 (8 bytes) 用于发送 */
void can_protocol_build_status(uint8_t *data, uint8_t *len);

/* 组装扩展状态帧 (8 bytes) */
void can_protocol_build_ext_status(uint8_t *data, uint8_t *len);

#ifdef __cplusplus
}
#endif

#endif /* CAN_PROTOCOL_H */
