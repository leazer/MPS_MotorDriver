#ifndef CAN_FRAME_H
#define CAN_FRAME_H

#include <stdint.h>

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

#endif /* CAN_FRAME_H */
