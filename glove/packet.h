#include "stdint.h"

#define ID_LENGTH 12

typedef struct {
        uint8_t rfid[ID_LENGTH];
        float ex, ey, ez;
        float ax, ay, az;
        uint16_t myo;
        uint8_t key : 1;
        uint8_t sw : 1;
        uint8_t capsens : 1;
        uint8_t lastnr : 1;
} Packet;
