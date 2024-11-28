// TCA9548.h
#ifndef TCA9548_H
#define TCA9548_H

#include "main.h"

#define TCA9548_ADDR 0x70

// Ð?nh nghia các kênh c?m bi?n
#define CHANNEL_TOP_LEFT     4
#define CHANNEL_TOP_RIGHT    5
#define CHANNEL_BOTTOM_LEFT  6
#define CHANNEL_BOTTOM_RIGHT 7

void TCA9548_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t channel);

#endif
