// TCA9548.c
#include "TCA.h"

void TCA9548_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t channel) {
    uint8_t tca_channel = (1 << channel);
    HAL_I2C_Master_Transmit(hi2c, TCA9548_ADDR<<1, &tca_channel, 1, 100);
    HAL_Delay(10);
}
