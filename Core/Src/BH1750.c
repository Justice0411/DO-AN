// BH1750.c
#include "BH1750.h"
#include <math.h>

void BH1750_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t power_on = BH1750_POWER_ON;
    uint8_t continuous_mode = BH1750_CONTINUOUS_MODE;
    
    HAL_I2C_Master_Transmit(hi2c, BH1750_ADDR<<1, &power_on, 1, 100);
    HAL_Delay(10);
    HAL_I2C_Master_Transmit(hi2c, BH1750_ADDR<<1, &continuous_mode, 1, 100);
}

float BH1750_ReadLight(I2C_HandleTypeDef *hi2c) {
    uint8_t data[2];
    uint16_t light;
    
    HAL_I2C_Master_Receive(hi2c, BH1750_ADDR<<1, data, 2, 100);
    light = (data[0] << 8) | data[1];
    return light / 1.2f;  // Chuy?n d?i sang lux
}

