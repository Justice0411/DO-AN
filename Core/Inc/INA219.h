#ifndef __INA219_H
#define __INA219_H

#include "stm32f1xx_hal.h"
#define INA219_ADDR 0x40 << 1 // Ð?a ch? I2C c?a INA219
#define INA219_REG_CONFIG 0x00
#define INA219_REG_SHUNTVOLTAGE 0x01
#define INA219_REG_BUSVOLTAGE 0x02
#define INA219_REG_POWER 0x03
#define INA219_REG_CURRENT 0x04
#define INA219_REG_CALIBRATION 0x05
// Khai báo các hàm
void INA219_Init(I2C_HandleTypeDef *hi2c);
float INA219_ReadVoltage(I2C_HandleTypeDef *hi2c);
float INA219_ReadCurrent(I2C_HandleTypeDef *hi2c);

#endif

