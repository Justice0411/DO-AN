#include "INA219.h"
extern I2C_HandleTypeDef hi2c2;
void INA219_Init(I2C_HandleTypeDef *hi2c) {
  uint16_t calibration_value = 4096; // Ði?u ch?nh theo c?m bi?n và nhu c?u do
  uint8_t data[3];
  data[0] = INA219_REG_CALIBRATION;
  data[1] = (calibration_value >> 8) & 0xFF;
  data[2] = calibration_value & 0xFF;
  HAL_I2C_Master_Transmit(hi2c, INA219_ADDR, data, 3, HAL_MAX_DELAY);  
	// Kh?i t?o c?m bi?n, thi?t l?p các giá tr? c?n thi?t
}

float INA219_ReadVoltage(I2C_HandleTypeDef *hi2c) {
    uint8_t data[2];
    HAL_I2C_Mem_Read(hi2c, INA219_ADDR, 0x02, I2C_MEMADD_SIZE_8BIT, data, 2, 1000);
    int16_t voltage = (data[0] << 8) | data[1];
		voltage >>= 3;
    return voltage * 0.004;// Chuy?n d?i giá tr? d?c du?c thành volt
}

float INA219_ReadCurrent(I2C_HandleTypeDef *hi2c) {
    uint8_t data[2];
    HAL_I2C_Mem_Read(hi2c, INA219_ADDR, 0x04, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
    int16_t current = (data[0] << 8) | data[1];
    return current * 0.001; // Chuy?n d?i giá tr? d?c du?c thành ampere
}

