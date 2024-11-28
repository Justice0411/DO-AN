// BH1750.h
#ifndef BH1750_H
#define BH1750_H

#include "main.h"

// �?nh nghia d?a ch? I2C BH1750
#define BH1750_ADDR 0x23
// C�c l?nh c?a BH1750
#define BH1750_POWER_ON 0x01
#define BH1750_CONTINUOUS_MODE 0x10

// Threshold cho vi?c di?u ch?nh servo
#define LIGHT_THRESHOLD 50.0f  // Ngu?ng ch�nh l?ch �nh s�ng
#define SERVO_STEP 10          // �? di?u ch?nh m?i bu?c c?a servo

// Khai b�o prototype
void BH1750_Init(I2C_HandleTypeDef *hi2c);
float BH1750_ReadLight(I2C_HandleTypeDef *hi2c);
#endif

