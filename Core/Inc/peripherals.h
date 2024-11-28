#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "stm32f1xx_hal.h"

// Khai báo các bi?n external
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;

//// Khai báo các hàm
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
void MX_TIM1_Init(void);
void MX_USART2_UART_Init(void);
void Error_Handler(void);

#endif // PERIPHERALS_H
