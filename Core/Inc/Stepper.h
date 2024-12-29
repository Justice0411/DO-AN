//// stepper.h
#ifndef STEPPER_H
#define STEPPER_H
#include "main.h"
// Ð?nh nghia chân GPIO chính xác
#define VERTICAL_STEPPER_EN_PIN    GPIO_PIN_10     // PA6 
#define VERTICAL_STEPPER_DIR_PIN   GPIO_PIN_11     // PA7
#define HORIZONTAL_STEPPER_EN_PIN  GPIO_PIN_6    // PA10
#define HORIZONTAL_STEPPER_DIR_PIN GPIO_PIN_7    // PA11
#define VERTICAL_STEPPER_EN_PORT   GPIOA
#define VERTICAL_STEPPER_DIR_PORT  GPIOA
#define HORIZONTAL_STEPPER_EN_PORT GPIOA
#define HORIZONTAL_STEPPER_DIR_PORT GPIOA

// C?u hình d?ng co bu?c
#define STEPS_PER_REVOLUTION 200    // 1.8° m?i bu?c
#define GEAR_RATIO 1        // Không có h?p s?
#define VERTICAL_STEPPER_MICROSTEPS 16   // Microstep cho d?ng co d?c
#define HORIZONTAL_STEPPER_MICROSTEPS 8 // Microstep cho d?ng co ngang

typedef enum {
    HORIZONTAL_STEPPER,
    VERTICAL_STEPPER
} StepperType;

typedef struct {
    GPIO_TypeDef* EN_PORT;
    uint16_t EN_PIN;
	
    GPIO_TypeDef* DIR_PORT;
    uint16_t DIR_PIN;
	
    TIM_HandleTypeDef* timer;
    uint32_t channel;
	
    int32_t current_position;
    int32_t target_position;
	
    uint32_t steps_per_revolution;
		uint32_t microsteps;
    float current_angle;
		float home_position;    // Thêm v? trí home
	
    uint32_t speed;
    uint8_t is_enabled;
		StepperType type;      // Thêm lo?i d?ng co
} Stepper_HandleTypeDef;
// External declarations for steppers
extern Stepper_HandleTypeDef vertical_stepper;
extern Stepper_HandleTypeDef horizontal_stepper;
// Khai báo hàm
void Stepper_Init(Stepper_HandleTypeDef* stepper, 
                  TIM_HandleTypeDef* timer,
                  uint32_t channel,
                  GPIO_TypeDef* EN_PORT,
                  uint16_t EN_PIN,
                  GPIO_TypeDef* DIR_PORT,
                  uint16_t DIR_PIN,
									StepperType type);

void Stepper_Enable(Stepper_HandleTypeDef* stepper);
void Stepper_Disable(Stepper_HandleTypeDef* stepper);
void Stepper_SetDirection(Stepper_HandleTypeDef* stepper, uint8_t direction);
void Stepper_SetSpeed(Stepper_HandleTypeDef* stepper, uint32_t speed);
void Stepper_Move(Stepper_HandleTypeDef* stepper, int32_t steps);
void Stepper_MoveToAngle(Stepper_HandleTypeDef* stepper, float angle);
#endif // STEPPER_H


