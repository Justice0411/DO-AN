//// stepper.c
#include "stepper.h"
#include "stdlib.h"

// G�c gi?i h?n cho d?ng co d?ng
#define VERTICAL_MIN_ANGLE 0.0f
#define VERTICAL_MAX_ANGLE 90.0f
// G�c gi?i h?n cho d?ng co ngang
#define HORIZONTAL_MIN_ANGLE -90.0f
#define HORIZONTAL_MAX_ANGLE 90.0f

// C?u tr�c d? qu?n l� tr?ng th�i d?ng co // moi 
typedef struct {
    Stepper_HandleTypeDef* stepper;
    uint32_t total_steps;
    uint32_t current_step;
    uint8_t direction;
} StepperMovement;

// Con tr? to�n c?c d? qu?n l� di chuy?n
StepperMovement* active_movement = NULL;

void Stepper_Init(Stepper_HandleTypeDef* stepper, 
                  TIM_HandleTypeDef* timer,
                  uint32_t channel,
                  GPIO_TypeDef* EN_PORT,
                  uint16_t EN_PIN,
                  GPIO_TypeDef* DIR_PORT,
                  uint16_t DIR_PIN,
                  StepperType type)
{
    stepper->timer = timer;
    stepper->channel = channel;
    stepper->EN_PORT = EN_PORT;
    stepper->EN_PIN = EN_PIN;
    stepper->DIR_PORT = DIR_PORT;
    stepper->DIR_PIN = DIR_PIN;
    stepper->type = type; 
    stepper->current_position = 0;
    stepper->target_position = 0;

    // Ch?n microstep d?a tr�n lo?i d?ng co
    if (type == HORIZONTAL_STEPPER) {
        stepper->microsteps = HORIZONTAL_STEPPER_MICROSTEPS;
    } else {
        stepper->microsteps = VERTICAL_STEPPER_MICROSTEPS;
    }

    // T�nh to�n bu?c v?i microstep ri�ng
    stepper->steps_per_revolution = STEPS_PER_REVOLUTION * stepper->microsteps * GEAR_RATIO;

    stepper->current_angle = 0.0f;
    stepper->home_position = 0.0f;
    stepper->speed = timer->Init.Period / 2;
    stepper->is_enabled = 0;
    
    // Kh?i t?o PWM
    HAL_TIM_PWM_Start(stepper->timer, stepper->channel);

    // M?c d?nh ? tr?ng th�i v� hi?u h�a
    Stepper_Disable(stepper);
}

void Stepper_Enable(Stepper_HandleTypeDef* stepper)
{
    if (!stepper->is_enabled) {
        HAL_GPIO_WritePin(stepper->EN_PORT, stepper->EN_PIN, GPIO_PIN_RESET);
        stepper->is_enabled = 1;
        // �? tr? nh? d? d?m b?o tr?ng th�i ?n d?nh
        HAL_Delay(5);
    }
}

void Stepper_Disable(Stepper_HandleTypeDef* stepper)
{
    if (stepper->is_enabled) {
        HAL_GPIO_WritePin(stepper->EN_PORT, stepper->EN_PIN, GPIO_PIN_SET);
        stepper->is_enabled = 0;
    }
}

void Stepper_SetDirection(Stepper_HandleTypeDef* stepper, uint8_t direction)
{
    HAL_GPIO_WritePin(stepper->DIR_PORT, stepper->DIR_PIN, 
                      direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
    // �? tr? nh? d? d?m b?o hu?ng ?n d?nh
    HAL_Delay(1);
}

void Stepper_SetSpeed(Stepper_HandleTypeDef* stepper, uint32_t speed)
{
    stepper->speed = speed;
    if(stepper->channel == TIM_CHANNEL_1) {
        __HAL_TIM_SET_COMPARE(stepper->timer, TIM_CHANNEL_1, speed);
    }
    else if(stepper->channel == TIM_CHANNEL_2) {
        __HAL_TIM_SET_COMPARE(stepper->timer, TIM_CHANNEL_2, speed);
    }
}

void Stepper_Move(Stepper_HandleTypeDef* stepper, int32_t steps)
{
    if (steps == 0) return;

    uint8_t direction = steps >= 0;
    steps = abs(steps);

    Stepper_Enable(stepper);
    Stepper_SetDirection(stepper, direction);
    Stepper_SetSpeed(stepper, stepper->speed);

    // C?p nh?t v? tr� theo d�i
    if (direction) {
        stepper->target_position += steps;
    } else {
        stepper->target_position -= steps;
    }

    // T�nh to�n d? tr? d?a tr�n t?c d? v� bu?c
    uint32_t delay_time = (steps * 1000) / stepper->speed;
    HAL_Delay(delay_time);

    stepper->current_position = stepper->target_position;
    Stepper_Disable(stepper);
		//Stepper_HardStop(stepper);
}

void Stepper_MoveToAngle(Stepper_HandleTypeDef* stepper, float angle)
{
    float steps_per_degree = stepper->steps_per_revolution / 360.0f;
    float min_angle, max_angle;
    
    // X�c d?nh g�c gi?i h?n d?a tr�n d?ng co
    if (stepper == &vertical_stepper) {
        min_angle = VERTICAL_MIN_ANGLE;
        max_angle = VERTICAL_MAX_ANGLE;
    } else if (stepper == &horizontal_stepper) {
        min_angle = HORIZONTAL_MIN_ANGLE;
        max_angle = HORIZONTAL_MAX_ANGLE;
    } else {
        // Tru?ng h?p kh�ng x�c d?nh, kh�ng di chuy?n
        return;
    }

    // Ki?m tra g�c n?m trong gi?i h?n
    if (angle < min_angle || angle > max_angle) {
        return;  // Kh�ng di chuy?n n?u g�c ngo�i gi?i h?n
    }

    int32_t target_steps = (int32_t)((angle - stepper->current_angle) * steps_per_degree);

    if (target_steps != 0) {
        Stepper_Move(stepper, target_steps);
			//Stepper_HardStop(stepper);
        stepper->current_angle = angle;
    }
		//Stepper_HardStop(stepper);
}

