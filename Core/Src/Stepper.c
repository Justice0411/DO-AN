// stepper.c
#include "stepper.h"
#include "stdlib.h"
// Góc gi?i h?n cho d?ng co d?c
#define VERTICAL_MIN_ANGLE 0.0f
#define VERTICAL_MAX_ANGLE 90.0f

// Góc gi?i h?n cho d?ng co ngang
#define HORIZONTAL_MIN_ANGLE -90.0f
#define HORIZONTAL_MAX_ANGLE 90.0f
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
		stepper->type = type; //moi
    stepper->current_position = 0;
    stepper->target_position = 0;
    stepper->steps_per_revolution = STEPS_PER_REVOLUTION * MICROSTEPS * GEAR_RATIO;
    stepper->current_angle = 0.0f;
    stepper->speed = timer->Init.Period / 2;  // Default 50% duty cycle
    stepper->is_enabled = 0;
		
	// Thi?t l?p v? trí ban d?u theo lo?i d?ng co
    if (type == HORIZONTAL_STEPPER) {
        stepper->current_angle = -90.0f;    // B?t d?u t? hu?ng Ðông
        stepper->home_position = -90.0f;
    } else {
        stepper->current_angle = 0.0f;     // B?t d?u ? góc nghiêng 45 d?
        stepper->home_position = 0.0f;
    }
    
    // Initialize PWM
    HAL_TIM_PWM_Start(stepper->timer, stepper->channel);

    // Default to disabled state
    Stepper_Disable(stepper);
}
void Stepper_Enable(Stepper_HandleTypeDef* stepper)
{
    if (!stepper->is_enabled) {
        HAL_GPIO_WritePin(stepper->EN_PORT, stepper->EN_PIN, GPIO_PIN_RESET);
        stepper->is_enabled = 1;
        // Small delay to ensure stable enable
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
    // Small delay to ensure direction is stable
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

    // Update position tracking
    if (direction) {
        stepper->target_position += steps;
    } else {
        stepper->target_position -= steps;
    }

    // Calculate delay based on speed and steps
    uint32_t delay_time = (steps * 1000) / stepper->speed;
    HAL_Delay(delay_time);

    stepper->current_position = stepper->target_position;
    Stepper_Disable(stepper);
}
void Stepper_MoveToAngle(Stepper_HandleTypeDef* stepper, float angle)
{
    float steps_per_degree = stepper->steps_per_revolution / 360.0f;
		float min_angle, max_angle;
    // Xác d?nh góc gi?i h?n d?a trên d?ng co
    if (stepper == &vertical_stepper) {
        min_angle = VERTICAL_MIN_ANGLE;
        max_angle = VERTICAL_MAX_ANGLE;
    } else if (stepper == &horizontal_stepper) {
        min_angle = HORIZONTAL_MIN_ANGLE;
        max_angle = HORIZONTAL_MAX_ANGLE;
    } else {
        // Tru?ng h?p không xác d?nh, không di chuy?n
        return;
    }

    // Ki?m tra góc n?m trong gi?i h?n
    if (angle < min_angle || angle > max_angle) {
        return;  // Không di chuy?n n?u góc ngoài gi?i h?n
    }

		int32_t target_steps = (int32_t)((angle - stepper->current_angle) * steps_per_degree);

    if (target_steps != 0) {
        Stepper_Move(stepper, target_steps);
        stepper->current_angle = angle;
    }
}
