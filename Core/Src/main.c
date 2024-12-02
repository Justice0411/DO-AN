#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "event_groups.h"
#include "TCA.h"
#include "BH1750.h" 
#include "string.h"
#include "stepper.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "peripherals.h"

/* USER CODE BEGIN PM */
#define QUEUE_LENGTH 1
#define LIGHT_DIFF_THRESHOLD 5.0f  // Ngu?ng chênh l?ch ánh sáng t?i thi?u
#define VERTICAL_MIN_ANGLE 0.0f  // Góc t?i thi?u cho d?ng co d?c
#define VERTICAL_MAX_ANGLE 90.0f   // Góc t?i da cho d?ng co d?c
#define HORIZONTAL_MIN_ANGLE -90.0f  // Góc t?i thi?u cho d?ng co ngang
#define HORIZONTAL_MAX_ANGLE  90.0f   // Góc t?i da cho d?ng co ngang
#define STEP_ANGLE 20.0f  // Góc di chuy?n m?i l?n di?u ch?nhTaskHandle_t xSensorTaskHandle;

/* USER CODE END PM */
Stepper_HandleTypeDef vertical_stepper, horizontal_stepper;

/* Queue Handles */
QueueHandle_t xStepperQueue;
QueueHandle_t xManualControlQueue;
QueueHandle_t xModeQueue; //moi 
/* USER CODE BEGIN PV */
typedef struct {
    float top_left;
    float top_right;
    float bottom_left;
    float bottom_right;
} LightValues;

typedef struct {
    float vertical_angle;
    float horizontal_angle;
} StepperAngles;

typedef enum {
    MODE_AUTO,
    MODE_MANUAL
} OperationMode;

typedef enum {
    CMD_HORIZONTAL_RIGHT,
    CMD_HORIZONTAL_LEFT,
    CMD_VERTICAL_UP,
    CMD_VERTICAL_DOWN
} ManualCommand;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
void MX_TIM1_Init(void);
void MX_USART2_UART_Init(void);

/* Function Prototypes */
void ReadSensorsTask(void *pvParameters);
void ControlSteppersTask(void *pvParameters);
void ManualControlTask(void *pvParameters);

/* USER CODE BEGIN PFP */
volatile OperationMode current_mode = MODE_AUTO;
/* USER CODE END PFP */

// Interrupt handler for buttons
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; // k ktra xem co task uu tien cao hon can th thi ko 
    ManualCommand command;
	
		 if (GPIO_Pin == GPIO_PIN_0) {  // Nút chuy?n che do
        if (current_mode == MODE_AUTO) {
            current_mode = MODE_MANUAL;
        } else {
            current_mode = MODE_AUTO;
        }
        
        xQueueSendFromISR(xModeQueue, (const void*)&current_mode, &xHigherPriorityTaskWoken);
    }
		 
		if (current_mode == MODE_MANUAL) 
		{
    switch(GPIO_Pin) 
			{
        case GPIO_PIN_1:  // Horizontal right
                command = CMD_HORIZONTAL_RIGHT;
                xQueueSendFromISR(xManualControlQueue, &command, &xHigherPriorityTaskWoken);
								break;      
        case GPIO_PIN_4:  // Horizontal left         
                command = CMD_HORIZONTAL_LEFT;
                xQueueSendFromISR(xManualControlQueue, &command, &xHigherPriorityTaskWoken);         
								break;        
        case GPIO_PIN_5:  // Vertical up         
                command = CMD_VERTICAL_UP;
                xQueueSendFromISR(xManualControlQueue, &command, &xHigherPriorityTaskWoken);
								break;     
        case GPIO_PIN_12:  // Vertical down        
                command = CMD_VERTICAL_DOWN;
                xQueueSendFromISR(xManualControlQueue, &command, &xHigherPriorityTaskWoken);
								break;
				}
		}
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ManualControlTask(void *pvParameters) {
    ManualCommand command;
    const float MANUAL_STEP_ANGLE = 10.0f;  // Adjust as needed
    while(1) {
			if (xQueueReceive(xManualControlQueue, &command, portMAX_DELAY) == pdPASS) {    
         if (current_mode == MODE_MANUAL) {    
						switch(command) {
                    case CMD_HORIZONTAL_RIGHT:
                        Stepper_MoveToAngle(&horizontal_stepper, 
                            fminf(horizontal_stepper.current_angle + MANUAL_STEP_ANGLE, HORIZONTAL_MAX_ANGLE));
                        break;
                    
                    case CMD_HORIZONTAL_LEFT:
                        Stepper_MoveToAngle(&horizontal_stepper, 
                            fmaxf(horizontal_stepper.current_angle - MANUAL_STEP_ANGLE, HORIZONTAL_MIN_ANGLE));
                        break;
                    
                    case CMD_VERTICAL_UP:
                        Stepper_MoveToAngle(&vertical_stepper, 
                            fminf(vertical_stepper.current_angle + MANUAL_STEP_ANGLE, VERTICAL_MAX_ANGLE));
                        break;
                    
                    case CMD_VERTICAL_DOWN:
                        Stepper_MoveToAngle(&vertical_stepper, 
                            fmaxf(vertical_stepper.current_angle - MANUAL_STEP_ANGLE, VERTICAL_MIN_ANGLE));
                        break;
								}
						}
				}
		}
}
void ReadSensorsTask(void *pvParameters) {
    LightValues sensors;
    StepperAngles angles;
    float current_vertical = 0;
    float current_horizontal = -90.0f;
		//OperationMode currentMode;
    while(1) {
				//xQueuePeek(xModeQueue, &currentMode, portMAX_DELAY);
        if (current_mode == MODE_AUTO) {
        TCA9548_SelectChannel(&hi2c1, 4);
        sensors.top_left = BH1750_ReadLight(&hi2c1);
        
        TCA9548_SelectChannel(&hi2c1, 5);
        sensors.top_right = BH1750_ReadLight(&hi2c1);
        
        TCA9548_SelectChannel(&hi2c1, 6);
        sensors.bottom_left = BH1750_ReadLight(&hi2c1);
        
        TCA9548_SelectChannel(&hi2c1, 7);
        sensors.bottom_right = BH1750_ReadLight(&hi2c1);

        // Tính toán chênh l?ch (gi? nguyên)
        float top_avg = (sensors.top_left + sensors.top_right) / 2.0f;
        float bottom_avg = (sensors.bottom_left + sensors.bottom_right) / 2.0f;
        float left_avg = (sensors.top_left + sensors.bottom_left) / 2.0f;
        float right_avg = (sensors.top_right + sensors.bottom_right) / 2.0f;
        
        float vertical_diff = top_avg - bottom_avg;
        float horizontal_diff = right_avg - left_avg;

        // Ði?u ch?nh góc d?c (Vertical)
        if (fabsf(vertical_diff) > LIGHT_DIFF_THRESHOLD) {
            float adjustment_angle = fminf(fabsf(vertical_diff) / 10.0f, 5.0f); // Gi?m góc t?i da xu?ng 5 d?
            float new_vertical = current_vertical;
            
            if (vertical_diff > 0 && current_vertical < VERTICAL_MAX_ANGLE) {
                new_vertical += adjustment_angle; // note
            } else if (vertical_diff < 0 && current_vertical > VERTICAL_MIN_ANGLE) {
                new_vertical -= adjustment_angle; //note
            }
            
            if (new_vertical > VERTICAL_MIN_ANGLE && new_vertical < VERTICAL_MAX_ANGLE) {
                angles.vertical_angle = new_vertical;
                current_vertical = new_vertical;
            }
        }

        // Ði?u ch?nh góc ngang (Horizontal)
        if (fabsf(horizontal_diff) > LIGHT_DIFF_THRESHOLD) {
            float adjustment_angle = fminf(fabsf(horizontal_diff) / 10.0f, 5.0f); // Gi?m góc t?i da xu?ng 5 d?
            float new_horizontal = current_horizontal;
            
            if (horizontal_diff > 0 && current_horizontal < HORIZONTAL_MAX_ANGLE) {
                new_horizontal += adjustment_angle;
            } else if (horizontal_diff < 0 && current_horizontal > HORIZONTAL_MIN_ANGLE) {
                new_horizontal -= adjustment_angle;
            }
            
            if (new_horizontal >= HORIZONTAL_MIN_ANGLE && new_horizontal <= HORIZONTAL_MAX_ANGLE) {
                angles.horizontal_angle = new_horizontal;
                current_horizontal = new_horizontal;
            }
        }
        xQueueSend(xStepperQueue, &angles, portMAX_DELAY);
			}
        // Delay tru?c khi d?c l?i c?m bi?n
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ControlSteppersTask(void *pvParameters) {
    StepperAngles angles;
    while(1) {
			if(xQueueReceive(xStepperQueue, &angles, portMAX_DELAY) == pdPASS) {
				if (current_mode == MODE_AUTO) {					
            Stepper_MoveToAngle(&vertical_stepper, angles.vertical_angle);        
            Stepper_MoveToAngle(&horizontal_stepper, angles.horizontal_angle);
					}
			}
	}
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	// Kh?i t?o d?ng co bu?c
    Stepper_Init(&vertical_stepper, &htim1, TIM_CHANNEL_1,
                 VERTICAL_STEPPER_EN_PORT, VERTICAL_STEPPER_EN_PIN,
                 VERTICAL_STEPPER_DIR_PORT, VERTICAL_STEPPER_DIR_PIN,VERTICAL_STEPPER);
                 
    Stepper_Init(&horizontal_stepper, &htim1, TIM_CHANNEL_2,
                 HORIZONTAL_STEPPER_EN_PORT, HORIZONTAL_STEPPER_EN_PIN,
                 HORIZONTAL_STEPPER_DIR_PORT, HORIZONTAL_STEPPER_DIR_PIN,HORIZONTAL_STEPPER);
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_QUEUES */
	
	if( xStepperQueue == NULL)
	{

		xManualControlQueue = xQueueCreate(5, sizeof(ManualCommand));
    xStepperQueue = xQueueCreate(QUEUE_LENGTH, sizeof(StepperAngles));
		xModeQueue = xQueueCreate(1, sizeof(OperationMode));
		
    xTaskCreate(ReadSensorsTask, "Sensors", 128, NULL, 2, NULL);
    xTaskCreate(ControlSteppersTask, "Steppers", 128, NULL, 1, NULL);
		xTaskCreate(ManualControlTask, "ManualControl", 128, NULL, 3, NULL);
    vTaskStartScheduler();
	}
  /* USER CODE BEGIN WHILE */
  while (1)
  {}
  /* USER CODE END 3 */
}


