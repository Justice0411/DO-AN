
//#include "main.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "timers.h"
//#include "queue.h"
//#include "event_groups.h"
//#include "TCA.h"
//#include "BH1750.h" 
//#include "string.h"
//#include "stepper.h"
//#include "stdio.h"
//#include "math.h"
//#include "stdlib.h"
//#include "peripherals.h"
//#include "INA219.h"

///* USER CODE BEGIN PM */
//#define QUEUE_LENGTH 5  // Tang kích thu?c queue
//#define LIGHT_DIFF_THRESHOLD 5.0f
//#define VERTICAL_MIN_ANGLE 0.0f
//#define VERTICAL_MAX_ANGLE 90.0f
//#define HORIZONTAL_MIN_ANGLE -90.0f
//#define HORIZONTAL_MAX_ANGLE 90.0f
//#define STACK_SIZE 128  // Tang stack size
///* USER CODE END PM */

//Stepper_HandleTypeDef vertical_stepper, horizontal_stepper;

///* Queue Handles */
//QueueHandle_t xStepperQueue;
//QueueHandle_t xManualControlQueue;
//QueueHandle_t xModeQueue;

///* USER CODE BEGIN PV */
//typedef struct {
//    float top_left;
//    float top_right;
//    float bottom_left;
//    float bottom_right;
//} LightValues;

//typedef struct {
//    float vertical_angle;
//    float horizontal_angle;
//} StepperAngles;

//typedef enum {
//    MODE_AUTO,
//    MODE_MANUAL
//} OperationMode;

//typedef enum {
//    CMD_HORIZONTAL_RIGHT,
//    CMD_HORIZONTAL_LEFT,
//    CMD_VERTICAL_UP,
//    CMD_VERTICAL_DOWN
//} ManualCommand;
///* USER CODE END PV */

///* Private function prototypes */
//void SystemClock_Config(void);
//void MX_GPIO_Init(void);
//void MX_I2C1_Init(void);
//void MX_I2C2_Init(void);
//void MX_TIM1_Init(void);
//void MX_USART2_UART_Init(void);

///* Function Prototypes */
//void ReadSensorsTask(void *pvParameters);
//void ControlSteppersTask(void *pvParameters);
//void ManualControlTask(void *pvParameters);
//void Read_INA219(void *p);

///* USER CODE BEGIN PFP */
//volatile OperationMode current_mode = MODE_AUTO;
///* USER CODE END PFP */

//// Interrupt handler for buttons
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { 
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    ManualCommand command;
//    
//    if (GPIO_Pin == GPIO_PIN_0) {
//        if (current_mode == MODE_AUTO) {
//            current_mode = MODE_MANUAL;
//        } else {
//            current_mode = MODE_AUTO;
//        }
//        xQueueSendFromISR(xModeQueue, (const void*)&current_mode, &xHigherPriorityTaskWoken);
//    }
//    
//    if (current_mode == MODE_MANUAL) {
//        switch(GPIO_Pin) {
//            case GPIO_PIN_1:
//                command = CMD_HORIZONTAL_RIGHT;
//                xQueueSendFromISR(xManualControlQueue, &command, &xHigherPriorityTaskWoken);
//                break;
//            case GPIO_PIN_4:
//                command = CMD_HORIZONTAL_LEFT;
//                xQueueSendFromISR(xManualControlQueue, &command, &xHigherPriorityTaskWoken);
//                break;
//            case GPIO_PIN_5:
//                command = CMD_VERTICAL_UP;
//                xQueueSendFromISR(xManualControlQueue, &command, &xHigherPriorityTaskWoken);
//                break;
//            case GPIO_PIN_12:
//                command = CMD_VERTICAL_DOWN;
//                xQueueSendFromISR(xManualControlQueue, &command, &xHigherPriorityTaskWoken);
//                break;
//        }
//    }
//    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//}

//void ManualControlTask(void *pvParameters) {
//    ManualCommand command;
//    const float MANUAL_STEP_ANGLE = 5.0f;
//    const TickType_t xDelay = pdMS_TO_TICKS(50); // Thêm delay ng?n
//    
//    while(1) {
//        if (xQueueReceive(xManualControlQueue, &command, portMAX_DELAY) == pdPASS) {
//            if (current_mode == MODE_MANUAL) {
//                switch(command) {
//                    case CMD_HORIZONTAL_RIGHT:
//                        Stepper_MoveToAngle(&horizontal_stepper, 
//                            fminf(horizontal_stepper.current_angle + MANUAL_STEP_ANGLE, HORIZONTAL_MAX_ANGLE));
//                        break;
//                    case CMD_HORIZONTAL_LEFT:
//                        Stepper_MoveToAngle(&horizontal_stepper, 
//                            fmaxf(horizontal_stepper.current_angle - MANUAL_STEP_ANGLE, HORIZONTAL_MIN_ANGLE));
//                        break;
//                    case CMD_VERTICAL_UP:
//                        Stepper_MoveToAngle(&vertical_stepper, 
//                            fminf(vertical_stepper.current_angle + MANUAL_STEP_ANGLE, VERTICAL_MAX_ANGLE));
//                        break;
//                    case CMD_VERTICAL_DOWN:
//                        Stepper_MoveToAngle(&vertical_stepper, 
//                            fmaxf(vertical_stepper.current_angle - MANUAL_STEP_ANGLE, VERTICAL_MIN_ANGLE));
//                        break;
//                }
//                vTaskDelay(xDelay); // Cho phép các task khác ch?y
//            }
//        }
//    }
//}

//void ReadSensorsTask(void *pvParameters) {
//    LightValues sensors;
//    StepperAngles angles;
//    float current_vertical = 90.0f;
//    float current_horizontal = 0.0f;
//    const TickType_t xQueueTimeout = pdMS_TO_TICKS(100); // timeout 100ms
//    
//    while(1) {
//        if (current_mode == MODE_AUTO) {
//            TCA9548_SelectChannel(&hi2c1, 4);
//            sensors.top_left = BH1750_ReadLight(&hi2c1);
//            
//            TCA9548_SelectChannel(&hi2c1, 7);
//            sensors.top_right = BH1750_ReadLight(&hi2c1);
//            
//            TCA9548_SelectChannel(&hi2c1, 5);
//            sensors.bottom_left = BH1750_ReadLight(&hi2c1);
//            
//            TCA9548_SelectChannel(&hi2c1, 6);
//            sensors.bottom_right = BH1750_ReadLight(&hi2c1);

//            float top_avg = (sensors.top_left + sensors.top_right) / 2.0f;
//            float bottom_avg = (sensors.bottom_left + sensors.bottom_right) / 2.0f;
//            float left_avg = (sensors.top_left + sensors.bottom_left) / 2.0f;
//            float right_avg = (sensors.top_right + sensors.bottom_right) / 2.0f;
//            
//            float vertical_diff = bottom_avg - top_avg;
//            float horizontal_diff = right_avg - left_avg;

//            if (fabsf(vertical_diff) > LIGHT_DIFF_THRESHOLD) {
//                float adjustment_angle = fminf(fabsf(vertical_diff) / 10.0f, 5.0f);
//                float new_vertical = current_vertical;
//                
//                if (vertical_diff > 0 && current_vertical < VERTICAL_MAX_ANGLE) {
//                    new_vertical += adjustment_angle;
//                } else if (vertical_diff < 0 && current_vertical > VERTICAL_MIN_ANGLE) {
//                    new_vertical -= adjustment_angle;
//                }
//                
//                if (new_vertical > VERTICAL_MIN_ANGLE && new_vertical < VERTICAL_MAX_ANGLE) {
//                    angles.vertical_angle = new_vertical;
//                    current_vertical = new_vertical;
//                }
//            }

//            if (fabsf(horizontal_diff) > LIGHT_DIFF_THRESHOLD) {
//                float adjustment_angle = fminf(fabsf(horizontal_diff) / 10.0f, 5.0f);
//                float new_horizontal = current_horizontal;
//                
//                if (horizontal_diff > 0 && current_horizontal < HORIZONTAL_MAX_ANGLE) {
//                    new_horizontal += adjustment_angle;
//                } else if (horizontal_diff < 0 && current_horizontal > HORIZONTAL_MIN_ANGLE) {
//                    new_horizontal -= adjustment_angle;
//                }
//                
//                if (new_horizontal >= HORIZONTAL_MIN_ANGLE && new_horizontal <= HORIZONTAL_MAX_ANGLE) {
//                    angles.horizontal_angle = new_horizontal;
//                    current_horizontal = new_horizontal;
//                }
//            }
//            
//            // S? d?ng timeout thay vì ch? vô h?n
//            xQueueSend(xStepperQueue, &angles, xQueueTimeout);
//        }
//        vTaskDelay(pdMS_TO_TICKS(1000));
//    }
//}

//void ControlSteppersTask(void *pvParameters) {
//    StepperAngles angles;
//    const TickType_t xDelay = pdMS_TO_TICKS(50); // Thêm delay ng?n
//    
//    while(1) {
//        if(xQueueReceive(xStepperQueue, &angles, portMAX_DELAY) == pdPASS) {
//            Stepper_MoveToAngle(&vertical_stepper, angles.vertical_angle);
//            Stepper_MoveToAngle(&horizontal_stepper, angles.horizontal_angle);
//            vTaskDelay(xDelay); // Cho phép các task khác ch?y
//        }
//    }
//}

////void Read_INA219(void *p){
////	char buffer2[100];
////	const TickType_t xDelay = pdMS_TO_TICKS(1000); // 1 second delay
////	while(1){
////				float bus_voltage = INA219_ReadVoltage(&hi2c2);
////        float shunt_current = INA219_ReadCurrent(&hi2c2);
////        float power = bus_voltage * shunt_current ;
////        
////        sprintf(buffer2, "V: %.2f V, V: %.3f A, P: %.3f W\r\n", 
////                bus_voltage, shunt_current, power);
////					HAL_UART_Transmit(&huart2, (uint8_t*)buffer2, strlen(buffer2), 100);
////        vTaskDelay(xDelay);
////	}
////}
//void Read_BH1750(void *p) {
//    char buffer1[100];
//		char buffer2[100];
//    const TickType_t xDelay = pdMS_TO_TICKS(1000); // 1 second delay
//    LightValues data;
//    while(1) {
//						TCA9548_SelectChannel(&hi2c1, 4);
//           data.top_left = BH1750_ReadLight(&hi2c1);
//            
//            TCA9548_SelectChannel(&hi2c1, 7);
//            data.top_right = BH1750_ReadLight(&hi2c1);
//            
//            TCA9548_SelectChannel(&hi2c1, 5);
//            data.bottom_left = BH1750_ReadLight(&hi2c1);
//            
//            TCA9548_SelectChannel(&hi2c1, 6);
//            data.bottom_right = BH1750_ReadLight(&hi2c1);
//            sprintf(buffer1, "%.1f;%.1f;%.1f;%.1f\n",                 
//                    data.top_left,
//                    data.top_right,
//                    data.bottom_left,
//                    data.bottom_right);
//            
//						HAL_UART_Transmit(&huart2, (uint8_t*)buffer1, strlen(buffer1),100);
//			vTaskDelay(pdMS_TO_TICKS(50));
//        float bus_voltage = INA219_ReadVoltage(&hi2c2);
//        float shunt_current = INA219_ReadCurrent(&hi2c2);
//        float power = bus_voltage * shunt_current ;
//        
//        sprintf(buffer2, "V: %.2f V, V: %.3f A, P: %.3f W\r\n", 
//                bus_voltage, shunt_current, power);
//					HAL_UART_Transmit(&huart2, (uint8_t*)buffer2, strlen(buffer2), 100);
//        
//        vTaskDelay(xDelay); // S? d?ng vTaskDelay thay vì HAL_Delay
//    }
//}



//int main(void) {
//    HAL_Init();
//    SystemClock_Config();
//    
//    MX_GPIO_Init();
//    MX_I2C1_Init();
//    MX_I2C2_Init();
//    MX_TIM1_Init();
//    MX_USART2_UART_Init();
//    INA219_Init(&hi2c2);
//    
//    Stepper_Init(&vertical_stepper, &htim1, TIM_CHANNEL_1,
//                 VERTICAL_STEPPER_EN_PORT, VERTICAL_STEPPER_EN_PIN,
//                 VERTICAL_STEPPER_DIR_PORT, VERTICAL_STEPPER_DIR_PIN,
//                 VERTICAL_STEPPER);
//                 
//    Stepper_Init(&horizontal_stepper, &htim1, TIM_CHANNEL_2,
//                 HORIZONTAL_STEPPER_EN_PORT, HORIZONTAL_STEPPER_EN_PIN,
//                 HORIZONTAL_STEPPER_DIR_PORT, HORIZONTAL_STEPPER_DIR_PIN,
//                 HORIZONTAL_STEPPER);

//    // Create queues with increased sizes
//    xManualControlQueue = xQueueCreate(5, sizeof(ManualCommand));
//    xStepperQueue = xQueueCreate(QUEUE_LENGTH, sizeof(StepperAngles));
//    xModeQueue = xQueueCreate(1, sizeof(OperationMode));

//    // Create tasks with increased stack sizes and appropriate priorities
//    xTaskCreate(ReadSensorsTask, "Sensors", STACK_SIZE, NULL, 2, NULL);
//    xTaskCreate(ControlSteppersTask, "Steppers", STACK_SIZE, NULL, 1, NULL);
//    xTaskCreate(ManualControlTask, "ManualControl", STACK_SIZE, NULL, 3, NULL);
//		//xTaskCreate(Read_INA219, "INA219", STACK_SIZE, NULL, 2, NULL);
//		xTaskCreate(Read_BH1750, "BH1750", STACK_SIZE, NULL, 2, NULL);
//    

//    vTaskStartScheduler();

//    while(1) {
//        // Should never get here
//    }
//}

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
#include "INA219.h"

/* Defines */
#define QUEUE_LENGTH 5
#define LIGHT_DIFF_THRESHOLD 5.0f
#define VERTICAL_MIN_ANGLE 0.0f
#define VERTICAL_MAX_ANGLE 90.0f
#define HORIZONTAL_MIN_ANGLE -90.0f
#define HORIZONTAL_MAX_ANGLE 90.0f
#define STACK_SIZE 128

/* Handles */
Stepper_HandleTypeDef vertical_stepper, horizontal_stepper;
QueueHandle_t xStepperQueue;
QueueHandle_t xManualControlQueue;
QueueHandle_t xModeQueue;
QueueHandle_t xSensorDataQueue;

/* Type Definitions */
typedef struct {
    float top_left;
    float top_right;
    float bottom_left;
    float bottom_right;
    float light_values[4];
} LightValues;

typedef struct {
    float vertical_angle;
    float horizontal_angle;
} StepperAngles;

typedef enum {
    MODE_AUTO,
    MODE_MANUAL
} OperationMode;

typedef struct {
    float bus_voltage;
    float shunt_current;
    float power;
    float light_values[4];
} SensorDatas;

typedef enum {
    CMD_HORIZONTAL_RIGHT,
    CMD_HORIZONTAL_LEFT,
    CMD_VERTICAL_UP,
    CMD_VERTICAL_DOWN
} ManualCommand;

/* Function Prototypes */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
void MX_TIM1_Init(void);
void MX_USART2_UART_Init(void);

void ReadSensorsTask(void *pvParameters);
void ControlSteppersTask(void *pvParameters);
void ManualControlTask(void *pvParameters);
void Read_INA219(void *p);
void Process_INA219_Data(void *pvParameters);

volatile OperationMode current_mode = MODE_AUTO;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ManualCommand command;
    
    if (GPIO_Pin == GPIO_PIN_0) {
        current_mode = (current_mode == MODE_AUTO) ? MODE_MANUAL : MODE_AUTO;
        xQueueSendFromISR(xModeQueue, (const void*)&current_mode, &xHigherPriorityTaskWoken);
    }
    
    if (current_mode == MODE_MANUAL) {
        switch(GPIO_Pin) {
            case GPIO_PIN_1:
                command = CMD_HORIZONTAL_RIGHT;
                xQueueSendFromISR(xManualControlQueue, &command, &xHigherPriorityTaskWoken);
                break;
            case GPIO_PIN_4:
                command = CMD_HORIZONTAL_LEFT;
                xQueueSendFromISR(xManualControlQueue, &command, &xHigherPriorityTaskWoken);
                break;
            case GPIO_PIN_5:
                command = CMD_VERTICAL_UP;
                xQueueSendFromISR(xManualControlQueue, &command, &xHigherPriorityTaskWoken);
                break;
            case GPIO_PIN_12:
                command = CMD_VERTICAL_DOWN;
                xQueueSendFromISR(xManualControlQueue, &command, &xHigherPriorityTaskWoken);
                break;
        }
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ManualControlTask(void *pvParameters) {
    ManualCommand command;
    const float MANUAL_STEP_ANGLE = 5.0f;
    const TickType_t xDelay = pdMS_TO_TICKS(50);
    
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
                vTaskDelay(xDelay);
            }
        }
    }
}

void ReadSensorsTask(void *pvParameters) {
    LightValues sensors;
    StepperAngles angles;
    SensorDatas data;
    float current_vertical = 90.0f;
    float current_horizontal = 0.0f;
    const TickType_t xQueueTimeout = pdMS_TO_TICKS(100);
    char buffer[200];
    
    while(1) {
        // Ð?c giá tr? c?m bi?n
        TCA9548_SelectChannel(&hi2c1, 4);
        data.light_values[0] = BH1750_ReadLight(&hi2c1);
        sensors.top_left = data.light_values[0];
        
        TCA9548_SelectChannel(&hi2c1, 7);
        data.light_values[1] = BH1750_ReadLight(&hi2c1);
        sensors.top_right = data.light_values[1];
        
        TCA9548_SelectChannel(&hi2c1, 5);
        data.light_values[2] = BH1750_ReadLight(&hi2c1);
        sensors.bottom_left = data.light_values[2];
        
        TCA9548_SelectChannel(&hi2c1, 6);
        data.light_values[3] = BH1750_ReadLight(&hi2c1);
        sensors.bottom_right = data.light_values[3];

        // G?i d? li?u d? hi?n th?
        sprintf(buffer, "%.1f;%.1f;%.1f;%.1f\n",
                data.light_values[0],
                data.light_values[1],
                data.light_values[2],
                data.light_values[3]);
        
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

        // X? lý di?u khi?n t? d?ng n?u ? ch? d? AUTO
        if (current_mode == MODE_AUTO) {
            float top_avg = (sensors.top_left + sensors.top_right) / 2.0f;
            float bottom_avg = (sensors.bottom_left + sensors.bottom_right) / 2.0f;
            float left_avg = (sensors.top_left + sensors.bottom_left) / 2.0f;
            float right_avg = (sensors.top_right + sensors.bottom_right) / 2.0f;
            
            float vertical_diff = bottom_avg - top_avg;
            float horizontal_diff = right_avg - left_avg;

            if (fabsf(vertical_diff) > LIGHT_DIFF_THRESHOLD) {
                float adjustment_angle = fminf(fabsf(vertical_diff) / 10.0f, 5.0f);
                float new_vertical = current_vertical;
                
                if (vertical_diff > 0 && current_vertical < VERTICAL_MAX_ANGLE) {
                    new_vertical += adjustment_angle;
                } else if (vertical_diff < 0 && current_vertical > VERTICAL_MIN_ANGLE) {
                    new_vertical -= adjustment_angle;
                }
                
                if (new_vertical > VERTICAL_MIN_ANGLE && new_vertical < VERTICAL_MAX_ANGLE) {
                    angles.vertical_angle = new_vertical;
                    current_vertical = new_vertical;
                }
            }

            if (fabsf(horizontal_diff) > LIGHT_DIFF_THRESHOLD) {
                float adjustment_angle = fminf(fabsf(horizontal_diff) / 10.0f, 5.0f);
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
            
            xQueueSend(xStepperQueue, &angles, xQueueTimeout);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void ControlSteppersTask(void *pvParameters) {
    StepperAngles angles;
    const TickType_t xDelay = pdMS_TO_TICKS(50);
    
    while(1) {
        if(xQueueReceive(xStepperQueue, &angles, portMAX_DELAY) == pdPASS) {
            Stepper_MoveToAngle(&vertical_stepper, angles.vertical_angle);
            Stepper_MoveToAngle(&horizontal_stepper, angles.horizontal_angle);
            vTaskDelay(xDelay);
        }
    }
}



void Read_INA219(void *p) {
    char buffer[100];
    const TickType_t xDelay = pdMS_TO_TICKS(1000); // 1 second delay
    
    while(1) {
        float bus_voltage = INA219_ReadVoltage(&hi2c2);
        float shunt_current = INA219_ReadCurrent(&hi2c2);
        float power = bus_voltage * shunt_current ;
        
        sprintf(buffer, "Bus Voltage: %.2f V, Ampe: %.3f A, Power: %.3f W\r\n", 
                bus_voltage, shunt_current, power);
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        
        vTaskDelay(xDelay); // S? d?ng vTaskDelay thay vì HAL_Delay
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_TIM1_Init();
    MX_USART2_UART_Init();
    INA219_Init(&hi2c2);
    
    Stepper_Init(&vertical_stepper, &htim1, TIM_CHANNEL_1,
                 VERTICAL_STEPPER_EN_PORT, VERTICAL_STEPPER_EN_PIN,
                 VERTICAL_STEPPER_DIR_PORT, VERTICAL_STEPPER_DIR_PIN,
                 VERTICAL_STEPPER);
                 
    Stepper_Init(&horizontal_stepper, &htim1, TIM_CHANNEL_2,
                 HORIZONTAL_STEPPER_EN_PORT, HORIZONTAL_STEPPER_EN_PIN,
                 HORIZONTAL_STEPPER_DIR_PORT, HORIZONTAL_STEPPER_DIR_PIN,
                 HORIZONTAL_STEPPER);

    xManualControlQueue = xQueueCreate(QUEUE_LENGTH, sizeof(ManualCommand));
    xStepperQueue = xQueueCreate(QUEUE_LENGTH, sizeof(StepperAngles));
    xModeQueue = xQueueCreate(1, sizeof(OperationMode));
    xSensorDataQueue = xQueueCreate(QUEUE_LENGTH, sizeof(SensorDatas));

    if (xManualControlQueue == NULL || xStepperQueue == NULL || 
        xModeQueue == NULL || xSensorDataQueue == NULL) {
        Error_Handler();
    }

    xTaskCreate(ReadSensorsTask, "Sensors", STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(ControlSteppersTask, "Steppers", STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(ManualControlTask, "ManualControl", STACK_SIZE, NULL, 3, NULL);
    xTaskCreate(Read_INA219, "INA219", STACK_SIZE, NULL, 1, NULL);
    //xTaskCreate(Process_INA219_Data, "INA219_Process", STACK_SIZE, NULL, 1, NULL);

    vTaskStartScheduler();

    while(1) {
        // Should never get here
    }
}
