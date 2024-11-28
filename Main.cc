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
//#define ANGLE_THRESHOLD 0.5f  // Ngu?ng chênh l?ch góc t?i thi?u d? di chuy?n
#define LIGHT_DIFF_THRESHOLD 5.0f  // Ngu?ng chênh l?ch ánh sáng t?i thi?u
#define VERTICAL_MIN_ANGLE 0.0f  // Góc t?i thi?u cho d?ng co d?c
#define VERTICAL_MAX_ANGLE 90.0f   // Góc t?i da cho d?ng co d?c
#define HORIZONTAL_MIN_ANGLE -90.0f  // Góc t?i thi?u cho d?ng co ngang
#define HORIZONTAL_MAX_ANGLE  90.0f   // Góc t?i da cho d?ng co ngang
#define STEP_ANGLE 20.0f  // Góc di chuy?n m?i l?n di?u ch?nhTaskHandle_t xSensorTaskHandle;


/* USER CODE END PM */
Stepper_HandleTypeDef vertical_stepper, horizontal_stepper;
QueueHandle_t xStepperQueue;

/* Queue Handles */
QueueHandle_t xReadSensorsTaskHandle;
QueueHandle_t xControlSteppersTaskHandle;

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

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

int main(void)
{
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

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
		// tao queue
    xStepperQueue = xQueueCreate(QUEUE_LENGTH, sizeof(StepperAngles));
    xTaskCreate(ReadSensorsTask, "Sensors", 128, NULL, 2, NULL);
    xTaskCreate(ControlSteppersTask, "Steppers", 128, NULL, 1, NULL);
    vTaskStartScheduler();
	}
  /* USER CODE END RTOS_QUEUES */

  
  /* USER CODE BEGIN WHILE */
  while (1)
  {}
  /* USER CODE END 3 */
}


void ReadSensorsTask(void *pvParameters) {
    LightValues sensors;
    StepperAngles angles;
    float current_vertical = 0;
    float current_horizontal = -90.0f;
    
    while(1) {
        // Ð?c giá tr? t? 4 c?m bi?n
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

        // G?i góc m?i d?n task di?u khi?n d?ng co
        xQueueSend(xStepperQueue, &angles, portMAX_DELAY);
        
        // Delay tru?c khi d?c l?i c?m bi?n
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ControlSteppersTask(void *pvParameters) {
    StepperAngles angles;
    
    while(1) {
        if(xQueueReceive(xStepperQueue, &angles, portMAX_DELAY) == pdPASS) {
            // Di chuy?n d?ng co tr?c d?ng
            Stepper_MoveToAngle(&vertical_stepper, angles.vertical_angle);
            
            // Di chuy?n d?ng co tr?c ngang
            Stepper_MoveToAngle(&horizontal_stepper, angles.horizontal_angle);
        }
    }
}


