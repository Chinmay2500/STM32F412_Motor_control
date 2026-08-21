	/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "motor.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim1; // ENC1
extern TIM_HandleTypeDef htim2; // PWM
extern TIM_HandleTypeDef htim3; // ENC2
extern TIM_HandleTypeDef htim5; // ENC3 (Motor 4)

Motor_t motors[NUM_MOTORS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Blocking function to move a single motor to a target position with a timeout
void Motor_MoveTo(Motor_t *m, int32_t target_ticks) {
    m->target_pos = target_ticks;
    uint32_t start_time = HAL_GetTick();

    while (!Motor_UpdatePositionPID(m)) {
        HAL_Delay(10);

        // SAFETY 1: Runaway Protection (If error grows 50 ticks beyond initial target, KILL POWER)
        int32_t current_error = m->target_pos - m->encoder_pos;
        if (abs(current_error) > abs(target_ticks) + 50) {
            Motor_SetRawOutput(m, 0); // Emergency stop
            while (1); // Halt execution to protect hardware
        }

        // SAFETY 2: Shortened 1.5-second timeout
        if (HAL_GetTick() - start_time > 1500) {
            Motor_SetRawOutput(m, 0);
            break;
        }
    }
    Motor_SetRawOutput(m, 0);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  // --- Motor 1 (P3 + P1) ---
  motors[0].htim_pwm    = &htim2;
  motors[0].pwm_channel = TIM_CHANNEL_1;
  motors[0].dir_port    = GPIOA;
  motors[0].dir_pin     = GPIO_PIN_7;
  motors[0].invert_dir  = false;
  motors[0].htim_enc    = &htim1;
  motors[0].invert_enc  = true;
  motors[0].kp          = 4.0f;
  motors[0].kd          = 0.8f;

  // --- Motor 2 (P6 + P2) ---
  motors[1].htim_pwm    = &htim2;
  motors[1].pwm_channel = TIM_CHANNEL_2;
  motors[1].dir_port    = GPIOC;
  motors[1].dir_pin     = GPIO_PIN_4;
  motors[1].invert_dir  = false;
  motors[1].htim_enc    = &htim3;
  motors[1].invert_enc  = true;
  motors[1].kp          = 4.0f;
  motors[1].kd          = 0.8f;

  // --- Motor 4 (P12 + P4) ---
  motors[2].htim_pwm    = &htim2;
  motors[2].pwm_channel = TIM_CHANNEL_4;
  motors[2].dir_port    = GPIOA;
  motors[2].dir_pin     = GPIO_PIN_4;
  motors[2].invert_dir  = false;
  motors[2].htim_enc    = &htim5;
  motors[2].invert_enc  = true;
  motors[2].kp          = 4.0f;
  motors[2].kd          = 0.8f;

  Motors_Init(motors);

  // Target tick count for 90 degrees (start with 400 as a test reference)
  int32_t target_ticks = 100;
  Motor_MoveTo(&motors[0],target_ticks);
  HAL_Delay(5000);
  Motor_MoveTo(&motors[1],target_ticks);
  HAL_Delay(5000);
  Motor_MoveTo(&motors[2],target_ticks);
  HAL_Delay(5000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  Motor_UpdatePositionPID(&motors[0]);
	  Motor_UpdatePositionPID(&motors[1]);
	  Motor_UpdatePositionPID(&motors[2]);

	  HAL_Delay(10);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
