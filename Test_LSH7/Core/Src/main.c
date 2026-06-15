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
#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_tim.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_DIR_TIM_PWM htim1
#define MOTOR1_TIM_PWM htim2
#define MOTOR3_TIM_PWM htim2
#define MOTOR2_TIM_PWM htim15
#define DIR_GPIO GPIOG

#define ENCODERS_STEER_TIM htim3

#define ENCODERS_DRIVE_MOTOR1_TIM htim4
#define ENCODERS_DRIVE_MOTOR3_TIM htim4
#define ENCODERS_DRIVE_MOTOR2_TIM htim5
#define ENCODERS_STEER_TIM htim3
/*########################################*/
/*MOTOR TYPE DRIVE*/
/*########################################*/
#define MOTOR1_IZQ_PWM TIM_CHANNEL_1
#define MOTOR1_DER_PWM TIM_CHANNEL_2
#define MOTOR2_IZQ_PWM TIM_CHANNEL_1
#define MOTOR2_DER_PWM TIM_CHANNEL_2
#define MOTOR3_IZQ_PWM TIM_CHANNEL_4
#define MOTOR3_DER_PWM TIM_CHANNEL_3

#define DIR_IZQ_DRIVE_PIN_INA GPIO_PIN_8
#define DIR_IZQ_DRIVE_PIN_INB GPIO_PIN_9
#define DIR_DER_DRIVE_PIN_INA GPIO_PIN_10
#define DIR_DER_DRIVE_PIN_INB GPIO_PIN_11

/*########################################*/
/*MOTOR TYPE STEER*/
/*########################################*/
#define MOTOR_DELANTERO_IZQ_PWM TIM_CHANNEL_1
#define MOTOR_DELANTERO_DER_PWM TIM_CHANNEL_2
#define MOTOR_TRASERO_IZQ_PWM TIM_CHANNEL_3
#define MOTOR_TRASERO_DER_PWM TIM_CHANNEL_4

#define DIR_A_TRASERO_IZQ_PIN GPIO_PIN_0
#define DIR_B_TRASERO_IZQ_PIN GPIO_PIN_1
#define DIR_A_TRASERO_DER_PIN GPIO_PIN_2
#define DIR_B_TRASERO_DER_PIN GPIO_PIN_3
#define DIR_A_DELANTERO_IZQ_PIN GPIO_PIN_4
#define DIR_B_DELANTERO_IZQ_PIN GPIO_PIN_5
#define DIR_A_DELANTERO_DER_PIN GPIO_PIN_6
#define DIR_B_DELANTERO_DER_PIN GPIO_PIN_7
/*########################################*/
/*ENCODERS */
/*########################################*/
#define A_MOTOR1_IZQ HAL_TIM_ACTIVE_CHANNEL_1
#define A_MOTOR3_IZQ HAL_TIM_ACTIVE_CHANNEL_2
#define A_MOTOR1_DER HAL_TIM_ACTIVE_CHANNEL_3
#define A_MOTOR3_DER HAL_TIM_ACTIVE_CHANNEL_4

#define A_MOTOR2_IZQ HAL_TIM_ACTIVE_CHANNEL_1
#define A_MOTOR2_DER HAL_TIM_ACTIVE_CHANNEL_2

#define ENC_A_TRASERO_IZQ HAL_TIM_ACTIVE_CHANNEL_1
#define ENC_A_TRASERO_DER HAL_TIM_ACTIVE_CHANNEL_2
#define ENC_A_DELANTERO_IZQ HAL_TIM_ACTIVE_CHANNEL_3
#define ENC_A_DELANTERO_DER HAL_TIM_ACTIVE_CHANNEL_4

#define BUSCAR 0
#define BUSCAR_ZERO 1
#define BUSCAR_FINAL 2
#define CENTRAR 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

typedef struct {
  TIM_HandleTypeDef *pwm_tim;
  uint16_t pwm_channel;
  int active_channel;
  TIM_HandleTypeDef *enc_tim;
  GPIO_TypeDef *gpio_dir_port;
  uint16_t dir_A;
  uint16_t dir_B;
  int angulo_centro;
  int max_pulse;
  int pulses;
  uint8_t dir_flag;
  uint16_t zerolimit;
  uint16_t finallimit;
  GPIO_TypeDef *gpio_ls_port;
  uint8_t id;

} motores_steer_t;
volatile int pulsos = 0;
motores_steer_t motores[4] = {
    [0] = // TRASERO DERECHO
    {
        .angulo_centro = 120,
        .pulses = 0,
        .max_pulse = 0,
        .dir_A = DIR_A_TRASERO_DER_PIN,
        .dir_B = DIR_B_TRASERO_DER_PIN,
        .gpio_dir_port = DIR_GPIO,
        .pwm_tim = &MOTOR_DIR_TIM_PWM,
        .pwm_channel = MOTOR_TRASERO_DER_PWM,
        .active_channel = ENC_A_TRASERO_DER,
        .dir_flag = 0,
        .zerolimit = LS_TRASERO_DX_0G_Pin,
        .finallimit = LS_TRASERO_DX_180G_Pin,
        .gpio_ls_port = LS_TRASERO_DX_0G_GPIO_Port,
        .enc_tim = &ENCODERS_STEER_TIM,
        .id = 1,

    },
    [1] = // TRASERO IZQUIERDO
    {
        .angulo_centro = 120,
        .pulses = 0,
        .max_pulse = 0,
        .dir_A = DIR_A_TRASERO_IZQ_PIN,
        .dir_B = DIR_B_TRASERO_IZQ_PIN,
        .gpio_dir_port = DIR_GPIO,
        .pwm_tim = &MOTOR_DIR_TIM_PWM,
        .pwm_channel = MOTOR_TRASERO_IZQ_PWM,
        .active_channel = ENC_A_TRASERO_IZQ,
        .enc_tim = &ENCODERS_STEER_TIM,
        .dir_flag = 0,
        .zerolimit = LS_TRASERO_IX_0G_Pin,
        .finallimit = LS_TRASERO_IX_180G_Pin,
        .gpio_ls_port = LS_TRASERO_IX_0G_GPIO_Port,
        .id = 2,
    },

    [2] = // DELANTERO DERECHO
    {
        .angulo_centro = 120,
        .pulses = 0,
        .max_pulse = 0,
        .dir_A = DIR_A_DELANTERO_IZQ_PIN,
        .dir_B = DIR_B_DELANTERO_IZQ_PIN,
        .gpio_dir_port = DIR_GPIO,
        .pwm_tim = &MOTOR_DIR_TIM_PWM,
        .pwm_channel = MOTOR_DELANTERO_IZQ_PWM,
        .active_channel = ENC_A_DELANTERO_IZQ,
        .enc_tim = &ENCODERS_STEER_TIM,
        .dir_flag = 0,
        .zerolimit = LS_DELANTERO_IX_0G_Pin,
        .finallimit = LS_DELANTERO_IX_180G_Pin,
        .gpio_ls_port = LS_DELANTERO_IX_0G_GPIO_Port,
        .id = 1,
    },
};
volatile bool zerodegre = 0;
volatile int times = 0;
volatile bool degrees180 = 0;
volatile uint8_t estado_de_maquina = BUSCAR;
volatile int max_pulse = 0;
volatile int target_pulse = 0;
uint8_t dir_FLAG = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
uint8_t seteo(motores_steer_t *);
void chaseInterrupt(motores_steer_t *motor, TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM15_Init();
  MX_I2C1_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&MOTOR_DIR_TIM_PWM, MOTOR_TRASERO_DER_PWM);
  HAL_TIM_PWM_Start(&MOTOR_DIR_TIM_PWM, MOTOR_DELANTERO_IZQ_PWM);
  HAL_TIM_PWM_Start(&MOTOR_DIR_TIM_PWM, MOTOR_TRASERO_IZQ_PWM);
  HAL_TIM_IC_Start_IT(&ENCODERS_STEER_TIM, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&ENCODERS_STEER_TIM, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&ENCODERS_STEER_TIM, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&ENCODERS_STEER_TIM, TIM_CHANNEL_4);
  HAL_Delay(100);

  /* USER CODE END 2 */
  for (uint8_t i = 0; i < 2; i++) {
    seteo(&motores[i]);
  }
  HAL_GPIO_WritePin(motores[2].gpio_dir_port, motores[2].dir_A, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motores[2].gpio_dir_port, motores[2].dir_B, GPIO_PIN_SET);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
   */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  for (int i = 0; i < 2; i++) {

    chaseInterrupt(&motores[i], htim);
  }
}

uint8_t seteo(motores_steer_t *motor_ptr) {

  // static uint8_t dir_FLAG = 0;
  uint8_t done = 0;

  static uint32_t pwm = 3300;

  while (!done) {

    bool zerodegre = (HAL_GPIO_ReadPin(motor_ptr->gpio_ls_port,
                                       motor_ptr->zerolimit)) == GPIO_PIN_RESET;
    bool degrees180 =
        (HAL_GPIO_ReadPin(motor_ptr->gpio_ls_port, motor_ptr->finallimit)) ==
        GPIO_PIN_RESET;
    switch (estado_de_maquina) {
    case BUSCAR: {
      HAL_GPIO_WritePin(motor_ptr->gpio_dir_port, motor_ptr->dir_A,
                        GPIO_PIN_SET);
      HAL_GPIO_WritePin(motor_ptr->gpio_dir_port, motor_ptr->dir_B,
                        GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(motor_ptr->pwm_tim, motor_ptr->pwm_channel, pwm);
      estado_de_maquina = BUSCAR_ZERO;
      motor_ptr->dir_flag = 0;
      break;
    }
    case BUSCAR_ZERO: {
      if (zerodegre) {
        HAL_GPIO_WritePin(motor_ptr->gpio_dir_port, motor_ptr->dir_A,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_ptr->gpio_dir_port, motor_ptr->dir_B,
                          GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor_ptr->pwm_tim, motor_ptr->pwm_channel, 0);
        motor_ptr->pulses = 0;
        HAL_Delay(200);

        motor_ptr->dir_flag = 0;
        HAL_GPIO_WritePin(motor_ptr->gpio_dir_port, motor_ptr->dir_A,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_ptr->gpio_dir_port, motor_ptr->dir_B,
                          GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor_ptr->pwm_tim, motor_ptr->pwm_channel, pwm);
        estado_de_maquina = BUSCAR_FINAL;
      }
      break;
    }
    case BUSCAR_FINAL: {

      if (degrees180) {
        HAL_GPIO_WritePin(motor_ptr->gpio_dir_port, motor_ptr->dir_A,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_ptr->gpio_dir_port, motor_ptr->dir_B,
                          GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor_ptr->pwm_tim, motor_ptr->pwm_channel, 0);
        motor_ptr->max_pulse = motor_ptr->pulses;
        HAL_Delay(200);
        motor_ptr->dir_flag = 0;
        estado_de_maquina = CENTRAR;
      }

      break;
    }
    case CENTRAR: {
      target_pulse = (120 * motor_ptr->max_pulse) / 270;
      if (target_pulse == motor_ptr->pulses) {

        HAL_GPIO_WritePin(motor_ptr->gpio_dir_port, motor_ptr->dir_A,
                          GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_ptr->gpio_dir_port, motor_ptr->dir_B,
                          GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor_ptr->pwm_tim, motor_ptr->pwm_channel, 0);
        estado_de_maquina = BUSCAR;
        motor_ptr->dir_flag = 0;
        done = 1;
      }
      if (target_pulse < motor_ptr->pulses) {
        motor_ptr->dir_flag = 1;
        HAL_GPIO_WritePin(motor_ptr->gpio_dir_port, motor_ptr->dir_A,
                          GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_ptr->gpio_dir_port, motor_ptr->dir_B,
                          GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor_ptr->pwm_tim, motor_ptr->pwm_channel, 6399);
      }
      if (target_pulse > motor_ptr->pulses) {
        motor_ptr->dir_flag = 0;
        HAL_GPIO_WritePin(motor_ptr->gpio_dir_port, motor_ptr->dir_A,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_ptr->gpio_dir_port, motor_ptr->dir_B,
                          GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor_ptr->pwm_tim, motor_ptr->pwm_channel, 6399);
      }

      break;
    }
    }
  }
  return 1;
}

void chaseInterrupt(motores_steer_t *motor, TIM_HandleTypeDef *htim) {
  if (htim->Channel == motor->active_channel) {
    if (motor->dir_flag == 1) {
      motor->pulses--;
    }
    if (motor->dir_flag == 0) {
      motor->pulses++;
    }
  }
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {

  /* Disables the MPU */
  HAL_MPU_Disable();

  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
