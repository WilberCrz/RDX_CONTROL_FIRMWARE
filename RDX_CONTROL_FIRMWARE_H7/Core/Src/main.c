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
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "AdcCapture.h"
#include "motor.h"
#include "sbus.h"
#include "stm32h7xx.h"
#include "toRaspy.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/_intsup.h>
#include <sys/cdefs.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  void (*taskFunction)(void *pvparameter);
  uint32_t interval;
  uint32_t lastExecution;

} Task_t;

typedef struct {
  uint16_t rpm[4];
  uint32_t Postop;
  uint32_t Posbot;
  uint8_t Motor_states;
} packedToRaspy_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MotorHandle_t motores[4];
sbus_Handle sbus = NULL;
uint8_t motor_array_size = 0;
ADC_module_handle JOY;
ADC_module_handle BATTERY;
uart_handle_t toraspy_uart_handle;
datos rover_data;
uint32_t current_tick = 0;
uint32_t throttle = 0;
volatile uint32_t dir = 0;
volatile uint32_t giro = 0;
volatile uint8_t motor_states = 0;
volatile uint8_t system_calibrated = 0;
char payload[128];
packedToRaspy_t dataToRaspy = {
    .Motor_states = 0,
    .Posbot = 0,
    .Postop = 0,
    .rpm = {0, 0, 0, 0},

};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void taskrfsbus(void *pvparameter);
void taskToRaspy(void *pvparameter);
void taskMotorControl(void *pvparameter);
void taskADC(void *pvparameter);
void taskHoming(void *pvparameter);

void serializer(packedToRaspy_t *data, char *buffer);
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
  MotorConfig_t cfg_motores[4] = {
      [0] =
          {
              // m3 & m1 izq
              .type = MOTOR_TYPE_DRIVE,
              .pwm_tim = &MOTOR3_TIM_PWM,
              .pwm_channel = MOTOR3_IZQ_PWM,
              .dir_portA = DIR_GPIO,
              .dir_portB = DIR_GPIO,
              .dir_A = DIR_IZQ_DRIVE_PIN_INA,
              .dir_B = DIR_IZQ_DRIVE_PIN_INB,
              .enc_capture_tim = &ENCODERS_DRIVE_MOTOR3_TIM,
              .enc_captureA_channel = A_MOTOR3_IZQ,
              .gear_ratio = 27,
              .enc_ppr = 16,
              .kp = 80,
              .ki = 20,
              .kd = 0,
              .boost_pwm = 15999,
              .max_pwm = 7999,
              .min_pwm = 0,
              .states = 0,
          },
      [1] =
          {
              // m3 & m1 DER
              .type = MOTOR_TYPE_DRIVE,
              .pwm_tim = &MOTOR3_TIM_PWM,
              .pwm_channel = MOTOR3_DER_PWM,
              .dir_portA = DIR_GPIO,
              .dir_portB = DIR_GPIO,
              .dir_A = DIR_DER_DRIVE_PIN_INA,
              .dir_B = DIR_DER_DRIVE_PIN_INB,
              .enc_capture_tim = &ENCODERS_DRIVE_MOTOR3_TIM,
              .enc_captureA_channel = A_MOTOR3_DER,
              .gear_ratio = 27,
              .enc_ppr = 16,
              .kp = 80,
              .ki = 20,
              .kd = 0,
              .boost_pwm = 15999,
              .max_pwm = 7999,
              .min_pwm = 0,
              .states = 0,
          },
      [2] =
          {
              // m2 DER

              .type = MOTOR_TYPE_DRIVE,
              .pwm_tim = &MOTOR2_TIM_PWM,
              .pwm_channel = MOTOR2_DER_PWM,
              .dir_portA = DIR_GPIO,
              .dir_portB = DIR_GPIO,
              .dir_A = DIR_DER_DRIVE_PIN_INA,
              .dir_B = DIR_DER_DRIVE_PIN_INB,
              .enc_capture_tim = &ENCODERS_DRIVE_MOTOR2_TIM,
              .enc_captureA_channel = A_MOTOR2_DER,
              .gear_ratio = 27,
              .enc_ppr = 16,
              .kp = 80,
              .ki = 20,
              .kd = 0,
              .boost_pwm = 15999,
              .max_pwm = 7999,
              .min_pwm = 0,
              .states = 0,
          },
      [3] =
          {
              // m2 IZQ
              .type = MOTOR_TYPE_DRIVE,
              .pwm_tim = &MOTOR2_TIM_PWM,
              .pwm_channel = MOTOR2_IZQ_PWM,
              .dir_portA = DIR_GPIO,
              .dir_portB = DIR_GPIO,
              .dir_A = DIR_IZQ_DRIVE_PIN_INA,
              .dir_B = DIR_IZQ_DRIVE_PIN_INB,
              .enc_capture_tim = &ENCODERS_DRIVE_MOTOR2_TIM,
              .enc_captureA_channel = A_MOTOR2_IZQ,
              .gear_ratio = 27,
              .enc_ppr = 16,
              .kp = 80,
              .ki = 20,
              .kd = 0,
              .boost_pwm = 15999,
              .max_pwm = 7999,
              .min_pwm = 0,
              .states = 0,
          },
  };

  motor_array_size = sizeof(cfg_motores) / sizeof(MotorConfig_t);
  rover_data.d_gyro = 0.0f;
  rover_data.d_temp = 0.0f;
  rover_data.d_vbat = (12 << 16) | 12;
  for (uint8_t i = 0; i < motor_array_size; i++) {
    rover_data.axis_RPM[i] = 100;
  }
  rover_data.current_angle = (30 << 16) | 30;

  __HAL_UART_CLEAR_OREFLAG(&huart5);
  __HAL_UART_CLEAR_NEFLAG(&huart5);
  __HAL_UART_CLEAR_FEFLAG(&huart5);
  __HAL_UART_CLEAR_PEFLAG(&huart5);

  sbus = init_sbus(&huart5);
  HAL_UART_Receive_IT(&huart5, getBuffer(sbus), 1);
  uart_init(&toraspy_uart_handle, &huart3);

  for (uint8_t i = 0; i < motor_array_size; i++) {
    motores[i] = Motor_Init(&cfg_motores[i]);
    Motor_Break(motores[i]);
  }
  __HAL_TIM_SET_COMPARE(&MOTOR_DIR_TIM_PWM, MOTOR_TRASERO_DER_PWM, 0);
  __HAL_TIM_SET_COMPARE(&MOTOR_DIR_TIM_PWM, MOTOR_DELANTERO_IZQ_PWM, 0);
  __HAL_TIM_SET_COMPARE(&MOTOR_DIR_TIM_PWM, MOTOR_TRASERO_IZQ_PWM, 0);

  __HAL_TIM_SET_COMPARE(&MOTOR3_TIM_PWM, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&MOTOR3_TIM_PWM, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&MOTOR3_TIM_PWM, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&MOTOR3_TIM_PWM, TIM_CHANNEL_4, 0);

  __HAL_TIM_SET_COMPARE(&MOTOR2_TIM_PWM, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&MOTOR2_TIM_PWM, TIM_CHANNEL_2, 0);

  HAL_TIM_PWM_Start(&MOTOR_DIR_TIM_PWM, MOTOR_TRASERO_DER_PWM);
  HAL_TIM_PWM_Start(&MOTOR_DIR_TIM_PWM, MOTOR_DELANTERO_IZQ_PWM);
  HAL_TIM_PWM_Start(&MOTOR_DIR_TIM_PWM, MOTOR_TRASERO_IZQ_PWM);

  HAL_TIM_PWM_Start(&MOTOR3_TIM_PWM, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&MOTOR3_TIM_PWM, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&MOTOR3_TIM_PWM, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&MOTOR3_TIM_PWM, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&MOTOR2_TIM_PWM, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&MOTOR2_TIM_PWM, TIM_CHANNEL_2);

  HAL_TIM_IC_Start_IT(&ENCODERS_STEER_TIM, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&ENCODERS_STEER_TIM, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&ENCODERS_STEER_TIM, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&ENCODERS_STEER_TIM, TIM_CHANNEL_4);

  HAL_TIM_IC_Start_IT(&ENCODERS_DRIVE_MOTOR3_TIM,
                      TIM_CHANNEL_2); // A_MOTOR3_IZQ (CH2)
  HAL_TIM_IC_Start_IT(&ENCODERS_DRIVE_MOTOR3_TIM,
                      TIM_CHANNEL_4); // A_MOTOR3_DER (CH4)

  Task_t tasks_array[4] = {
      [0] = {.taskFunction = taskMotorControl,
             .interval = 5,
             .lastExecution = 0},
      [1] = {.taskFunction = taskrfsbus, .interval = 20, .lastExecution = 0},
      [2] = {.taskFunction = taskADC, .interval = 25, .lastExecution = 0},
      [3] = {.taskFunction = taskToRaspy, .interval = 100, .lastExecution = 0},
  };
  uint32_t NUM_TASKS = sizeof(tasks_array) / sizeof(Task_t);

  system_calibrated = 1;

  // ADC_Start(&JOY, &hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    current_tick = HAL_GetTick();

    for (uint8_t i = 0; i < NUM_TASKS; i++) {
      if (current_tick - tasks_array[i].lastExecution >=
          tasks_array[i].interval) {
        tasks_array[i].lastExecution = current_tick;
        tasks_array[i].taskFunction(NULL);
      }
    }
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

void taskrfsbus(void *pvparameter) {

  if (getFailsafe(sbus)) {
    throttle = 0;
    dir = 0;
    giro = 70;
    motor_states |= (1 << 1);
  } else {
    if ((sbus_GetState(sbus) & (1 << 1)) == 0) {
      throttle = getAcc(sbus);
      dir = getDir(sbus);
      giro = getGiro(sbus);
      motor_states &= ~(1 << 1); // Limpiar el bit de break
      if (dir == 1) {
        motor_states |= (1 << 0); // Establecer el bit de dirección
      } else {
        motor_states &= ~(1 << 0); // Limpiar el bit de dirección
      }
    } else {
      motor_states |= (1 << 1); // Establecer el bit de parking
      throttle = getAcc(sbus);
      giro = getGiro(sbus);
      dir = getDir(sbus);
    }
  }
  if (system_calibrated == 1) {

    int32_t diff = (int32_t)giro - 70;
    int32_t Lineal_throttle =
        (motor_states & (1 << 0)) ? (int32_t)throttle : -(int32_t)throttle;

    float mezcla_izq = (float)(Lineal_throttle + diff);
    float mezcla_der = (float)(Lineal_throttle - diff);
    float max_calculated=fmaxf(fabsf(mezcla_izq),fabsf(mezcla_der));

    if (max_calculated > MAX_ROVER_RPM) {
      mezcla_izq=mezcla_izq*(MAX_ROVER_RPM/max_calculated);
    
      mezcla_izq=mezcla_der*(MAX_ROVER_RPM/max_calculated);
    }

    if (Lineal_throttle > 15) {
      if(mezcla_der<0)mezcla_der=0;
      if(mezcla_izq<0)mezcla_izq=0;
    
    }else if(Lineal_throttle < -15){
      if (mezcla_der>0) mezcla_der = 0;
      if (mezcla_izq>0) mezcla_izq = 0;
    }
    uint8_t mtizq[2] = {0, 3};
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t j = mtizq[i];

      Motor_SetTargetSpeed(motores[j], fabsf(mezcla_izq));
      Motor_SetDriveDirection(motores[j], mezcla_izq >= 0);
      Motor_SetParking(motores[j], (motor_states & (1 << 1)) != 0);
    }
    uint8_t mtder[2] = {1, 2};
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t j = mtder[i];

      Motor_SetTargetSpeed(motores[j], fabsf(mezcla_der));
      Motor_SetDriveDirection(motores[j], mezcla_der >= 0);
      Motor_SetParking(motores[j], (motor_states & (1 << 1)) != 0);
    }

  } else {
    for (uint8_t i = 0; i < motor_array_size; i++) {
      Motor_SetParking(motores[i], 1);
    }
  }
}
void taskToRaspy(void *pvparameter) {

  dataToRaspy.Motor_states = motor_states;
  dataToRaspy.Posbot = (giro << 16) | giro;
  dataToRaspy.Postop = 0;
  for (uint8_t i = 0; i < motor_array_size; i++) {
    dataToRaspy.rpm[i] = Motor_GetCurrentSpeed(motores[i]);
  }
  // Procesar mensaje recibido
  serializer(&dataToRaspy, payload);
  HAL_UART_Transmit_IT(&huart3, (uint8_t *)payload, (uint16_t)sizeof(payload));

  // Aquí puedes agregar el código para manejar el mensaje recibido
}
void taskMotorControl(void *pvparameter) {
  static uint32_t last_pid_time = 0;
  uint32_t current_time = HAL_GetTick();
  float delta_time_sec = (current_time - last_pid_time) / 1000.0f;
  if (delta_time_sec <= 0.0f) {
    delta_time_sec = 0.001f;
  }
  for (uint8_t i = 0; i < motor_array_size; i++) {
    ControllerLoop(motores[i], delta_time_sec);
  }
  last_pid_time = current_time;
}
void taskADC(void *pvparameter) {}

void serializer(packedToRaspy_t *data, char *buffer) {
  sprintf(
      buffer,
      " \"PDT\":\"%hu\",\r\n\"PIT\":\"%hu\",\r\n\"PDD\":\"%hu\","
      "\r\n\"PID\":\"%hu\",\r\n\"MTS\":\"%u\",\r\n\"MI1\":\"%u\",\r\n\"MI2\":"
      "\"%u\",\r\n\"MI3\":\"%u\",\r\n\"MI4\":\"%u\"",
      (data->Posbot & 0xFFFF), (data->Posbot >> 16), (data->Postop & 0xFFFF),
      (data->Postop >> 16), data->Motor_states, data->rpm[0], data->rpm[1],
      data->rpm[2], data->rpm[3]);
}

// callback para timer con canales configurados como input capture direct
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  for (uint8_t i = 0; i < motor_array_size; i++) {
    Motor_UpdateEncoder(motores[i], htim);
  }
}
// callback para sbustouart dma
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (sbusGetUartHandle(sbus) == huart->Instance) {
    sbus_commit_data(sbus);
    HAL_UART_Receive_IT(huart, getBuffer(sbus), 1);
  }
  if (huart->Instance == USART3) {
    uart_rx_callback(&toraspy_uart_handle, toraspy_uart_handle.rx_byte);
    uart_start_receive(&toraspy_uart_handle);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (sbusGetUartHandle(sbus) == huart->Instance) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    // Reiniciar la recepción en caso de error
    HAL_UART_AbortReceive_IT(huart);
    HAL_UART_Receive_IT(huart, getBuffer(sbus), 1);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == NULL || huart->Instance != USART3) {
    return;
  } else {
  }
}

// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//   if (hadc == NULL)
//     return;
//   if (hadc->Instance == ADC1) {
//     ADC_Module_NotifyFromISR(&JOY);
//   }
//   if (hadc->Instance == ADC2) {

//     ADC_Module_NotifyFromISR(&BATTERY);
//   }
// }

// Recorremos los motores de dirección (índices 6 al 9 en tu array)
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
