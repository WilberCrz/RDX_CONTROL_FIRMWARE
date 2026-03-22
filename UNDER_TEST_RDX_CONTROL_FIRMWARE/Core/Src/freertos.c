/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "sbus.h"
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
/* USER CODE BEGIN Variables */

extern MotorHandle_t motores[2];
extern sbus_Handle sbus;
extern uint16_t motor_array_size;
volatile uint16_t throttle = 0;
volatile uint16_t dir = 0;
volatile uint8_t task_heartbeat = 0;
volatile uint16_t giro = 0;

osThreadId_t xMotorTaskHandle = NULL;
const osThreadAttr_t xMotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
osThreadId_t xSbusTaskHandle = NULL;
const osThreadAttr_t xSbusTask_attributes = {
  .name = "SbusTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
osThreadId_t xTelemetryTaskHandle = NULL;
const osThreadAttr_t xTelemetryTask_attributes = {
  .name = "TelemetryTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t xRaspyComTaskHandle = NULL;
const osThreadAttr_t xRaspyComTask_attributes = {
  .name = "RaspyComTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
osThreadId_t xFNCompleteTaskHandle = NULL;
const osThreadAttr_t xFNCompleteTask_attributes = {
  .name = "FNCompleteTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityIdle,
};


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void TaskMotorControl(void *pvParameters);
void TaskRFSbus(void *pvParameters);
void TaskTelemetria(void *pvParameters);
void TaskRaspyCom(void *pvParameters);
void TaskFNComplete(void *pvParameters);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  xMotorTaskHandle = osThreadNew(TaskMotorControl, NULL, &xMotorTask_attributes);

  // 2. Tarea de Radiofrecuencia SBUS (Prioridad alta)
 xSbusTaskHandle = osThreadNew(TaskRFSbus, NULL, &xSbusTask_attributes);  

  // 3. Tarea de Telemetría (Prioridad media)
  xTelemetryTaskHandle = osThreadNew(TaskTelemetria, NULL, &xTelemetryTask_attributes);

  // 4. Tarea de Comunicación con Raspberry (Prioridad media-baja)
  xRaspyComTaskHandle = osThreadNew(TaskRaspyCom, NULL, &xRaspyComTask_attributes);

  // 5. Tareas Completadas / Background (Prioridad mínima)
  xFNCompleteTaskHandle = osThreadNew(TaskFNComplete, NULL, &xFNCompleteTask_attributes);

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/********TAREAS BEGIN ***************** */

// ----------------------------------------------------------------------------
// TAREA 1: CONTROL DE MOTORES (Frecuencia: 5 ms)
// ----------------------------------------------------------------------------
void TaskMotorControl(void *Pvparameter) {

uint32_t tick = osKernelGetTickCount();
  for (;;) {

    for (uint8_t i = 0; i < motor_array_size; i++) {

      ControllerLoop(motores[i], 0.005f);
    }
    tick +=5U;
    osDelayUntil(tick);
  }
}

// ----------------------------------------------------------------------------
// TAREA 2: LECTURA SBUS Y COMANDOS (Frecuencia: 20 ms)
// ----------------------------------------------------------------------------
void TaskRFSbus(void *Pvparameter) {

  for (;;) {

    sbusParse(sbus);
    taskENTER_CRITICAL();
    uint16_t value = getAcc(sbus);
    uint16_t gvalue = getGiro(sbus);
    uint16_t is_forward = getDir(sbus);
    taskEXIT_CRITICAL();

    throttle = (value>1025)?0:value; // Limitar a 1024 para evitar valores negativos por overflow
    dir=is_forward;
    giro = (gvalue>181)?0:gvalue;
    for (uint8_t i = 0; i < motor_array_size; i++) {

      Motor_SetTargetSpeed(motores[i],(float)throttle); // Escalamos a porcentaje
      Motor_SetTargetPosition(motores[i], (float)giro); // Escalamos a grados
      Motor_SetDriveDirection(motores[i], (is_forward==1));
    }
    osDelay(20U);
  }
}

// ----------------------------------------------------------------------------
// TAREA 3: TELEMETRÍA (Frecuencia: 500 ms)
// ----------------------------------------------------------------------------
void TaskTelemetria(void *Pvparameter) {

  for (;;) {

    osDelay(500U);
  }
}

// ----------------------------------------------------------------------------
// TAREA 4: COMUNICACIÓN RASPBERRY PI
// ----------------------------------------------------------------------------
void TaskRaspyCom(void *Pvparamter) {

  for(;;){

    osDelay(1000U);
  }
}

// ----------------------------------------------------------------------------
// TAREA 5: FUNCIONES COMPLETADAS
// ----------------------------------------------------------------------------
void TaskFNComplete(void *Pvparamter) {

  for(;;){
    osDelay(1000U);
  }
}
/************TAREAS END**************** */

/* USER CODE END Application */

