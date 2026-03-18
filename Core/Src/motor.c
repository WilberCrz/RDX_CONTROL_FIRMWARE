/**
 * @file motor.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-03-01
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "FreeRTOS.h"
#include "task.h"


#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"

#include "motor.h"
#include "portable.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

struct Motor_t {
  int type;
  TIM_HandleTypeDef *pwm_tim;
  uint32_t pwm_channel;
  GPIO_TypeDef *dir_portA;
  GPIO_TypeDef *dir_portB;
  uint16_t dirPin_A;
  uint16_t dirPin_B;
  bool is_enable;
  bool forward;
  bool has_fault;

  union {
    /****************motor modo steering ****************** */
    struct {

      GPIO_TypeDef *izq_limit_port;
      GPIO_TypeDef *der_limit_port;
      uint16_t izqPin_limit;
      uint16_t derPin_limit;
      float target_position_deg;
      float current_position_deg;
    } steering;
    /***********************motor modo drive ****************** */
    struct {
      float current_speed_rpm;
      float target_speed_rpm;
    } drive;
  };

  /****************encoder **************************************** */
  TIM_HandleTypeDef *enc_capture_tim;
  uint16_t enc_captureA_channel;
  uint16_t motor_gear_ratio;
  uint32_t enc_ppr_per_turn;

  union {
    /****************encoder modo steering ****************** */
    struct {
      uint32_t pulses; // pulsos_1 = sum_pulsos_actual, pulsos_0 =
                       // sum_pulsos_anterior.
      uint8_t is_calibrated;
      uint32_t max_pulses;
      uint32_t center_pulses;
      uint32_t max_degrees;
    } enc_steering;

    /***********************encoder modo drive ****************** */
    struct {
      uint32_t ticks_time[2]; // captura de tiempos en 2 puntos para calcular la
      // diferencia.
    } enc_drive;
  };

  /***********************constantes para PID ****************** */
  float kp;
  float ki;
  float kd;

  /***********************variables para PID ****************** */
  float integral_error;
  float previous_error;
  uint32_t max_pwm;
  uint32_t min_pwm;
};

MotorHandle_t Motor_Init(const MotorConfig_t *config) {
  if (config == NULL)
    return NULL;
  MotorHandle_t motor = (MotorHandle_t)pvPortMalloc(sizeof(struct Motor_t));
  if (motor != NULL) {
    motor->type = config->type;               // roll del motor
    motor->pwm_tim = config->pwm_tim;         // pwm_timer
    motor->pwm_channel = config->pwm_channel; // canal pwm
    motor->dir_portA = config->dir_portA;     // puerto de pin de direccion A
    motor->dir_portB = config->dir_portB;     // puerto de pin de direccion B
    motor->dirPin_A = config->dir_A;          // pin direccion A
    motor->dirPin_B = config->dir_b;          // pin direccion B
    motor->max_pwm = config->max_pwm;
    motor->min_pwm = config->min_pwm;

    /*----------encoder-----------------------*/
    motor->motor_gear_ratio = config->gear_ratio;
    motor->enc_ppr_per_turn = config->enc_ppr * motor->motor_gear_ratio;
    motor->enc_capture_tim = config->enc_capture_tim;
    motor->enc_captureA_channel = config->enc_captureA_channel;

    HAL_GPIO_WritePin(motor->dir_portA, motor->dirPin_A, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->dir_portB, motor->dirPin_B, GPIO_PIN_RESET);

    if (motor->type == MOTOR_TYPE_STEER) {
      motor->steering.der_limit_port =
          config->der_limit_port; // puerto de limit_switch derecho
      motor->steering.izq_limit_port =
          config->izq_limit_port; // puerto de limit_switch izquierdo
      motor->steering.izqPin_limit =
          config->izq_limit; //  pin limit_switch izquiero
      motor->steering.derPin_limit =
          config->der_limit;                       // pin limit_switch derecho
      motor->steering.target_position_deg = 0.0f;  // angulo objetivo en grados
      motor->steering.current_position_deg = 0.0f; // angulo actual en grados

      /*----------------Encoder steering mode-----------------*/
      motor->enc_steering.pulses = 0;
      motor->enc_steering.is_calibrated = 0;
      motor->enc_steering.max_pulses = 0;
      motor->enc_steering.center_pulses = 0;
    }

    if (motor->type == MOTOR_TYPE_DRIVE) {
      motor->drive.current_speed_rpm = 0.0f;
      motor->drive.target_speed_rpm = 0.0f;

      /*----------------Encoder drive mode-----------------*/
      motor->enc_drive.ticks_time[0] = 0;
      motor->enc_drive.ticks_time[1] = 0;
    }

    motor->kd = config->kd; // constante proporcional
    motor->ki = config->ki; // constante integral
    motor->kp = config->kp; // constante derivativa

    motor->previous_error = 0.0f; // error previo
    motor->integral_error = 0.0f; // error acumulado

    HAL_TIM_PWM_Start(motor->pwm_tim, motor->pwm_channel);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, 0);

    motor->is_enable = 1;
  }
  return motor;
}

void Motor_Destroy(MotorHandle_t handle) {
  if (handle == NULL)
    return;

  HAL_TIM_PWM_Stop(handle->pwm_tim, handle->pwm_channel);
  __HAL_TIM_SET_COMPARE(handle->pwm_tim, handle->pwm_channel, 0);

  HAL_GPIO_WritePin(handle->dir_portA, handle->dirPin_A, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(handle->dir_portB, handle->dirPin_B, GPIO_PIN_RESET);
  handle->is_enable = 0;
  vPortFree(handle);
}

void Motor_SetTargetSpeed(MotorHandle_t handle, float speed_rpm) {
  if (handle != NULL && handle->type == MOTOR_TYPE_DRIVE && handle->is_enable) {
    handle->drive.target_speed_rpm = speed_rpm;
  }
}

void Motor_SetTargetPosition(MotorHandle_t handle, float angle_degrees) {

  if (handle != NULL && handle->type == MOTOR_TYPE_STEER &&
      handle->enc_steering.is_calibrated == 1) {
    if (angle_degrees < 0.0f) {
      angle_degrees = 0.0f;
    } else if (angle_degrees > (float)handle->enc_steering.max_degrees) {
      angle_degrees = (float)handle->enc_steering.max_degrees;
    }

    handle->steering.target_position_deg = angle_degrees;
  }
}

void Motor_UpdateEncoder(MotorHandle_t handle) {
  switch (handle->type) {
  case MOTOR_TYPE_DRIVE: {

    uint32_t captured_time = HAL_TIM_ReadCapturedValue(
        handle->enc_capture_tim, handle->enc_captureA_channel);

    handle->enc_drive.ticks_time[0] = handle->enc_drive.ticks_time[1];
    handle->enc_drive.ticks_time[1] = captured_time;
    break;
  }

  case MOTOR_TYPE_STEER:
    if (handle->forward) {
      handle->enc_steering.pulses++;
    } else {

      handle->enc_steering.pulses--;
    }
    break;
  default:
    return;
    break;
  }
}

/**
 * @brief  Calcula la velocidad actual en RPM utilizando los valores
 * actualizados por motor_update.
 *
 * @param handle Descriptor del motor
 * @return Velocidad calculada en float.Retorna 0.0f si el motor esta detenido o
 * se exede el timeout.
 */
void GetRPM_IT(MotorHandle_t handle) {
  if (handle == NULL || handle->type != MOTOR_TYPE_DRIVE ||
      handle->enc_capture_tim == NULL)
    return;
  taskENTER_CRITICAL();
  uint32_t t0 = handle->enc_drive.ticks_time[0];
  uint32_t t1 = handle->enc_drive.ticks_time[1];
  taskEXIT_CRITICAL();
  uint32_t current_time = __HAL_TIM_GET_COUNTER(handle->enc_capture_tim);
  uint32_t time_since_pulse = 0;
  if (current_time >= t1) {
    time_since_pulse = current_time - t1;
  } else {
    time_since_pulse =
        (handle->enc_capture_tim->Instance->ARR - t1) + current_time + 1;
  }

  if (time_since_pulse >= 100000) {
    handle->drive.current_speed_rpm = 0.0f;
    return;
  }
  uint32_t delta_ticks = 0;

  if (t1 >= t0) {
    delta_ticks = t1 - t0;
  } else {
    delta_ticks = (handle->enc_capture_tim->Instance->ARR - t0) + t1 + 1;
  }

  if (delta_ticks == 0) {
    handle->drive.current_speed_rpm = 0.0f;
    return;
  }
  float rps = (float)delta_ticks / 1000000.0f;
  float rpm = (1.0f / handle->enc_ppr_per_turn) * (60.0f / rps);
  handle->drive.current_speed_rpm = rpm;
}

/**
 * @brief
 *
 * @param handle
 */
void GetDegrees_IT(MotorHandle_t handle) {
  if (handle == NULL || handle->type != MOTOR_TYPE_STEER ||
      handle->enc_steering.max_pulses == 0) {
    return;
  }
  float current_pulses = (float)handle->enc_steering.pulses;
  float max_pulses = (float)handle->enc_steering.max_pulses;
  float max_degrees = (float)handle->enc_steering.max_degrees; //
  float pulses_to_degres = (current_pulses * max_degrees) / max_pulses;
  handle->steering.current_position_deg = pulses_to_degres;
}

/**
 * @brief implementacion del argoritmo de control de velocidad para motor tipo
 * MOTOR_TYPE_DRIVE.
 *
 * @details Calcula la salida basandose en la ecuacion de control.
 *
 * @param motor Descriptor del motor.
 */
static float RPM_PID(MotorHandle_t motor, float delta_time_SEC) {
  if (motor == NULL || motor->type != MOTOR_TYPE_DRIVE) {
    return 0.0f;
  }
  // 1. Calculamos el Error (Diferencia entre lo que queremos y lo que tenemos)
  float error = motor->drive.target_speed_rpm - motor->drive.current_speed_rpm;

  // 2. Proporcional (P)
  float p_term = motor->kp * error;

  // 3. Integral (I)
  motor->integral_error += error * delta_time_SEC;

  // Anti-Windup (Evita que el error integral crezca hasta el infinito si el
  // motor se traba)
  if (motor->integral_error > motor->max_pwm)
    motor->integral_error = motor->max_pwm;
  if (motor->integral_error < -motor->max_pwm)
    motor->integral_error = -motor->max_pwm;

  float i_term = motor->ki * motor->integral_error;

  // 4. Derivativo (D)
  float derivative = (error - motor->previous_error) / delta_time_SEC;
  float d_term = motor->kd * derivative;

  // 5. Guardamos el error para el próximo ciclo
  motor->previous_error = error;

  // 6. Retornamos la suma total (Este será el valor para el PWM)
  return p_term + i_term + d_term;
}

/**
 * @brief Implementacion del algoritmo de control de direccion para motor tipo
 * MOTOR_TYPE_STEER.
 *
 * @details Calcula el esfuerzo necesario para minimizar el error entre los
 * grados actuales y los objetivos.
 *
 * @param motor Descriptor del motor.
 *
 * @return Valor de ajuste para el ciclo de trabajo PWM float.
 */
static float STEER_PID(MotorHandle_t motor, float delta_time_sec) {
  if (motor == NULL || motor->type != MOTOR_TYPE_STEER)
    return 0.0f;
  // El error es la diferencia en grados
  float error = motor->steering.target_position_deg -
                motor->steering.current_position_deg;

  // Proporcional
  float p_term = motor->kp * error;

  motor->integral_error += error * delta_time_sec;

  if (motor->integral_error > motor->max_pwm)
    motor->integral_error = motor->max_pwm;
  if (motor->integral_error < -motor->max_pwm)
    motor->integral_error = -motor->max_pwm;
  // Integral
  float i_term = motor->ki * motor->integral_error;

  // Derivativo
  float derivative = (error - motor->previous_error) / delta_time_sec;
  float d_term = motor->kd * derivative;

  motor->previous_error = error;

  return p_term + i_term + d_term;
}

void ControllerLoop(MotorHandle_t handle, float delta_time_sec) {
  float pid_output = 0.0f;
  if (handle == NULL || !handle->is_enable)
    return;

  if (handle->type == MOTOR_TYPE_DRIVE) {
    GetRPM_IT(handle);
    pid_output = RPM_PID(handle, delta_time_sec);
  }
  if (handle->type == MOTOR_TYPE_STEER) {
    GetDegrees_IT(handle);
    pid_output = STEER_PID(handle, delta_time_sec);
  }

  if (pid_output >= 0) {
    handle->forward = 1;
    HAL_GPIO_WritePin(handle->dir_portA, handle->dirPin_A, GPIO_PIN_SET);
    HAL_GPIO_WritePin(handle->dir_portB, handle->dirPin_B, GPIO_PIN_RESET);
  } else {

    HAL_GPIO_WritePin(handle->dir_portA, handle->dirPin_A, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(handle->dir_portB, handle->dirPin_B, GPIO_PIN_SET);
  }
  uint32_t final_pwm = (uint32_t)fabs(pid_output);
  if (final_pwm > handle->max_pwm) {
    final_pwm = handle->max_pwm;
  } else if (final_pwm < handle->min_pwm && final_pwm > 0) {
    final_pwm = 0;
  }
  __HAL_TIM_SET_COMPARE(handle->pwm_tim, handle->pwm_channel, final_pwm);
}
bool Motor_GetState(MotorHandle_t handle) {
  if (handle->is_enable == 1) {
    return 1;
  }
  return 0;
}
void SetZeroDegres(MotorHandle_t handle) {
  if (handle == NULL || handle->type != MOTOR_TYPE_STEER)
    return;
  handle->enc_steering.pulses = 0;
  handle->enc_steering.is_calibrated = 0;
}

void SetMaxDegres(MotorHandle_t handle, uint32_t max_degrees) {
  if (handle == NULL || max_degrees == 0 || handle->type != MOTOR_TYPE_STEER) {
    return;
  }
  handle->enc_steering.max_degrees = max_degrees;
  handle->enc_steering.max_pulses = handle->enc_steering.pulses;
  handle->enc_steering.center_pulses = handle->enc_steering.max_pulses / 2;
  handle->enc_steering.is_calibrated = 1;
}

/**
 * @brief
 *
 * @param handle
 * @return float
 */
float Motor_GetCurrentSpeed(MotorHandle_t handle) {

  if (handle == NULL || handle->type != MOTOR_TYPE_DRIVE) {
    return 0.0f;
  }
  return handle->drive.current_speed_rpm;
}
/**
 * @brief
 *
 * @param handle
 * @return float
 */
float Motor_GetCurrentPosition(MotorHandle_t handle) {
  if (handle == NULL || handle->type != MOTOR_TYPE_STEER) {
    return 0.0f;
  }
  return handle->steering.current_position_deg;
}