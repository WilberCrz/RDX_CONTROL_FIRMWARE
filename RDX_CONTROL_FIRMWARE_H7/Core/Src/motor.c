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

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"

#include "motor.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/_intsup.h>


// Variables para leer en la gráfica
volatile float plot_target_rpm = 0.0f;
volatile float plot_current_rpm = 0.0f;
volatile float rpm_target = 0.0f;
volatile uint32_t counter_tick = 0;
volatile uint8_t estado_de_maquina = BUSCAR;
volatile float plot_target_dir = 0.0f;
volatile float plot_current_dir = 0.0f;

volatile float tune_kp_drive = 0.0f;
volatile float tune_ki_drive = 0.0f;
volatile float tune_kd_drive = 0.0f;

volatile float tune_kp_steer = 0.0f;
volatile float tune_ki_steer = 0.0f;
volatile float tune_kd_steer = 0.0f;

struct Motor_t {
  int type;
  TIM_HandleTypeDef *pwm_tim;
  uint32_t pwm_channel;
  GPIO_TypeDef *dir_portA;
  GPIO_TypeDef *dir_portB;
  uint16_t dirPin_A;
  uint16_t dirPin_B;
  float smooth_target_speed_rpm;
  float max_acceleration_rpm_per_sec;
  uint8_t states;
  bool is_enable;
  uint8_t dir_flag;

  union {
    /****************motor modo steering ****************** */
    struct {

      GPIO_TypeDef *izq_limit_port;
      GPIO_TypeDef *der_limit_port;
      uint16_t izqPin_limit;
      uint16_t derPin_limit;
      float target_position_deg;
      float current_position_deg;
      float center_degrees;
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
      uint32_t center_pulses;
      uint32_t max_degrees;
      uint32_t max_pulses;
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
  uint32_t boost_pwm;
  uint32_t max_pwm;
  uint32_t min_pwm;
};

MotorHandle_t Motor_Init(const MotorConfig_t *config) {
  if (config == NULL)
    return NULL;
  MotorHandle_t motor = (MotorHandle_t)malloc(sizeof(struct Motor_t));
  if (motor != NULL) {
    motor->type = config->type;               // roll del motor
    motor->pwm_tim = config->pwm_tim;         // pwm_timer
    motor->pwm_channel = config->pwm_channel; // canal pwm
    motor->dir_portA = config->dir_portA;     // puerto de pin de direccion A
    motor->dir_portB = config->dir_portB;     // puerto de pin de direccion B
    motor->dirPin_A = config->dir_A;          // pin direccion A
    motor->dirPin_B = config->dir_B;          // pin direccion B
    motor->max_pwm = config->max_pwm;
    motor->min_pwm = config->min_pwm;
    motor->boost_pwm = config->boost_pwm;
    motor->states = 0;

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
      motor->steering.center_degrees =
          config
              ->center_angle_deg; // angulo en grados correspondiente al centro
      motor->enc_steering.max_degrees =
          config->max_angle_deg; // angulo maximo en grados desde el 0 grados
      /*----------------Encoder steering mode-----------------*/
      motor->enc_steering.pulses = 0;
      motor->enc_steering.is_calibrated = 0;
      motor->enc_steering.max_pulses = 0;
      motor->enc_steering.center_pulses = 0;
    }

    if (motor->type == MOTOR_TYPE_DRIVE) {
      motor->drive.current_speed_rpm = 0.0f;
      motor->drive.target_speed_rpm = 0.0f;
      motor->smooth_target_speed_rpm = 0.0f;
      motor->max_acceleration_rpm_per_sec = 250.0f;

      /*----------------Encoder drive mode-----------------*/
      motor->enc_drive.ticks_time[0] = 0;
      motor->enc_drive.ticks_time[1] = 0;
    }

    motor->kd = config->kd; // constante proporcional
    motor->ki = config->ki; // constante integral
    motor->kp = config->kp; // constante derivativa

    motor->previous_error = 0.0f; // error previo
    motor->integral_error = 0.0f; // error acumulado

    motor->is_enable = 1;
  }

  return motor;
}

void Motor_Destroy(MotorHandle_t handle) {
  if (handle == NULL)
    return;

  __HAL_TIM_SET_COMPARE(handle->pwm_tim, handle->pwm_channel, 0);
  HAL_TIM_PWM_Stop(handle->pwm_tim, handle->pwm_channel);

  HAL_GPIO_WritePin(handle->dir_portA, handle->dirPin_A, GPIO_PIN_SET);
  HAL_GPIO_WritePin(handle->dir_portB, handle->dirPin_B, GPIO_PIN_SET);
  handle->is_enable = 0;
  free(handle);
}

void Motor_SetTargetSpeed(MotorHandle_t handle, float speed_rpm) {
  if (handle != NULL && handle->type == MOTOR_TYPE_DRIVE && handle->is_enable) {
    handle->drive.target_speed_rpm = speed_rpm;
    rpm_target = handle->drive.target_speed_rpm;
  }
}

void Motor_SetTargetPosition(MotorHandle_t handle, float rc_angle_deg) {
    // Si tu control RC envía directamente de 0 a 270, lo mapeamos 1:1
    float physical_target = rc_angle_deg;

    // Limitador de seguridad estricto para no chocar la estructura
    if (physical_target < ((float)handle->enc_steering.max_degrees * 0.1f)) {
      physical_target = ((float)handle->enc_steering.max_degrees * 0.1f);
    } else if (physical_target >
               (float)handle->enc_steering.max_degrees -
                   ((float)handle->enc_steering.max_degrees * 0.1f)) {
      physical_target = (float)handle->enc_steering.max_degrees -
                        ((float)handle->enc_steering.max_degrees * 0.1f);
    }

    handle->steering.target_position_deg = physical_target;
}

void Motor_UpdateEncoder(MotorHandle_t handle, TIM_HandleTypeDef *_htim) {
  if (handle == NULL) {
    return;
  }
  if (handle->enc_capture_tim->Instance == _htim->Instance &&
      handle->enc_captureA_channel == _htim->Channel) {
    switch (handle->type) {
    case MOTOR_TYPE_DRIVE: {
      uint32_t tim_channel = 0;

      // Traducimos el ACTIVE_CHANNEL al TIM_CHANNEL correcto para poder leer el
      // registro
      switch (handle->enc_captureA_channel) {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        tim_channel = TIM_CHANNEL_1;
        break;
      case HAL_TIM_ACTIVE_CHANNEL_2:
        tim_channel = TIM_CHANNEL_2;
        break;
      case HAL_TIM_ACTIVE_CHANNEL_3:
        tim_channel = TIM_CHANNEL_3;
        break;
      case HAL_TIM_ACTIVE_CHANNEL_4:
        tim_channel = TIM_CHANNEL_4;
        break;
      default:
        return; // Error de canal
      }

      uint32_t captured_time =
          HAL_TIM_ReadCapturedValue(handle->enc_capture_tim, tim_channel);

      handle->enc_drive.ticks_time[0] = handle->enc_drive.ticks_time[1];
      handle->enc_drive.ticks_time[1] = captured_time;
      break;
    }

    case MOTOR_TYPE_STEER:
      if (handle->states & (1 << 0)) { // Si el bit de dirección esta activo,
                                       // incrementamos los pulsos
        handle->enc_steering.pulses++;
      } else {

        handle->enc_steering.pulses--;
      }
      counter_tick = handle->enc_steering.pulses;
      break;
    default:
      return;
      break;
    }
  }
}

/*  ####################################################################################*/
/*  ####################################################################################*/

void GetRPM_IT(MotorHandle_t handle) {
  if (handle == NULL || handle->type != MOTOR_TYPE_DRIVE ||
      handle->enc_capture_tim == NULL)
    return;
  uint32_t t0 = handle->enc_drive.ticks_time[0];
  uint32_t t1 = handle->enc_drive.ticks_time[1];
  uint32_t current_time = __HAL_TIM_GET_COUNTER(handle->enc_capture_tim);
  uint32_t time_since_pulse = 0;
  if (current_time >= t1) {
    time_since_pulse = current_time - t1;
  } else {
    time_since_pulse =
        (handle->enc_capture_tim->Instance->ARR - t1) + current_time + 1;
  }

  if (time_since_pulse >= 60000) {
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

  float rpm =
      60000000.0f / ((float)delta_ticks * (float)handle->enc_ppr_per_turn);
  if (handle->drive.current_speed_rpm == 0.0f) {
    handle->drive.current_speed_rpm = rpm;
  } else {
    handle->drive.current_speed_rpm =
        (handle->drive.current_speed_rpm * 0.90f) + (rpm * 0.10f);
  }

  plot_current_rpm = handle->drive.current_speed_rpm;
  handle->drive.current_speed_rpm = rpm;
}

/* ####################################################*/

/* ####################################################*/
void GetDegrees_IT(MotorHandle_t handle) {
  if (handle == NULL || handle->type != MOTOR_TYPE_STEER ||
      handle->enc_steering.max_pulses == 0) {
    return;
  }
  float current_pulses = (float)handle->enc_steering.pulses;
  float max_pulses = (float)handle->enc_steering.max_pulses;
  float position_deg = 0.0f;
  if (current_pulses <= 0.0f) {
    position_deg = 0.0f;
  } else {
    position_deg =
        (current_pulses * handle->enc_steering.max_degrees) / max_pulses;
  }
  handle->steering.current_position_deg = position_deg;
}

/* ####################################################*/

/* ####################################################*/

static float RPM_PID(MotorHandle_t motor, float delta_time_SEC) {
  if (motor == NULL || motor->type != MOTOR_TYPE_DRIVE) {
    return 0.0f;
  }

  if (motor->drive.target_speed_rpm == 0.0f) {
    motor->integral_error =
        0.0f; // Reiniciar el error integral si el objetivo es detenerse
    motor->previous_error = 0.0f; // Reiniciar el error previo
    return 0.0f;
  }
  // 1. Calculamos el Error (Diferencia entre lo que queremos y lo que
  // tenemos)
  float error = motor->drive.target_speed_rpm - motor->drive.current_speed_rpm;

  // 2. Proporcional (P)
  float p_term = motor->kp * error;

  // 3. Integral (I)
  motor->integral_error += error * delta_time_SEC;

  // Anti-Windup (Evita que el error integral crezca hasta el infinito si el
  // motor se traba)
  float max_i_memory = (float)motor->max_pwm * 0.4f;
  if (motor->integral_error > max_i_memory) {

    motor->integral_error = max_i_memory;
  }
  if (motor->integral_error < -max_i_memory) {

    motor->integral_error = -max_i_memory;
  }

  float i_term = motor->ki * motor->integral_error;

  // 4. Derivativo (D)
  float derivative = (error - motor->previous_error) / delta_time_SEC;
  float d_term = motor->kd * derivative;

  // 5. Guardamos el error para el próximo ciclo
  motor->previous_error = error;

  // 6. Retornamos la suma total (Este será el valor para el PWM)
  return p_term + i_term + d_term;
}

/* ####################################################*/

/* ####################################################*/

static float STEER_PID(MotorHandle_t motor, float delta_time_sec) {
  if (motor == NULL || motor->type != MOTOR_TYPE_STEER)
    return 0.0f;
  // El error es la diferencia en grados
  float error = motor->steering.target_position_deg -
                motor->steering.current_position_deg;

  if (fabsf(error) < 0.5f) {
    motor->integral_error = 0.0f;
    return 0.0f;
  }
  // Proporcional
  float p_term = motor->kp * error;

  motor->integral_error += error * delta_time_sec;
  float max_i_memory = (float)motor->max_pwm * 0.4f;
  if (motor->integral_error > max_i_memory)
    motor->integral_error = max_i_memory;
  if (motor->integral_error < -max_i_memory)
    motor->integral_error = -max_i_memory;
  // Integral
  float i_term = motor->ki * motor->integral_error;

  // Derivativo
  float derivative = (error - motor->previous_error) / delta_time_sec;
  float d_term = motor->kd * derivative;

  motor->previous_error = error;

  return p_term + i_term + d_term;
}

/* ####################################################*/

/* ####################################################*/

void ControllerLoop(MotorHandle_t handle, float delta_time_sec) {
  float pid_output = 0.0f;
  if (handle == NULL || !handle->is_enable)
    return;

  if (handle->states & (1 << 1)) { // Si el bit de parking esta activo, frenamos
    // el motor y no permitimos que se mueva
    Motor_Break(handle);

  } else {

    if (handle->type == MOTOR_TYPE_DRIVE) {

      float_t max_target_rpm =
          handle->max_acceleration_rpm_per_sec * delta_time_sec;
      float error_target =
          handle->drive.target_speed_rpm - handle->smooth_target_speed_rpm;

      if (error_target > max_target_rpm) {
        handle->smooth_target_speed_rpm += max_target_rpm;
      } else if (error_target < -max_target_rpm) {
        handle->smooth_target_speed_rpm -= max_target_rpm;
      } else {
        handle->smooth_target_speed_rpm = handle->drive.target_speed_rpm;
      }
      float original_target = handle->drive.target_speed_rpm;
      handle->drive.target_speed_rpm = handle->smooth_target_speed_rpm;

      GetRPM_IT(handle);

      plot_target_rpm = handle->drive.target_speed_rpm;
      plot_current_rpm = handle->drive.current_speed_rpm;
      pid_output = RPM_PID(handle, delta_time_sec);

      handle->drive.target_speed_rpm = original_target;

      if (pid_output < 0.0f) {
        pid_output = 0.0f;
      }

      if (handle->states & (1 << 0)) {
        HAL_GPIO_WritePin(handle->dir_portA, handle->dirPin_A, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(handle->dir_portB, handle->dirPin_B, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(handle->dir_portA, handle->dirPin_A, GPIO_PIN_SET);
        HAL_GPIO_WritePin(handle->dir_portB, handle->dirPin_B, GPIO_PIN_RESET);
      }
      uint32_t final_pwm = (uint32_t)fabsf(pid_output);
      uint32_t max =
          (handle->states & (1 << 3)) ? handle->boost_pwm : handle->max_pwm;
      if (final_pwm > max) {
        final_pwm = max;
      } else if (final_pwm < handle->min_pwm) {
        final_pwm = 0;
      }
      if (handle->states &
          (1 << 1)) { // Si el bit de parking esta activo, frenamos
                      // el motor y no permitimos que se mueva
        final_pwm = 0;
      }
      __HAL_TIM_SET_COMPARE(handle->pwm_tim, handle->pwm_channel, final_pwm);
    }
    if (handle->type == MOTOR_TYPE_STEER) { // TIPO STEER
      GetDegrees_IT(handle);
      plot_target_dir = handle->steering.target_position_deg; // Lo que quieres
      plot_current_dir = handle->steering.current_position_deg; // Lo que tienes

      pid_output = STEER_PID(handle, delta_time_sec);
      if ((handle->steering.current_position_deg <= (24.0f / 2.0f)) &&
          pid_output < 0.0f) {
        pid_output = 0.0f;
      }
      if (handle->steering.current_position_deg >=
          (handle->enc_steering.max_degrees - (24.0f / 2.0f))) {
        pid_output = 0.0f;
      }

      handle->kp = tune_kp_steer;
      handle->ki = tune_ki_steer;
      handle->kd = tune_kd_steer;
      counter_tick = handle->enc_steering.max_pulses;
      // NO limites el pid_output a 0 aquí. El signo define la dirección.
      if (pid_output > 0.0f) {
        handle->states |= (1 << 0);
      } else {
        handle->states &= ~(1 << 0);
      }

      if (handle->states & (1 << 0)) {
        HAL_GPIO_WritePin(handle->dir_portA, handle->dirPin_A, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(handle->dir_portB, handle->dirPin_B, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(handle->dir_portA, handle->dirPin_A, GPIO_PIN_SET);
        HAL_GPIO_WritePin(handle->dir_portB, handle->dirPin_B, GPIO_PIN_RESET);
      }
      // Aplicar PWM (siempre positivo para el timer)
      uint32_t final_pwm = (uint32_t)fabsf(pid_output);
      if (final_pwm > handle->max_pwm)
        final_pwm = handle->max_pwm;
      if (final_pwm < handle->min_pwm)
        final_pwm = 0;

      __HAL_TIM_SET_COMPARE(handle->pwm_tim, handle->pwm_channel, final_pwm);
    }
  }
}

/* ####################################################*/

/* ####################################################*/
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
/* ####################################################*/

/* ####################################################*/
void SetMaxDegres(
    MotorHandle_t handle) { // Ya no necesitas pasarle el "180" como parámetro
  if (handle == NULL || handle->type != MOTOR_TYPE_STEER)
    return;

  handle->enc_steering.max_pulses = handle->enc_steering.pulses;
  // Calculamos dónde caen los pulsos del centro basándonos en la
  // configuración
  handle->enc_steering.center_pulses =
      (uint32_t)((handle->steering.center_degrees /
                  handle->enc_steering.max_degrees) *
                 handle->enc_steering.max_pulses);
}

/* ####################################################*/

/* ####################################################*/
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

/* ####################################################*/

/* ####################################################*/
void Motor_SetDriveDirection(MotorHandle_t handle, bool is_forward) {
  // Solo permitimos cambiar la dirección manualmente al motor de tracción
  if (handle != NULL && handle->type == MOTOR_TYPE_DRIVE) {
    if (is_forward) {
      handle->states |= (1 << 0);
    } else {
      handle->states &= ~(1 << 0);
    }
  }
}

/* ####################################################*/

/* ####################################################*/
void Motor_SetParking(MotorHandle_t handle, bool park_state) {
  if (handle != NULL) {
    handle->states = (handle->states & ~(1 << 1)) | (park_state << 1);
  }
}

void Motor_Break(MotorHandle_t handle) {
  if (handle != NULL) {
    HAL_GPIO_WritePin(handle->dir_portA, handle->dirPin_A, GPIO_PIN_SET);
    HAL_GPIO_WritePin(handle->dir_portB, handle->dirPin_B, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(handle->pwm_tim, handle->pwm_channel, 0);
    handle->states |=
        (1 << 1); // Establecer el bit de parking para activar el freno
  }
}

uint16_t Get_LS_IZQ(MotorHandle_t handle) {
  if (handle == NULL || handle->type != MOTOR_TYPE_STEER) {
    return 0;
  }
  return handle->steering.izqPin_limit;
}
uint16_t Get_LS_DER(MotorHandle_t handle) {
  if (handle == NULL || handle->type != MOTOR_TYPE_STEER) {
    return 0;
  }

  return handle->steering.derPin_limit;
}

uint8_t Motor_GetType(MotorHandle_t handle) {
  if (handle == NULL) {
    return 0xFF; // Valor inválido para indicar error
  }
  return (uint8_t)handle->type;
}

uint8_t seteo(MotorHandle_t motor_ptr) {
  uint8_t done = 0;
  uint32_t target_pulse = 0;
  static uint32_t pwm = 6999;

  // Reiniciamos la máquina de estados cada vez que inicializamos un motor
  // nuevo
  estado_de_maquina = BUSCAR;

  while (!done) {

    bool zerodegre =
        (HAL_GPIO_ReadPin(motor_ptr->steering.izq_limit_port,
                          motor_ptr->steering.izqPin_limit)) == GPIO_PIN_RESET;
    bool degrees180 =
        (HAL_GPIO_ReadPin(motor_ptr->steering.der_limit_port,
                          motor_ptr->steering.derPin_limit)) == GPIO_PIN_RESET;

    switch (estado_de_maquina) {
    case BUSCAR: {
      HAL_GPIO_WritePin(motor_ptr->dir_portA, motor_ptr->dirPin_A,
                        GPIO_PIN_SET);
      HAL_GPIO_WritePin(motor_ptr->dir_portB, motor_ptr->dirPin_B,
                        GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(motor_ptr->pwm_tim, motor_ptr->pwm_channel, pwm);

      estado_de_maquina = BUSCAR_ZERO;
      break;
    }
    case BUSCAR_ZERO: {
      if (zerodegre) {
        HAL_GPIO_WritePin(motor_ptr->dir_portA, motor_ptr->dirPin_A,
                          GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_ptr->dir_portB, motor_ptr->dirPin_B,
                          GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor_ptr->pwm_tim, motor_ptr->pwm_channel,
                              0); // <-- CORREGIDO: 0 en lugar de -1
        motor_ptr->enc_steering.pulses = 0;
        HAL_Delay(200);

        motor_ptr->states |= (1 << 0);
        HAL_GPIO_WritePin(motor_ptr->dir_portA, motor_ptr->dirPin_A,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_ptr->dir_portB, motor_ptr->dirPin_B,
                          GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor_ptr->pwm_tim, motor_ptr->pwm_channel, pwm);
        estado_de_maquina = BUSCAR_FINAL;
      }
      break;
    }
    case BUSCAR_FINAL: {
      if (degrees180) {
        HAL_GPIO_WritePin(motor_ptr->dir_portA, motor_ptr->dirPin_A,
                          GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_ptr->dir_portB, motor_ptr->dirPin_B,
                          GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor_ptr->pwm_tim, motor_ptr->pwm_channel,
                              0); // <-- CORREGIDO: 0 en lugar de -1

        motor_ptr->enc_steering.max_pulses = motor_ptr->enc_steering.pulses;
        HAL_Delay(200);
        estado_de_maquina = CENTRAR;
      }
      break;
    }
    case CENTRAR: {
      // Usamos tu mapeo inverso: pulsos = (grados * max_pulses) / max_grados
      target_pulse = (uint32_t)((motor_ptr->steering.center_degrees *
                                 motor_ptr->enc_steering.max_pulses) /
                                motor_ptr->enc_steering.max_degrees);

      if (target_pulse == motor_ptr->enc_steering.pulses) {
        HAL_GPIO_WritePin(motor_ptr->dir_portA, motor_ptr->dirPin_A,
                          GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_ptr->dir_portB, motor_ptr->dirPin_B,
                          GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor_ptr->pwm_tim, motor_ptr->pwm_channel,
                              0); // <-- CORREGIDO: 0 en lugar de -1

        estado_de_maquina = BUSCAR;
        done = 1;
      }
      if (target_pulse < motor_ptr->enc_steering.pulses) {
        motor_ptr->states &= ~(1 << 0);
        HAL_GPIO_WritePin(motor_ptr->dir_portA, motor_ptr->dirPin_A,
                          GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_ptr->dir_portB, motor_ptr->dirPin_B,
                          GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor_ptr->pwm_tim, motor_ptr->pwm_channel, 7999);
      }
      if (target_pulse > motor_ptr->enc_steering.pulses) {
        motor_ptr->states |= (1 << 0);
        HAL_GPIO_WritePin(motor_ptr->dir_portA, motor_ptr->dirPin_A,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_ptr->dir_portB, motor_ptr->dirPin_B,
                          GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor_ptr->pwm_tim, motor_ptr->pwm_channel, 7999);
      }
      break;
    }
    }
  }
return 1;
}
