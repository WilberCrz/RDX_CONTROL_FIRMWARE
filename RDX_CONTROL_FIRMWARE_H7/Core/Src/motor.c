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
#include "stm32h7xx_hal_gpio.h"

#include "motor.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// Variables para leer en la gráfica
volatile float plot_target_rpm = 0.0f;
volatile float plot_current_rpm = 0.0f;
volatile float plot_pwm = 0.0f;
volatile float rpm_target = 0.0f;

volatile float plot_pwm_dir = 0.0f;
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

  union {
    /****************motor modo steering ****************** */
    struct {

      GPIO_TypeDef *izq_limit_port;
      GPIO_TypeDef *der_limit_port;
      uint16_t izqPin_limit;
      uint16_t derPin_limit;
      float target_position_deg;
      float current_position_deg;
      float max_degrees;
      float center_degrees;
      uint8_t homing_state;
      uint32_t last_homing_pulses;
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
          config->center_angle_deg; // angulo en grados correspondiente al centro
      motor->steering.max_degrees =
          config->max_angle_deg; // angulo maximo en grados desde el 0 grados
      motor->steering.homing_state =
          HOMING_IDLE; // estado de homing, inicia en idle no caliibrado
      motor->steering.last_homing_pulses =
          0; // variable para almacenar el conteo de pulsos en el último ciclo
             // de homing, útil para detectar si el motor se atascó durante el
             // homing
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
      motor->max_acceleration_rpm_per_sec = 1000.0f;

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
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel,
                          0); // Iniciamos con un valor de PWM bajo para evitar
                              // movimientos bruscos al iniciar

    if (motor->enc_capture_tim != NULL) {
      HAL_TIM_IC_Start_IT(motor->enc_capture_tim, motor->enc_captureA_channel);
    }

    motor->is_enable = 1;
  }
  return motor;
}

void Motor_Destroy(MotorHandle_t handle) {
  if (handle == NULL)
    return;

  __HAL_TIM_SET_COMPARE(handle->pwm_tim, handle->pwm_channel, 0);
  HAL_TIM_PWM_Stop(handle->pwm_tim, handle->pwm_channel);

  HAL_GPIO_WritePin(handle->dir_portA, handle->dirPin_A, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(handle->dir_portB, handle->dirPin_B, GPIO_PIN_RESET);
  handle->is_enable = 0;
  free(handle);
}

void Motor_SetTargetSpeed(MotorHandle_t handle, float speed_rpm) {
  if (handle != NULL && handle->type == MOTOR_TYPE_DRIVE && handle->is_enable) {
    handle->drive.target_speed_rpm = speed_rpm;
    rpm_target = handle->drive.target_speed_rpm;
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

void Motor_UpdateEncoder(MotorHandle_t handle, TIM_HandleTypeDef *_htim) {
  if (handle == NULL) {
    return;
  }
  HAL_TIM_ActiveChannel active_channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
  switch (handle->enc_captureA_channel) {
  case TIM_CHANNEL_1:
    active_channel = HAL_TIM_ACTIVE_CHANNEL_1;
    break;
  case TIM_CHANNEL_2:
    active_channel = HAL_TIM_ACTIVE_CHANNEL_2;
    break;
  case TIM_CHANNEL_3:
    active_channel = HAL_TIM_ACTIVE_CHANNEL_3;
    break;
  case TIM_CHANNEL_4:
    active_channel = HAL_TIM_ACTIVE_CHANNEL_4;
    break;
  default:
    break;
  };
  if (handle->enc_capture_tim->Instance == _htim->Instance &&
      active_channel == _htim->Channel) {
    switch (handle->type) {
    case MOTOR_TYPE_DRIVE: {

      uint32_t captured_time = HAL_TIM_ReadCapturedValue(
          handle->enc_capture_tim, handle->enc_captureA_channel);

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
  float max_degrees = (float)handle->enc_steering.max_degrees; //
  float pulses_to_degres = (current_pulses * max_degrees) / max_pulses;
  handle->steering.current_position_deg = pulses_to_degres;
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
  // 1. Calculamos el Error (Diferencia entre lo que queremos y lo que tenemos)
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

  if (fabsf(error) < 4.0f) {
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
    HAL_GPIO_WritePin(handle->dir_portA, handle->dirPin_A, GPIO_PIN_SET);
    HAL_GPIO_WritePin(handle->dir_portB, handle->dirPin_B, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(handle->pwm_tim, handle->pwm_channel, 0);

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
    }
    if (handle->type == MOTOR_TYPE_STEER) {
      if (handle->steering.homing_state == HOMING_SEEK_LEFT ||
          handle->steering.homing_state == HOMING_SEEK_RIGHT) {
        uint32_t delta_pulses = (uint32_t)handle->enc_steering.pulses -
                                handle->steering.last_homing_pulses;
        handle->steering.last_homing_pulses = handle->enc_steering.pulses;
        float current_speed =
            (fabsf((float)delta_pulses / (float)handle->enc_ppr_per_turn) *
             (60.0f / delta_time_sec));

        float target_speed = HOMING_RPM_SPEED;
        float error = target_speed - current_speed;

        float kp_homing = 100.0f; // Constante proporcional para homing, puedes
                                  // ajustarla según tu sistema
        float ki_homing = 80.0f;  // Constante integral para homing, puedes
                                  // ajustarla según tu sistema

        float pterm = kp_homing * error;
        handle->integral_error += error * delta_time_sec;

        float max_i = handle->max_pwm * 0.8f;
        if (handle->integral_error > max_i)
          handle->integral_error = max_i;
        if (handle->integral_error < -max_i)
          handle->integral_error = -max_i;

        pid_output = 6000.0f;

        // 3. Forzar el signo según la dirección que mande la máquina de estados
        if ((handle->states & (1 << 0)) == 0) {
          pid_output = -fabsf(pid_output); // Moviendo a la izquierda
        } else {
          pid_output = fabsf(pid_output); // Moviendo a la derecha
        }
      } else {
        GetDegrees_IT(handle);

        plot_target_dir = handle->steering.target_position_deg;
        plot_current_dir = handle->steering.current_position_deg;
        pid_output = STEER_PID(handle, delta_time_sec);
      }
    }

    if (pid_output > 0.0f) {
      handle->states |=
          (1 << 0); // Establecer el bit de dirección para adelante
    } else if (pid_output < 0.0f) {
      handle->states &= ~(1 << 0); // Limpiar el bit de dirección para atrás
    }

    if (handle->states & (1 << 0)) {
      HAL_GPIO_WritePin(handle->dir_portA, handle->dirPin_A, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(handle->dir_portB, handle->dirPin_B, GPIO_PIN_SET);
    } else if ((handle->states & ~(1 << 0)) == 0) {
      HAL_GPIO_WritePin(handle->dir_portA, handle->dirPin_A, GPIO_PIN_SET);
      HAL_GPIO_WritePin(handle->dir_portB, handle->dirPin_B, GPIO_PIN_RESET);
    }
    uint32_t final_pwm = (uint32_t)fabsf(pid_output);
    if (final_pwm > handle->max_pwm) {
      final_pwm = handle->max_pwm;
    } else if (final_pwm < handle->min_pwm) {
      final_pwm = 0;
    }
    plot_pwm = final_pwm;
    if (handle->states &
        (1 << 1)) { // Si el bit de parking esta activo, frenamos
                    // el motor y no permitimos que se mueva
      final_pwm = 0;
    }
    __HAL_TIM_SET_COMPARE(handle->pwm_tim, handle->pwm_channel, final_pwm);
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

void SetMaxDegres(MotorHandle_t handle, uint32_t max_degrees) {
  if (handle == NULL || max_degrees == 0 || handle->type != MOTOR_TYPE_STEER) {
    return;
  }
  handle->enc_steering.max_degrees = max_degrees;
  handle->enc_steering.max_pulses = handle->enc_steering.pulses;
  handle->enc_steering.center_pulses = handle->enc_steering.max_pulses / 2;
  handle->enc_steering.is_calibrated = 1;
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

void Motor_SetParking(MotorHandle_t handle, bool park_state) {
  if (handle != NULL && handle->type == MOTOR_TYPE_DRIVE) {
    handle->states = (handle->states & ~(1 << 1)) | (park_state << 1);
  }
}

void Motor_Break(MotorHandle_t handle) {
  if (handle != NULL) {
    HAL_GPIO_WritePin(handle->dir_portA, handle->dirPin_A, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(handle->dir_portB, handle->dirPin_B, GPIO_PIN_RESET);
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
uint8_t Motor_SteerHomingTask(MotorHandle_t handle) {
  if (handle == NULL || handle->type != MOTOR_TYPE_STEER)
    return HOMING_DONE;
  if (handle->enc_steering.is_calibrated == 1)
    return HOMING_DONE;

  bool hit_left =
      (HAL_GPIO_ReadPin(handle->steering.izq_limit_port,
                        handle->steering.izqPin_limit) == GPIO_PIN_RESET);
  bool hit_right =
      (HAL_GPIO_ReadPin(handle->steering.der_limit_port,
                        handle->steering.derPin_limit) == GPIO_PIN_RESET);

  switch (handle->steering.homing_state) {
  case HOMING_IDLE:
    handle->integral_error = 0.0f; // Limpiar memoria del PID
    handle->steering.homing_state = HOMING_SEEK_LEFT;
    handle->states &= ~(1 << 0); // Establecer Dirección Izquierda (Bit 0 en 0)
    handle->states &= ~(1 << 1); // Liberar freno
    break;

  case HOMING_SEEK_LEFT:
    if (hit_right) {
      Motor_Break(handle);
      SetZeroDegres(handle);
      handle->enc_steering.pulses = 0; // Este es nuestro Cero Absoluto
      handle->integral_error = 0.0f;
      handle->steering.homing_state = HOMING_SEEK_RIGHT;
      handle->states |= (1 << 0);  // Establecer Dirección Derecha (Bit 0 en 1)
      handle->states &= ~(1 << 1); // Liberar freno para que arranque de nuevo
    }
    break;

  case HOMING_SEEK_RIGHT:
    if (hit_left) {
      Motor_Break(handle);
      SetMaxDegres(handle, 180); // Registra el total de pulsos
      handle->integral_error = 0.0f;
      Motor_SetTargetPosition(
          handle,
          180.0f); // Mover a centro para evitar estar justo en el límite
      handle->steering.homing_state = HOMING_CENTERING;
      handle->states &= ~(1 << 1); // Liberar freno
    }
    break;

  case HOMING_CENTERING:
    // Cuando el PID normal lo haya llevado cerca de los 90°
    if (fabsf(handle->steering.current_position_deg - 90.0f) < 2.0f) {
      handle->steering.homing_state = HOMING_DONE;
    }
    break;

  case HOMING_DONE:
    return HOMING_DONE;
  }
  return handle->steering.homing_state;
}

uint8_t Motor_GetType(MotorHandle_t handle) {
  if (handle == NULL) {
    return 0xFF; // Valor inválido para indicar error
  }
  return (uint8_t)handle->type;
}