/**
 * @file motor.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-03-01
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef MOTOR_H
#define MOTOR_H
#define DELTA_TIME_SEC 0.005f

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
#define A_MOTOR1_IZQ TIM_CHANNEL_1
#define A_MOTOR3_IZQ TIM_CHANNEL_2
#define A_MOTOR1_DER TIM_CHANNEL_3
#define A_MOTOR3_DER TIM_CHANNEL_4

#define A_MOTOR2_IZQ TIM_CHANNEL_1
#define A_MOTOR2_DER TIM_CHANNEL_2

#define ENC_A_TRASERO_IZQ TIM_CHANNEL_1
#define ENC_A_TRASERO_DER TIM_CHANNEL_2
#define ENC_A_DELANTERO_IZQ TIM_CHANNEL_3
#define ENC_A_DELANTERO_DER TIM_CHANNEL_4

#include "stm32h7xx.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

// tipo de motor(que tipo de operacion ejecuta)
typedef enum { MOTOR_TYPE_DRIVE, MOTOR_TYPE_STEER } Motor_type_t;

// publico el handle, contenido privado, puntero opaco
typedef struct Motor_t *MotorHandle_t;

typedef struct {
  int8_t type;
  TIM_HandleTypeDef *pwm_tim;
  uint16_t pwm_channel;
  GPIO_TypeDef *dir_portA;
  GPIO_TypeDef *dir_portB;
  uint16_t dir_A;
  uint16_t dir_B;
  uint8_t states;

  TIM_HandleTypeDef *enc_capture_tim;
  uint16_t enc_captureA_channel;
  uint16_t gear_ratio;
  uint32_t enc_ppr;

  GPIO_TypeDef *izq_limit_port;
  GPIO_TypeDef *der_limit_port;
  uint16_t izq_limit;
  uint16_t der_limit;
  uint32_t boost_pwm;
  uint32_t max_pwm;
  uint32_t min_pwm;

  float kp;
  float ki;
  float kd;

} MotorConfig_t;

/**
 * @brief Inicializa una instancia de motor, reserva memoria dinamica y
 * configura hardware periferico.
 *
 * @param config Puntero a la estructura con pines, temporizadores y constantes
 * PID.
 * @return MotorHandle_t con instancia configurada o NULL si falla la
 * asignacion.
 */
MotorHandle_t Motor_Init(const MotorConfig_t *config);

/**
 * @brief Libera los recursos de hardware y la memoria dinamica asociada al
 * motor.
 *
 * @param handle Descriptor del motor a eliminar.
 */
void Motor_Destroy(MotorHandle_t handle);

/**
 * @brief Actualiza el estado interno del encoder, debe llamarse desde una
 * interrupcion.
 *
 * @param handle Descriptor del motor.
 *
 * @param _htim timer que activo la interrupcion.
 */
void Motor_UpdateEncoder(MotorHandle_t handle, TIM_HandleTypeDef *_htim);

/**
 * @brief Establece el valor objetivo para motores de traccion.
 *
 * @param handle Descriptor del motor tipo MOTOR_TYPE_DRIVE.
 * @param speed_rpm  Velocidad objetivo en RPM
 */
void Motor_SetTargetSpeed(MotorHandle_t handle, float speed_rpm);

/**
 * @brief Establece el angulo objetivo para motores de direccion
 *
 * @param handle Descriptor del motor tipo MOTOR_TYPE_STEER.
 * @param angle_degrees Posicion objetivo en grados dentro del rango
 * establecido.
 */
void Motor_SetTargetPosition(MotorHandle_t handle, float_t angle_degrees);

/**
 * @brief Indica si un motor esta habilitad
 *
 * @param handle
 * @return true is esta habilitado
 * @return false si no esta inicializado
 */
bool Motor_GetState(MotorHandle_t handle);
float Motor_GetCurrentSpeed(MotorHandle_t handle);
float Motor_GetCurrentPosition(MotorHandle_t handle);

/**
 * @brief Setea el 0 grados para motores modo steer
 *
 * @param handle
 */
void SetZeroDegres(MotorHandle_t handle);
/**
 * @brief Setea en el motor tipo steer los grados maximos de rotacion a partir
 * del 0 grados
 *
 * @param handle
 * @param max_degrees
 */
void SetMaxDegres(MotorHandle_t handle, uint32_t max_degrees);

/**
 * @brief Calcula el pid y modifica la direccion y pwm. Exelente para llamar
 * desde tarea FreeRtos
 *
 * @param handle de cualquier tipo de motor
 *
 * @param delta_time_sec periodo de tiempo para calculo de pid clasico con
 * transformada de laplace
 */
void ControllerLoop(MotorHandle_t handle, float delta_time_sec);
/**
 * @brief
 *
 * @param handle
 * @param is_forward
 */
void Motor_SetDriveDirection(MotorHandle_t handle, bool is_forward);

/**
 * @brief
 *
 */

void Motor_SetParking(MotorHandle_t handle, bool park_state);
#endif