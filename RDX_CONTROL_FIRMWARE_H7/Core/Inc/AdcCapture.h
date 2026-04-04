#ifndef STM32_ADC_H
#define STM32_ADC_H

#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

// ------- CONFIGURACIONES --------
#define SAMPLES 4
#define ADC_MAX 65535
#define VREF_MV 3300
#define JOY_DEADZONE 1500

#define ADC_CHANNELS 4
#define ADC_FILTER_LEN 8

#define CH_JOY_X 0
#define CH_JOY_Y 1
#define CH_BAT1 2
#define CH_BAT2 3

// ------- PAQUETE A ENVIAR --------
typedef struct {
  ADC_HandleTypeDef *hadc;
  int16_t joy_x[8];
  int16_t joy_y[8];
  uint32_t BATT;
  uint32_t avg_x;
  uint32_t avg_y;
  uint32_t avg_b;
} ADC_module_handle;

// ------- PROTOTIPOS --------
void ADC_Start(ADC_module_handle *mod, ADC_HandleTypeDef *hadc_inst);
void ADC_Module_Update(ADC_module_handle *mod);
void ADC_Module_NotifyFromISR(ADC_module_handle *mod);
void send_packet_uart(void);

#endif