
#include "AdcCapture.h"
#include "adc.h"
#include "stm32h743xx.h"
#include <stdint.h>
#include <sys/_types.h>

uint32_t adc_values[8];
uint8_t sample_count = 0;
// Estructura de transmisión

void ADC_Start(ADC_module_handle *mod, ADC_HandleTypeDef *hadc_inst) {
  mod->hadc = hadc_inst;
}

// --------------------------------------------------
// Filtro PROMEDIO de 8 muestras
// --------------------------------------------------
void ADC_Module_Update(ADC_module_handle *mod) {
  uint32_t sumX = 0;
  uint32_t sumY = 0;
  for (int i = 0; i < SAMPLES; i++) {
    /*Canal 1*/
    HAL_ADC_Start_IT(mod->hadc);
  }
  mod->avg_x = sumX/SAMPLES;
  mod->avg_y = sumY/SAMPLES;

}

