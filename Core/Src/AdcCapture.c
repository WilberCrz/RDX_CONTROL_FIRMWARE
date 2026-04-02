#include "stm32_adc.h"

// DMA buffer: Alineado para D-Cache
__attribute__((aligned(32))) volatile uint16_t adc_dma_buffer[ADC_CHANNELS];

// Buffers del filtro
static uint16_t adc_samples[ADC_CHANNELS][ADC_FILTER_LEN];
static uint8_t sample_index = 0;

// Estructura de transmisión
static TxPacket_t txp;

void ADC_Start(void)
{
    // Activar regulador interno del ADC
    LL_ADC_EnableInternalRegulator(ADC1);
    osDelay(10); //

    // Calibración en modo single-ended
    LL_ADC_StartCalibration(ADC1, LL_ADC_CALIB_OFFSET, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1));

    osDelay(1);

    // Iniciar DMA
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_CHANNELS);

    osDelay(1);

   // Prellenar buffer
    for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++) {
        uint16_t v = adc_dma_buffer[ch];
        for (uint8_t i = 0; i < ADC_FILTER_LEN; i++)
            adc_samples[ch][i] = v;
    }
}

// --------------------------------------------------
// Filtro PROMEDIO de 8 muestras
// --------------------------------------------------
uint16_t adc_get_filtered(uint8_t ch)
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < ADC_FILTER_LEN; i++)
        sum += adc_samples[ch][i];

    return (uint16_t)(sum >> 3); // dividir entre 8
}

// --------------------------------------------------
// Mapeo Joystick → -100 a +100 con deadzone
// --------------------------------------------------
int16_t map_joystick(uint16_t value)
{
    int32_t centered = (int32_t)value - (ADC_MAX / 2);
    int32_t out = 0;

    int32_t range_max = (ADC_MAX / 2) - JOY_DEADZONE;

    if (centered > JOY_DEADZONE) {
        out = ((centered - JOY_DEADZONE) * 100) / range_max;
    }
    else if (centered < -JOY_DEADZONE) {
        out = ((centered + JOY_DEADZONE) * 100) / range_max;
    }
    else {
        return 0;
    }

    if (out > 100) out = 100;
    if (out < -100) out = -100;

    return (int16_t)out;
}

// --------------------------------------------------
// Actualiza el paquete filtrado
// --------------------------------------------------
void update_packet(void)
{
    // IMPORTANTE: Solo usar si ADC DMA está en RAM cachéada (AXI SRAM)
    SCB_InvalidateDCache_by_Addr((void*)adc_dma_buffer, 32);

    // Insertar nuevas muestras en buffers (antes de avanzar índice)
    for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++) {
        adc_samples[ch][sample_index] = adc_dma_buffer[ch];
    }

    // Avanzar índice circular
    sample_index = (sample_index + 1) & (ADC_FILTER_LEN - 1);

    // Obtener promedio
    uint16_t joy_x_raw = adc_get_filtered(CH_JOY_X);
    uint16_t joy_y_raw = adc_get_filtered(CH_JOY_Y);
    uint16_t bat1_raw  = adc_get_filtered(CH_BAT1);
    uint16_t bat2_raw  = adc_get_filtered(CH_BAT2);

    // Mapear
    txp.joy_x     = map_joystick(joy_x_raw);
    txp.joy_y     = map_joystick(joy_y_raw);
    txp.batt1_raw = bat1_raw;
    txp.batt2_raw = bat2_raw;
}

// --------------------------------------------------
// Enviar paquete directamente por UART
// --------------------------------------------------
void send_packet_uart(void)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)&txp, sizeof(txp), HAL_MAX_DELAY);
}
