#ifndef STM32_ADC_H
#define STM32_ADC_H

#include "main.h"
#include <stdint.h>

// ------- CONFIGURACIONES --------
#define ADC_MAX         65535
#define VREF_MV         3300
#define JOY_DEADZONE    1500

#define ADC_CHANNELS    4
#define ADC_FILTER_LEN  8

#define CH_JOY_X    0
#define CH_JOY_Y    1
#define CH_BAT1     2
#define CH_BAT2     3

// ------- PAQUETE A ENVIAR --------
typedef struct __attribute__((packed)) {
    int16_t joy_x;
    int16_t joy_y;
    uint16_t batt1_raw;
    uint16_t batt2_raw;
} TxPacket_t;

// ------- PROTOTIPOS --------
void ADC_Start(void);
void update_packet(void); // Añadido para que el bucle principal lo reconozca
void send_packet_uart(void);

// Las siguientes se pueden usar de forma interna o externa
uint16_t adc_get_filtered(uint8_t ch);
int16_t map_joystick(uint16_t value);

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart3;

#endif
