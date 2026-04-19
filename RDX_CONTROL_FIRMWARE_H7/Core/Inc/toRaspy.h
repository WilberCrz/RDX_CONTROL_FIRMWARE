/**
 * @file toRaspy.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-04-03
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef INC_UART_DRIVER_H_
#define INC_UART_DRIVER_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define UART_HEADER 0xAA
#define UART_MAX_DATA 100

typedef struct {
    uint8_t data[UART_MAX_DATA];
    uint8_t length;
} uart_payload_t;

typedef struct {
    UART_HandleTypeDef *huart;

    uint8_t tx_buffer[64];
    uint8_t rx_buffer[64];

    uint8_t rx_byte;
    uint8_t rx_index;
    uint8_t rx_length;
    uint8_t rx_state;

    uint8_t tx_busy;

    uart_payload_t received_msg;

} uart_handle_t;

typedef struct{
    uint32_t d_temp; //Temperatura
    uint32_t d_vbat; // pos 0-15 vbat1, pos 16-31 vbat2.
    uint32_t d_gyro; // en espera. se debe identificar el modelo
    uint32_t axis_RPM[6]; // rmp de los ejes de cada motor.
    uint32_t current_angle; // pos 0-15 angulo ruedas delanteras, pos 16-31 pos angulo ruedas traseras
} datos;

/**
 * @brief 
 * 
 * @param handle 
 * @param huart 
 */
void uart_init(uart_handle_t *handle, UART_HandleTypeDef *huart);

/**
 * @brief 
 * 
 * @param handle 
 * @param data 
 * @param length 
 */
void uart_send(uart_handle_t *handle, char *data, uint16_t length);

/**
 * @brief 
 * 
 * @param handle 
 */
void uart_start_receive(uart_handle_t *handle);

/**
 * @brief 
 * 
 * @param handle 
 * @return uint8_t 
 */
uint8_t uart_available(uart_handle_t *handle);

/**
 * @brief 
 * 
 * @param handle 
 * @param byte 
 */
void uart_rx_callback(uart_handle_t *handle, uint8_t byte);

/**
 * @brief 
 * @note se debe ejecutar dentro d la ISR 
 * @param handle
 */
void uart_tx_callback(uart_handle_t *handle);

/**
 * @brief  Envía los datos de telemetría a la Raspberry Pi a través de UART.
 * @note  Se debe definir un protocolo de comunicación con la Raspberry Pi para enviar los datos de telemetría. Este protocolo puede incluir un encabezado, la longitud de los datos, los datos en sí y un byte de verificación (checksum) para asegurar la integridad de los datos.
 * @param d estructura con los datos a enviar a la raspberry
 * @param buffer  buffer donde se almacenaran los bytes a enviar por uart, debe ser al menos de 16 bytes
 */
void datos_to_bytes(datos *d, uint8_t *buffer);

#endif
 /* INC_UART_DRIVER_H_ */
