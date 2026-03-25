/*
 * st_to_raspi.h
 *
 *  Created on: Mar 24, 2026
 *      Author: Reymon
 */

#ifndef INC_UART_DRIVER_H_
#define INC_UART_DRIVER_H_

#include "stm32l4xx_hal.h"
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
    float d_temp;
    float d_vbat;
    uint32_t d_gyro;
    uint32_t d_accel;
    uint32_t d_rpm;
} datos;

void uart_init(uart_handle_t *handle, UART_HandleTypeDef *huart);
void uart_send(uart_handle_t *handle, uint8_t *data, uint8_t length);
void uart_start_receive(uart_handle_t *handle);
uint8_t uart_available(uart_handle_t *handle);

void uart_rx_callback(uart_handle_t *handle, uint8_t byte);
void uart_tx_callback(uart_handle_t *handle);
void datos_to_bytes(datos *d, uint8_t *buffer);
#endif
 /* INC_UART_DRIVER_H_ */
