
#include "uart_driver.h"
#include<string.h>
#include<stdio.h>
// Estados RX
#define RX_WAIT_HEADER  0
#define RX_WAIT_LENGTH  1
#define RX_WAIT_DATA    2
#define RX_WAIT_CHECK   3

// ================= INIT =================
void uart_init(uart_handle_t *handle, UART_HandleTypeDef *huart)
{
    handle->huart = huart;
    handle->rx_state = RX_WAIT_HEADER;
    handle->rx_index = 0;
    handle->rx_length = 0;
    handle->tx_busy = 0;

}

// ================= CHECKSUM =================
static uint8_t calc_checksum(uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for(uint8_t i = 0; i < len; i++)
        sum += data[i];
    return sum;
}

// ================= SEND =================
void uart_send(uart_handle_t *handle, uint8_t *data, uint8_t length)
{
    if(handle->tx_busy) return;
    if(length > UART_MAX_DATA) return;

    uint8_t i = 0;

    handle->tx_buffer[i++] = UART_HEADER;
    handle->tx_buffer[i++] = length;

    for(uint8_t j = 0; j < length; j++)
        handle->tx_buffer[i++] = data[j];

    uint8_t checksum = calc_checksum(data, length);
    handle->tx_buffer[i++] = checksum;

    handle->tx_busy = 1;

    HAL_UART_Transmit_IT(handle->huart, handle->tx_buffer, i);
}

// ================= START RX =================
void uart_start_receive(uart_handle_t *handle)
{
    HAL_UART_Receive_IT(handle->huart, &handle->rx_byte, 1);
}

// ================= RX CALLBACK CORE =================
void uart_rx_callback(uart_handle_t *handle, uint8_t byte)
{
    switch(handle->rx_state)
    {
        case RX_WAIT_HEADER:
            if(byte == UART_HEADER)
            {
                handle->rx_state = RX_WAIT_LENGTH;
            }
            break;

        case RX_WAIT_LENGTH:
            handle->rx_length = byte;
            handle->rx_index = 0;

            if(handle->rx_length > UART_MAX_DATA)
            {
                handle->rx_state = RX_WAIT_HEADER;
            }
            else
            {
                handle->rx_state = RX_WAIT_DATA;
            }
            break;

        case RX_WAIT_DATA:
            handle->rx_buffer[handle->rx_index++] = byte;

            if(handle->rx_index >= handle->rx_length)
            {
                handle->rx_state = RX_WAIT_CHECK;
            }
            break;

        case RX_WAIT_CHECK:
        {
            uint8_t checksum = calc_checksum(handle->rx_buffer, handle->rx_length);

            if(checksum == byte)
            {
                // Mensaje válido
                handle->received_msg.length = handle->rx_length;

                for(uint8_t i = 0; i < handle->rx_length; i++)
                    handle->received_msg.data[i] = handle->rx_buffer[i];
            }

            handle->rx_state = RX_WAIT_HEADER;
        }
        break;
    }
}

// ================= TX DONE =================
void uart_tx_callback(uart_handle_t *handle)
{
    handle->tx_busy = 0;
}

// ================= DATA AVAILABLE =================
uint8_t uart_available(uart_handle_t *handle)
{
    return (handle->received_msg.length > 0);
}

//=================== STRUCT TO BUFFER============
void datos_to_bytes(datos *d, uint8_t *buffer)
{
    uint8_t i = 0;

    memcpy(&buffer[i], &d->d_temp, sizeof(float));
    i += sizeof(float);

    memcpy(&buffer[i], &d->d_vbat, sizeof(float));
    i += sizeof(float);

    memcpy(&buffer[i], &d->d_gyro, sizeof(uint32_t));
    i += sizeof(uint32_t);

    memcpy(&buffer[i], &d->d_accel, sizeof(uint32_t));
    i += sizeof(uint32_t);

    memcpy(&buffer[i], &d->d_rpm, sizeof(uint32_t));
}

/*
 *
 *
 *
 * uart_driver.C
 *
 *  Created on: Mar 24, 2026
 *      Author: Reymon
 */


