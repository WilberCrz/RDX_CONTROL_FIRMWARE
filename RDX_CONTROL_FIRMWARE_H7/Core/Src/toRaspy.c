/**
 * @file toRaspy.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-04-03
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "toRaspy.h"
#include "stm32h7xx_hal_uart.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/_intsup.h>
// Estados RX
#define RX_WAIT_HEADER 0
#define RX_WAIT_LENGTH 1
#define RX_WAIT_DATA 2
#define RX_WAIT_CHECK 3

// ================= INIT =================
void uart_init(uart_handle_t *handle, UART_HandleTypeDef *huart) {
  handle->huart = huart;
  handle->rx_state = RX_WAIT_HEADER;
  handle->rx_index = 0;
  handle->rx_length = 0;
  handle->tx_busy = 0;
  memset(handle->rx_buffer, 0, sizeof(handle->rx_buffer));
  memset(handle->tx_buffer, 0, sizeof(handle->tx_buffer));
}

// ================= CHECKSUM =================
static uint8_t calc_checksum(uint8_t *data, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; i++)
    sum += data[i];
  return sum;
}

// ================= SEND =================
void uart_send(uart_handle_t *handle, char *data, uint16_t length) {
  if (handle->huart->gState == HAL_UART_STATE_READY) {

    HAL_UART_Transmit_IT(handle->huart, (uint8_t *)data, length);
  }
}

// ================= START RX =================
void uart_start_receive(uart_handle_t *handle) {
  HAL_UART_Receive_IT(handle->huart, &handle->rx_byte, 1);
}

// ================= RX CALLBACK CORE =================
void uart_rx_callback(uart_handle_t *handle, uint8_t byte) {
  switch (handle->rx_state) {
  case RX_WAIT_HEADER:
    if (byte == UART_HEADER) {
      handle->rx_state = RX_WAIT_LENGTH;
    }
    break;

  case RX_WAIT_LENGTH:
    handle->rx_length = byte;
    handle->rx_index = 0;

    if (handle->rx_length > UART_MAX_DATA) {
      handle->rx_state = RX_WAIT_HEADER;
    } else {
      handle->rx_state = RX_WAIT_DATA;
    }
    break;

  case RX_WAIT_DATA:
    handle->rx_buffer[handle->rx_index++] = byte;

    if (handle->rx_index >= handle->rx_length) {
      handle->rx_state = RX_WAIT_CHECK;
    }
    break;

  case RX_WAIT_CHECK: {
    uint8_t checksum = calc_checksum(handle->rx_buffer, handle->rx_length);

    if (checksum == byte) {
      // Mensaje válido
      handle->received_msg.length = handle->rx_length;

      for (uint8_t i = 0; i < handle->rx_length; i++)
        handle->received_msg.data[i] = handle->rx_buffer[i];
    }

    handle->rx_state = RX_WAIT_HEADER;
  } break;
  }
}

// ================= TX DONE =================

// ================= DATA AVAILABLE =================
uint8_t uart_available(uart_handle_t *handle) {
  return (handle->received_msg.length > 0);
}

//=================== STRUCT TO BUFFER============
void datos_to_bytes(datos *d, uint8_t *buffer) {
  if (d == NULL || buffer == NULL)
    return;

  memcpy(buffer, d, sizeof(datos));
}
