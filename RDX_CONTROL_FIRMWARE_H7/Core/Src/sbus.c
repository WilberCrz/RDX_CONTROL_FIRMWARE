/**
 * @file sbus.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-03-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "sbus.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_uart.h"
#include "usart.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
            int32_t out_max);
struct sbus_t {

  UART_HandleTypeDef *uartHandle;
  uint8_t rx_buffer;
  uint8_t process_buffer[25];
  uint8_t lost;
  uint8_t off;

  uint16_t Acel;
  uint8_t states;
  uint16_t Giro;
  uint16_t canales[10];
  uint32_t last_packet_tick;
};

sbus_Handle init_sbus(UART_HandleTypeDef *UART_Handle) {

  if (UART_Handle == NULL) {

    return NULL;
  }

  sbus_Handle sbus = (sbus_Handle)malloc(sizeof(struct sbus_t));
  if (sbus != NULL) {

    sbus->Acel = 0;
    sbus->states = 0;
    sbus->Giro = 0;
    sbus->lost = 0;
    sbus->off = 0;
    sbus->last_packet_tick = 0;
    sbus->rx_buffer = 0;
    sbus->states=0;

    memset(sbus->process_buffer, 0, sizeof(sbus->process_buffer));
    memset(sbus->canales, 0, sizeof(sbus->canales));
    sbus->uartHandle = UART_Handle;

    return sbus;
  }

  return NULL;
}

uint8_t getFailsafe(sbus_Handle Handle) {
  if (Handle == NULL)
    return 1;
  uint32_t current_tick = HAL_GetTick();
  if (current_tick - Handle->last_packet_tick >
      100) {  // Si han pasado más de 100 ms desde el último paquete válido
    return 1; // Failsafe activado
  }
  return (Handle->off != 0);
}

uint8_t getFrameLost(sbus_Handle Handle) {
  if (Handle == NULL)
    return 1;
  return (Handle->lost != 0);
}

uint8_t *getBuffer(sbus_Handle Handle) { return &Handle->rx_buffer; };

uint32_t getAcc(sbus_Handle Handle) {
  if (Handle == NULL)
    return 0;
  int32_t rpm = map(Handle->Acel, 1788, 200, 0, 290);

  if (rpm < 10) {
    rpm = 0;
  }
  if (rpm > 285) {
    rpm = 290;
  }
  return (uint32_t)rpm;
};

// Función clásica para escalar valores
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
            int32_t out_max) {

  // 1. Prevenir divisiones por cero si los mínimos y máximos son iguales por
  // error
  if (in_max == in_min) {
    return out_min;
  }

  // 2. Cálculo con paréntesis correctos (+ out_min va por FUERA de la división)
  int32_t mapped_value =
      ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;

  return mapped_value;
}
uint8_t getDir(sbus_Handle Handle) {
  if (Handle == NULL)
    return 0;
  if (Handle->states & (1 << 0)) {
    return 1;
  }
  return 0;
};

uint32_t getGiro(sbus_Handle Handle) {
  if (Handle == NULL)
    return 0;

  static uint32_t last_pos = 0;
  uint32_t giro = map(Handle->Giro, 200, 1800, 140, 0);

  if (abs((int32_t)giro - (int32_t)last_pos) > 3) {
    last_pos = giro;
  } else {
    giro = last_pos;
  }

  if (giro < 0) {
    giro = 0;
  }
  if (giro > 270) {
    giro = 270;
  }

  if (60 < giro && giro < 80) {
    giro = 70;
  }

  return giro;
}

uint8_t sbus_GetState(sbus_Handle Handle) {
  if (Handle == NULL)
    return 0;
  return Handle->states;
};

uint16_t getChannel(sbus_Handle Handle, uint8_t num_Channel) {
  if (Handle == NULL)
    return 0;
  if (num_Channel >= 0 && num_Channel < 10) {

    return Handle->canales[num_Channel];
  }

  return 0;
};

void sbusParse(
    sbus_Handle
        Handle) { // ch1=left-right,ch2=not_to_use,ch3=accel,ch4=not_to_use,ch5=F,ch6=not_to_use,ch7=H,ch8=C,ch9=B,ch10=A,

  if (Handle == NULL)
    return;

  if (Handle->process_buffer[0] == 0x0F && Handle->process_buffer[24] == 0x00) {

    Handle->canales[0] =
        (Handle->process_buffer[1] | (Handle->process_buffer[2] & 0x07) << 8);
    Handle->canales[1] =
        ((Handle->process_buffer[2] >> 3 | Handle->process_buffer[3] << 5) &
         0x07FF);
    Handle->canales[2] =
        ((Handle->process_buffer[3] >> 6 | Handle->process_buffer[4] << 2 |
          Handle->process_buffer[5] << 10) &
         0x07FF);
    Handle->canales[3] =
        ((Handle->process_buffer[5] >> 1 | Handle->process_buffer[6] << 7) &
         0x07FF);
    Handle->canales[4] =
        ((Handle->process_buffer[6] >> 4 | Handle->process_buffer[7] << 4) &
         0x07FF);
    Handle->canales[5] =
        ((Handle->process_buffer[7] >> 7 | Handle->process_buffer[8] << 1 |
          Handle->process_buffer[9] << 9) &
         0x07FF);
    Handle->canales[6] =
        ((Handle->process_buffer[9] >> 2 | Handle->process_buffer[10] << 6) &
         0x07FF);
    Handle->canales[7] =
        ((Handle->process_buffer[10] >> 5 | Handle->process_buffer[11] << 3) &
         0x07FF);
    Handle->canales[8] =
        ((Handle->process_buffer[12] | Handle->process_buffer[13] << 8) &
         0x07FF);
    Handle->canales[9] =
        ((Handle->process_buffer[13] >> 3 | Handle->process_buffer[14] << 5) &
         0x07FF);

    Handle->lost = Handle->process_buffer[23] & (1 << 2);
    Handle->off = Handle->process_buffer[23] & (1 << 3);

    Handle->Acel = Handle->canales[2];
    Handle->Giro = Handle->canales[0];
    Handle->last_packet_tick =
        HAL_GetTick(); // Actualizar el tick del último paquete válido

    if (Handle->canales[6] < 1000)// Establecer bit 3 para boost.
      Handle->states |= (1 << 3);
    } else {
      Handle->states &= ~(1 << 3); // Limpiar bit 3 para desactivar boost.
    }

    if (Handle->canales[9] < 1000) {
      Handle->states |= (1 << 0); // Establecer bit 0 para dirección adelante
    } else {
      Handle->states &= ~(1 << 0); // Limpiar bit 0 para dirección atrás
    }

    if (Handle->canales[8] > 1500) {
      Handle->states |= (1 << 1); // establecer bit 1 para freno de mano
    } else {
      Handle->states &=
          ~(1 << 1); // Limpiar bit 1 para liberar freno de mano
    }
  }

USART_TypeDef *sbusGetUartHandle(sbus_Handle Handle) {
  return Handle->uartHandle->Instance;
}

void sbus_commit_data(sbus_Handle Handle) {
  if (Handle == NULL)
    return;

  static uint8_t currentbytepos = 0;

  // RUTINA DE SINCRONIZACIÓN
  if (currentbytepos == 0) {
    // Si estamos esperando el primer byte, DEBE ser estrictamente 0x0F
    if (Handle->rx_buffer == 0x0F) {
      Handle->process_buffer[currentbytepos] = Handle->rx_buffer;
      currentbytepos++;
    }
    // Si no es 0x0F, es basura. Lo ignoramos y currentbytepos sigue siendo 0.
  } else {
    // Ya estamos sincronizados, guardamos el byte donde toca
    Handle->process_buffer[currentbytepos] = Handle->rx_buffer;
    currentbytepos++;
  }

  // Cuando tenemos el paquete completo (25 bytes)
  if (currentbytepos >= 25) {
    // Doble verificación: El último byte debe ser el cierre de SBUS
    if (Handle->process_buffer[24] == 0x00 ||
        Handle->process_buffer[24] == 0x04 ||
        Handle->process_buffer[24] == 0x14 ||
        Handle->process_buffer[24] == 0x24) {

      sbusParse(Handle); // Procesamos los canales
    }

    // Pase lo que pase (haya sido un paquete válido o corrupto),
    // reiniciamos el contador para buscar el próximo 0x0F
    currentbytepos = 0;
  }
}