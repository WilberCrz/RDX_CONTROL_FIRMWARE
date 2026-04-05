#ifndef SBUS_H
#define SBUS_H


#include "stm32h7xx.h"
#include <stdint.h>

typedef struct sbus_t *sbus_Handle;

sbus_Handle init_sbus(UART_HandleTypeDef *UART_Handle);

void sbus_start(sbus_Handle Handle);
void sbusParse(sbus_Handle Handle);


uint32_t getAcc(sbus_Handle Handle);
uint8_t getDir(sbus_Handle Handle);
uint8_t sbus_GetState(sbus_Handle Handle);
uint32_t getGiro(sbus_Handle Handle);
uint16_t getChannel(sbus_Handle Handle , uint8_t num_Channel);
uint8_t *getBuffer(sbus_Handle Handle);

USART_TypeDef* sbusGetUartHandle(sbus_Handle Handle);
uint8_t getFailsafe(sbus_Handle Handle);
uint8_t getFrameLost(sbus_Handle Handle);

void sbus_commit_data(sbus_Handle Handle);
































#endif

