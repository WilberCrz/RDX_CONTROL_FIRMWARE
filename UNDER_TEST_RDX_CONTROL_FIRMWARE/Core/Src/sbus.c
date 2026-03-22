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
#include "FreeRTOS.h"
#include "stm32f3xx.h"
#include "stm32f3xx_hal_uart.h"
#include "usart.h"
#include "sbus.h"
#include "cmsis_os.h"
#include <string.h>


int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);

struct sbus_t {

	UART_HandleTypeDef *uartHandle;
	uint8_t rx_buffer[25];
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

	sbus_Handle sbus=(sbus_Handle) pvPortMalloc(sizeof(struct sbus_t));
	if (sbus != NULL) {

		sbus->Acel = 0;
		sbus->states = 0;
		sbus->Giro = 0;
		sbus->lost = 0;
		sbus->off = 0;
		sbus->last_packet_tick = 0;

		memset(sbus->rx_buffer, 0, sizeof(sbus->rx_buffer));
		memset(sbus->process_buffer, 0, sizeof(sbus->process_buffer));
		memset(sbus->canales, 0, sizeof(sbus->canales));

		sbus->uartHandle = UART_Handle;

		return sbus;

	}

	return NULL;

}

uint8_t getFailsafe(sbus_Handle Handle) {
    if (Handle == NULL) return 1;
	uint32_t current_tick = osKernelGetTickCount();
	if(current_tick - Handle->last_packet_tick > 100) { // Si han pasado más de 100 ms desde el último paquete válido
		return 1; // Failsafe activado
	}
    return (Handle->off != 0);
}

uint8_t getFrameLost(sbus_Handle Handle) {
    if (Handle == NULL) return 1;
    return (Handle->lost != 0);
}


uint8_t getbyte(sbus_Handle Handle, uint8_t num_byte) {

	if (Handle == NULL) return 0;

	if (num_byte >= 0 && num_byte <25) {

		return Handle->rx_buffer[num_byte];

	}

return 0;


};

uint8_t *getBuffer(sbus_Handle Handle) {

	return Handle->rx_buffer;
};



uint16_t getAcc(sbus_Handle Handle){
	if (Handle == NULL) return 0;
	uint16_t rpm = map(Handle->Acel, 1788, 200, 0, 100);

	if(rpm < 10){
		rpm = 0;
	}
	if(rpm > 90){
		rpm = 100;
	}
	return rpm;
};

// Función clásica para escalar valores
int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


uint8_t getDir(sbus_Handle Handle) {
	if (Handle == NULL) return 0;
	if(Handle->states & (1<<0)){
		return 1;
	}
	return 0;
};


uint16_t getGiro(sbus_Handle Handle) {
	if (Handle == NULL) return 0;

	uint16_t giro = map(Handle->Giro, 200, 1800, 0, 180);

	if(giro>87 && giro < 93){
		giro = 90;
	}

	if(giro < 0) {
		giro = 0;
	}
	if(giro > 180){
		giro = 180;
	}
	return giro;

}

uint8_t getparking(sbus_Handle Handle) {
	if (Handle == NULL) return 0;
	if(Handle->states & (1<<1)){
		return 1;
	}
	return 0;
};


uint16_t getChannel(sbus_Handle Handle , uint8_t num_Channel) {
	if (Handle == NULL) return 0;
	if (num_Channel >= 0 && num_Channel <10) {

		return Handle->canales[num_Channel];

	}

	return 0;

};


void sbusParse(sbus_Handle Handle) {

	if (Handle == NULL) return;


		Handle->canales[0] = (Handle->process_buffer[1]       |(Handle->process_buffer[2] & 0x07) << 8 );
		Handle->canales[1] = ((Handle->process_buffer[2] >> 3 | Handle->process_buffer[3] << 5 ) & 0x07FF);
		Handle->canales[2] = ((Handle->process_buffer[3] >> 6 | Handle->process_buffer[4] << 2  | Handle->process_buffer[5] <<10) & 0x07FF);
		Handle->canales[3] = ((Handle->process_buffer[5] >> 1 | Handle->process_buffer[6] << 7) & 0x07FF);
		Handle->canales[4] = ((Handle->process_buffer[6] >> 4 | Handle->process_buffer[7] << 4) & 0x07FF);
		Handle->canales[5] = ((Handle->process_buffer[7] >> 7 | Handle->process_buffer[8] << 1 | Handle->process_buffer[9] << 9) & 0x07FF);
		Handle->canales[6] = ((Handle->process_buffer[9] >> 2 | Handle->process_buffer[10] << 6 ) & 0x07FF);
		Handle->canales[7] = ((Handle->process_buffer[10] >> 5| Handle->process_buffer[11] << 3 ) & 0x07FF);
		Handle->canales[8] = ((Handle->process_buffer[12]     | Handle->process_buffer[13] << 8 ) & 0x07FF);
		Handle->canales[9] = ((Handle->process_buffer[13] >> 3| Handle->process_buffer[14] << 5 ) & 0x07FF);


		Handle->lost = Handle->process_buffer[23] & (1 <<2);
		Handle->off = Handle->process_buffer[23] & (1 <<3);

		Handle->Acel = Handle->canales[2];
		Handle->Giro = Handle->canales[0];
		
		if(Handle->canales[9] < 1000){
			Handle->states |= (1<<0); // Establecer bit 0 para dirección adelante
		} else {
			Handle->states &= ~(1<<0); // Limpiar bit 0 para dirección atrás
		}

		if (Handle->canales[8]>1500) {
			Handle->states |= (1<<1);
		}else{
			Handle->states &= ~(1<<1);
		}



}

USART_TypeDef* sbusGetUartHandle(sbus_Handle Handle){
	return  Handle->uartHandle->Instance;
}

void sbus_commit_data(sbus_Handle Handle) {

	if (Handle == NULL) return;
	memcpy(Handle->process_buffer, Handle->rx_buffer, 25);
	Handle->last_packet_tick = osKernelGetTickCount(); // Actualizar el tick del último paquete válido

}