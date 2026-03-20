
#include "FreeRTOS.h"
#include "stm32f3xx.h"
#include "stm32f3xx_hal_uart.h"
#include "usart.h"
#include "sbus.h"
#include <string.h>



struct sbus_t {

	UART_HandleTypeDef *uartHandle;
	uint8_t rx_buffer[25];
	uint8_t tx_buffer[25];
	uint8_t lost;
	uint8_t off;

	uint16_t Acel;
	uint16_t Direc;
	uint16_t Giro;
	uint16_t canales[10];


};

sbus_Handle init_sbus(UART_HandleTypeDef *UART_Handle) {

	if (UART_Handle == NULL) {

		return NULL;

	}

	sbus_Handle sbus=(sbus_Handle) pvPortMalloc(sizeof(struct sbus_t));
	if (sbus != NULL) {

		sbus->Acel = 0;
		sbus->Direc = 0;
		sbus->Giro = 0;
		sbus->lost = 0;
		sbus->off = 0;

		memset(sbus->rx_buffer, 0, sizeof(sbus->rx_buffer));
		memset(sbus->tx_buffer, 0, sizeof(sbus->tx_buffer));
		memset(sbus->canales, 0, sizeof(sbus->canales));

		sbus->uartHandle = UART_Handle;

		return sbus;

	}

	return NULL;

}

uint8_t getFailsafe(sbus_Handle Handle) {
    if (Handle == NULL) return 1;
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

	return Handle->Acel;

};


uint16_t getDir(sbus_Handle Handle) {
	if (Handle == NULL) return 0;

	return Handle->Direc;


};


uint16_t getGiro(sbus_Handle Handle) {
	if (Handle == NULL) return 0;

	return Handle->Giro;

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

	if (Handle->rx_buffer[0] == 0x0F && Handle->rx_buffer[24] == 0x00 ) {

		Handle->canales[0] = (Handle->rx_buffer[1] | ( Handle->rx_buffer[2] & 0x07) << 8 );
		Handle->canales[1] = ((Handle->rx_buffer[2] >> 3 | Handle->rx_buffer[3] << 5 ) & 0x07FF);
		Handle->canales[2] = ((Handle->rx_buffer[3] >> 6 | Handle->rx_buffer[4] << 2  | Handle->rx_buffer[5] <<10) & 0x07FF);
		Handle->canales[3] = ((Handle->rx_buffer[5] >> 1 | Handle->rx_buffer[6] << 7) & 0x07FF);
		Handle->canales[4] = ((Handle->rx_buffer[6] >> 4 | Handle->rx_buffer[7] << 4) & 0x07FF);
		Handle->canales[5] = ((Handle->rx_buffer[7] >> 7 | Handle->rx_buffer[8] << 1 | Handle->rx_buffer[9] << 9) & 0x07FF);
		Handle->canales[6] = ((Handle->rx_buffer[9] >> 2 | Handle->rx_buffer[10] << 6 ) & 0x07FF);
		Handle->canales[7] = ((Handle->rx_buffer[10] >> 5 | Handle->rx_buffer[11] << 3 ) & 0x07FF);
		Handle->canales[8] = ((Handle->rx_buffer[12] | Handle->rx_buffer[13] << 8 ) & 0x07FF);
		Handle->canales[9] = ((Handle->rx_buffer[13] >> 3 | Handle->rx_buffer[14] << 5 ) & 0x07FF);


		Handle->lost = Handle->rx_buffer[23] & (1 <<2);
		Handle->off = Handle->rx_buffer[23] & (1 <<3);

		Handle->Direc = Handle->canales[0];
		Handle->Acel = Handle->canales[2];
		Handle->Giro = Handle->canales[3];


	}

}

USART_TypeDef* sbusGetUartHandle(sbus_Handle Handle){
	return  Handle->uartHandle->Instance;
}