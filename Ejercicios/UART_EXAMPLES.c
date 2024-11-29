#include "LPC17xx.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"

/*Declaracion de funciones*/
void ConfPin();
void ConfUart();

int main(){
	ConfPin();
	ConfUart();

	uint8_t info[] = "Hola mundo\t-\tElectrónica Digital 3\t-\tFCEFyN-UNC \n\r";
	while(1)
	{
		UART_Send(LPC_UART0 , sizeof(info) , BLOCKING); //QUE HACE BLOCKING?
	}

	return 0;
}

void ConfPin(){
	PINSEL_CFG_Type PinCfg;
	//P0.2 as TXD0
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 2;
	PinCfg.Funcnum = 1; //TXD0
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;//Normal

	PINSEL_ConfigPin(&PinCfg);

	//P0.3 as RXD0
	PinCfg.Pinnum = 3;

	PINSEL_ConfigPin(&PinCfg);
}


void ConfUart(){
	UART_CFG_Type UARTCfg;
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	//Configuracion por defecto
	UART_ConfigStructInit(&UARTCfg);

	//Inicializa el periferico
	UART_Init(LPC_UART0,&UARTCfg);

	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);

	//Inicializa FIFO
	UART_FIFOConfig(LPC_UART0 , &UARTFIFOConfigStruct); //Carga los valores del UART_FIFOConfigStructInit
	//Habilita transmision
	UART_TxCmd(LPC_UART0 , ENABLE);
}
