/*
 * UART_Rx.c
 *
 *  Created on: 29 nov. 2024
 *      Author: DELL
 *      Cuando se tengan datos para sacar del receptor, se genera una interrupcion
 *      y se extraen los datos
 *      Se usa el terminal para escribir y mandar caracteres ASCII de la compu a la lpc
 */

#include "LPC17xx.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"

void ConfPin(void);
void ConfUart(void);
void UART0_IRQHandler(void);
void UART_IntRecieve(void);

uint8_t info[1] = ""; //Guarda  los valores recibidos

int main(){
	ConfPin();
	ConfUart();
	while(1){} //Entra en un bucle infinito, ya que el unico evento que se produce es interrupcion por que hay
	//un dato nuevo en el receptor del UART
}

void ConfPin(void){
	PINSEL_CFG_Type PinCfg;

	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 10;
	PinCfg.Funcnum = 1;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;//Normal

	PINSEL_ConfigPin(&PinCfg);


	PinCfg.Pinnum = 11;

	PINSEL_ConfigPin(&PinCfg);
}


void ConfUart(void){


}

void UART2_IRQHandler(void){


}


