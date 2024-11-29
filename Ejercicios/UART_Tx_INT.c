#include "LPC17xx.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"

void ConfPin(void);
void ConUart(void);
void UART0_IRQHandler(void);
void UART_IntTransmit(void);
void UART_IntReceive(void);

uint8_t info1[] = {0xC};//Limpia la FIFO
uint8_t info2[] = "FCEFyN - ";

int main(){
	ConfPin();
	ConfUart();
	//Hago un primer envio de datos, para iniciar comunicacion
	UART_Send(LPC_UART2 , info1 , sizeof(info1) , NONE_BLOCKING);
	while(1){}
}

void ConfPin(){
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

void ConfUart(){
	//Usa UART2
	UART_CFG_Type UARTCfg;
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	//Configuracion por defecto
	UART_ConfigStructInit(&UARTCfg);

	//Inicializa el periferico
	UART_Init(LPC_UART2,&UARTCfg);

	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);

	//Inicializa FIFO
	UART_FIFOConfig(LPC_UART2 , &UARTFIFOConfigStruct); //Carga los valores del UART_FIFOConfigStructInit
	//Habilita transmision
	UART_TxCmd(LPC_UART2 , ENABLE);
	//Habilita interrupcion Tx
	UART_IntConfig(LPC_UART2 , UART_INTCFG_THRE , ENABLE);//Habilita interrupcion por transimision
	//Habilita interrupcion por UART2
	NVIC_EnableIRQ(UART2_IRQn);
}


void UART2_IRQHandler(){
	uint32_t intsrc , tmp , bSent;
	static uint8_t conta = 0; //O sino la puedo declarar global
	//Determina la fuente de la interrupcion
	intsrc = UART_GetIntId(LPC_UART2);

	tmp = intsrc & UART_IIR_INTID_MASK;

	//Evalua si transimt holding esta vacio
	if(tmp == UART_IIR_INTID_THRE){
		if(conta < sizeof(info2))
		{
				bSent = UART_Send(LPC_UART2 , info2+conta , sizeof(info2) , NONE_BLOCKING);	//Si esta vacio, se vuelve a cargar la FIFO y se siguen mandando los datos
				conta = conta+bSent;
		}
		else
			{
				UART_IntConfig(LPC_UART2 , UART_INTCFG_THRE , DISABLE);
			}
	}
	//Hay que agregar logica para decir que se envie todo, ya que se usa un NONE_BLOCKIN
}



