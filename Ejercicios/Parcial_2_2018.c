/*1.
    Utilizando CMSIS escriba y comente un código que genere una onda del tipo trapezoidal a la salida del DAC como se
muestra en la figura. Para ello el DAC comenzará, a partir de cero, a incrementar su valor de a un bits hasta llegar a un
valor máximo que se mantendrá un tiempo dado y luego decrementará de a un bits hasta volver a cero nuevamente. Los
controles del valor máximo y los tiempos de duración de los escalones de subida y bajada están organizados en la
posición de memoria 0x10004000 de la siguiente forma:
bits 0 a 7: valor máximo a alcanzar por el DAC.
bits 8 a 15: valor a utilizar en una función de demora que define el tiempo que se mantiene el valor máximo.
bits 16 a 23: valor a utilizar en una función de demora para definir el tiempo que se mantiene cada incremento de 1 bits
en la subida.
bits 24 a 31: valor a utilizar en una función de demora para definir el tiempo que se mantiene cada decremento de 1 bits
en bajada.
*/

#ifdef __USE_CMSIS
#include <LPC17xx.h>
#endif

#ifdef __USE_MCUEXPRESSO
#include <cr_section_macros.h> /* MCUXpresso-specific macros */
#endif

#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_pinsel.h"

#define MEMORIA_ADRESS 0x10004000 // Memory Adress
#define valor_max ((0xFF) & ((uint32_t) * MEMORIA_ADRESS)) //Valor maximo a alcanzar por el DAC
#define t_max ((0xFF << 8) & ((uint32_t) * MEMORIA_ADRESS)) //Tiempo del valor maximo
#define t_alto ((0xFF << 16) & ((uint32_t) * MEMORIA_ADRESS)) //Tiempo en la subida
#define t_bajo ((0xFF << 32) & ((uint32_t) * MEMORIA_ADRESS)) //Tiempo en la bajada
#define DMA_SIZE 10


uint16_t VALOR_DAC = 0; //Inicializa el valor a cargar al DAC  




void ConfPuertos(){
    PINSEL_CFG_Type Pincfg;
    //Estructura que configura P0.26 como salida del DAC        
    Pincfg.Pinnum = 26;
    Pincfg.Portnum = 0;
    Pincfg.Pinmode = PINSEL_FUNC_2; //Func DAC

    PINSEL_ConfigPin(&Pincfg); //Cargo los parametros de la estructura
}


void Conf_DAC(){   
    DAC_Init(LPC_DAC); //Inicializa DAC

    DAC_CONVERTER_CFG_Type DAC_ConverCfg;
    //Configuracion del DAC 
    DAC_ConverCfg.CNT_ENA = ENABLE; //Habilita time Out counter
    DAC_ConverCfg.DMA_ENA = ENABLE; //Hablita DMA
    
    DAC_ConfigDAConverterControl(LPC_DAC , &DAC_ConverCfg);
    DAC_SetDMATimeOut(LPC_DAC , t_alto);
}

void Conf_DMA(){
    //HAGO QUE DMA INTERRUMPA ANTE CADA CONVERSION
    GPDMA_Init(); //Primero inicializo DMA

    GPDMA_Channel_CFG_Type DMAChannelCfg;

    DMAChannelCfg.ChannelNum = 0;
    DMAChannelCfg.TransferSize = DMA_SIZE; //10 bits del dac
    DMAChannelCfg.TransferWidth = 0; //Usado solo cuando es de memomria a memoria
    DMAChannelCfg.SrcMemAddr = ;
    DMAChannelCfg.DstMemAddr =  
}

void dma_handler(){
    //Pregunto si llego al valor maximo

}



















