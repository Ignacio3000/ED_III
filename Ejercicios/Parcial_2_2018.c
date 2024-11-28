/*1.
    Utilizando CMSIS escriba y comente un código que genere una onda del tipo trapezoidal a la salida del DAC. Para ello el DAC comenzará, a partir de cero, a incrementar su valor de a un bits hasta llegar a un
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

#define MEMORIA_ADRESS ((uint32_t*)(0x10004000)) // Memory Adress

#define valor_max (( *MEMORIA_ADRESS) & (0xFF) ) // Maximum DAC value

#define t_max (( *MEMORIA_ADRESS >> 8) & (0xFF)) // Time at maximum value

#define t_alto (( *MEMORIA_ADRESS >> 16) & (0xFF)) //Increment step delay

#define t_bajo (( *MEMORIA_ADRESS >> 24) & (0xFF)) //Decrement step delay

#define DMA_SIZE 10

//Global Variables
uint16_t VALOR_DAC; //DAC Value  
uint16_t FLAG_BAJADA = 0; //Descending flag


//Fuction to configurate DAC pins
void ConfPines(){
    PINSEL_CFG_Type Pincfg;
           
    Pincfg.Pinnum = 26;
    Pincfg.Portnum = 0;
    Pincfg.Pinmode = PINSEL_FUNC_2; //Set as DAC output

    PINSEL_ConfigPin(&Pincfg); //Cargo los parametros de la estructura
}


void Conf_DAC(){  
    VALOR_DAC = 0; //Inicializa valor a cargar a DAC 
    DAC_Init(LPC_DAC); //Inicializa DAC

    

    DAC_CONVERTER_CFG_Type DAC_ConverCfg;
    DAC_ConverCfg.CNT_ENA = ENABLE; //Habilita time Out counter
    DAC_ConverCfg.DMA_ENA = ENABLE; //Hablita DMA
    DAC_ConfigDAConverterControl(LPC_DAC , &DAC_ConverCfg);
    
    DAC_UpdateValue(LPC_DAC, VALOR_DAC);
    DAC_SetDMATimeOut(LPC_DAC , t_alto);

    /*Habilitacion interrupcion DMA*/
    NVIC_EnableIRQ(DMA_IRQn); 
}

int main(){
    ConfPines();
    Conf_DAC();
    while (1)
    {
        /* code */
    }
    
}

void DMA_IRQHandler(){
    if (FLAG_BAJADA)
    {
        
        if (LPC_DAC -> DACCNTVAL != t_bajo)
        {
            DAC_SetDMATimeOut(LPC_DAC , t_bajo); //Carga bits 24 a 31
        }

        if(VALOR_DAC > 0)
        {
            VALOR_DAC--; //Decrementa el valor del DAC
            DAC_UpdateValue(LPC_DAC, VALOR_DAC);
        }
        else if (VALOR_DAC == 0)
        {
            /* End of WAVEFORM generation */
            NVIC_DisableIRQ(DMA_IRQn);
            FLAG_BAJADA = 0;
        }
        /*else
        {
            GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC , 0);
        } */        
    }
    
    else //Ascending or holding phase
    {
        if(VALOR_DAC < valor_max)
        {
            VALOR_DAC++;
            DAC_UpdateValue(LPC_DAC, VALOR_DAC);//Update DAC
        }
        else if(VALOR_DAC == valor_max)
        { //Hold at maximum
            DAC_SetDMATimeOut(LPC_DAC, t_max); //Set maximum hold delay
            FLAG_BAJADA = 1;     
        }
    }

    GPDMA_ClearIntPending(GPDMA_STATCLR_INTERR, 0); //Clear DMA Interrupt 
}



















