/*Desde una region de memoria, transimitir una onda senoidal al DAC. En la region de memoria se tienen 60 datos.
Se usan para conformar la cantidad de datos de una onda senoidal. La frecuencia de un ciclo debe ser de 50Hz.
 uint32_t sin_0_to_90_16_samples[16] = {
        0,1045,2079,3090,4067, 
        5000,5877,6691,7431,8090,
        8660,9135,9510,9781,9945,10000
    };
*/

#ifdef __USE_CMSIS
#include <LPC17xx.h>
#endif

#ifdef __USE_MCUEXPRESSO
#include <cr_section_macros.h>
#endif

#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_pinsel.h"

#define DMA_SIZE 60
#define NUM_SINE_SAMPLE 60
#define SINE_FREQ_IN_HZ 60
#define PCLK_DAC_IN_MHZ 25 //CCLK divided by 4

uint32_t dac_sine_lut[NUM_SINE_SAMPLE]; //SrcAddr
//uint32_t tmp = 0; //TIME OUT DAC



void ConfPines();
void ConfDAC();
void ConfDMA();

int main(){

    uint32_t sin_0_to_90_16_samples[16] = {\
        0,1045,2079,3090,4067,\ 
        5000,5877,6691,7431,8090,\
        8660,9135,9510,9781,9945,10000\
    };

    ConfPines();
    ConfDAC();

    //Prepare DMA SINE look up table
    for (int i = 0; i < NUM_SINE_SAMPLE; i++)
    {
        if (i <= 15)
        {
            dac_sine_lut[i] = 512 + 512 * sin_0_to_90_16_samples[i] / 1000;
            if (i==15)
            {
                dac_sine_lut[i] = 1023;/* code */
            }
            
        }   
        else if (i <= 30)
        {
            dac_sine_lut[i] = 512 + 512 * sin_0_to_90_16_samples[30 - i] / 1000;
        }
        else if (i <= 45)
        {
            dac_sine_lut[i] = 512 - 512 * sin_0_to_90_16_samples[i - 30]/1000;
        }
        else if(i <= 60){
            dac_sine_lut[i] = 512 - 512 * sin_0_to_90_16_samples[60 - i]/1000;
            if (i == 60)
            {
                dac_sine_lut[i] = 512;
            }
        }
        
    }
    

    ConfDMA();
    while(1){}
}

void ConfPines(){
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 26;
    PinCfg.Funcnum = 2; // AOUT
    PinCfg.Pinmode = 0;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;

    PINSEL_ConfigPin(&PinCfg);
}

void ConfDMA(){
    //Configuracion lista linkeada
    GPDMA_LLI_Type GPDMA_LLI_Struct;

    GPDMA_LLI_Struct.SrcAddr = (uint32_t)dac_sine_lut;
    GPDMA_LLI_Struct.DstAddr = (uint32_t) & (LPC_DAC -> DACR);//Destino: DAC
    GPDMA_LLI_Struct.NextLLI = (uint32_t) & (GPDMA_LLI_Struct);
    GPDMA_LLI_Struct.Control = DMA_SIZE 
                                |(2 << 18) //SWidth = 32bit
                                |(2 << 21) //DWidth = 32bit
                                |(1 << 26) //Source Increment
                                ;
    
    GPDMA_Init(); //Inicializacion Modulo DMA Limpian banderas y canales   
   
    //Configuracion canal DMA
    GPDMA_Channel_CFG_Type GPDMAChannelCfg;
    
    GPDMAChannelCfg.ChannelNum = 0; //Canal 0
    GPDMAChannelCfg.TransferSize = DMA_SIZE; 
    GPDMAChannelCfg.TransferWidth = 0; //Unused - solo cuando es de M2M
    GPDMAChannelCfg.SrcMemAddr = (uint32_t)dac_sine_lut;
    GPDMAChannelCfg.TransferType = GPDMA_TRANSFERTYPE_M2P; //Memoria a periferico
    GPDMAChannelCfg.SrcConn = 0; //Unused-- Src es MEMORIA
    GPDMAChannelCfg.DstConn = GPDMA_CONN_DAC; //Destino DAC 
    GPDMAChannelCfg.DMALLI = (uint32_t) & GPDMA_LLI_Struct;

    GPDMA_Setup(&GPDMAChannelCfg); 
    //Habilitacion de canal 
    GPDMA_ChannelCmd(0 ,ENABLE);
}


void ConfDAC(){
    uint32_t tmp; //TIME OUT DAC
    
    DAC_CONVERTER_CFG_Type DAC_ConverterCfg;

    DAC_ConverterCfg.CNT_ENA = ENABLE;
    DAC_ConverterCfg.DMA_ENA = ENABLE;
    

    DAC_Init(LPC_DAC);

    //TIME OUT
    tmp = (PCLK_DAC_IN_MHZ * 1000000)/(SINE_FREQ_IN_HZ * NUM_SINE_SAMPLE) ;
    DAC_SetDMATimeOut(LPC_DAC , tmp);
    DAC_ConfigDAConverterControl(LPC_DAC , &DAC_ConverterCfg); //Primero inicializo y luego configuro
}
