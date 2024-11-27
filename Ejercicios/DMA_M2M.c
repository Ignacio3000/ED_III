/*
Transferencia de una region de memoria FLASH a una regionde memoria RAM
El destino de memoria simpre va a ser RAM
Puede ser de RAM a RAM tambien
const uint32_t DMASrc_Buffer[DMA_SIZE] = 
{
    0x01020304, 0x05060708, 0x090A0B0C, 0x0D0E0F10,
	0x11121314, 0x15161718, 0x191A1B1C, 0x1D1E1F20,
	0x21222324, 0x25262728, 0x292A2B2C, 0x2D2E2F20,
	0x31323334, 0x35363738, 0x393A3D3C, 0x3D3E3F30
}
*/

#ifdef __USE_CMSIS
#include <LPC17xx.h>
#endif

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpdma.h"
 
#define DMA_SIZE 16 //Tamaño de la transferencia
//DMASrc_Buffer will be burn into flash when compile
const uint32_t DMASrc_Buffer[DMA_SIZE] = //Region de memoria fuente. Se encuentra en la flash
{
    0x01020304, 0x05060708, 0x090A0B0C, 0x0D0E0F10,
	0x11121314, 0x15161718, 0x191A1B1C, 0x1D1E1F20,
	0x21222324, 0x25262728, 0x292A2B2C, 0x2D2E2F20,
	0x31323334, 0x35363738, 0x393A3D3C, 0x3D3E3F30
};

uint32_t DMADst_Buffer[DMA_SIZE]; //Region de memoria de destino

//Terminal Counter Flag for Channel 0
volatile uint32_t Channel0_TC;
//Eror Counter Flag for Channel 0
volatile uint32_t Channel0_EC;


//Declaro funciones
void ConfPines();
void ConfDMA();
void Buffer_Verify();

int main(){
    ConfDMA();
    /*Reset Terminal Counter*/
    Channel0_TC = 0;
    /*Reset Error Counter*/
    Channel0_EC = 0;
    /*Enable DMA Interrupt*/
    NVIC_EnableIRQ(DMA_IRQn);
    /*Wait for DMA Processing complete*/
    while ((Channel0_EC == 0) & (Channel0_TC == 0))
    {
        
    };
    /*Verify Buffer*/
    Buffer_Verify();
    
    /*Loop infinito*/
    while (1)
    {
        return 0;
    }
    
}


ConfDMA(){
    /*Disable DMA Interrupt*/
    NVIC_DisableIRQ(DMA_IRQn);

    GPDMA_ChannelCmd(7 , ENABLE); //Canal 7

    /*GPDMA_LLI_Type GPDMA_LLI_Cfg;

    GPDMA_LLI_Cfg.SrcAddr = (uint32_t)(DMASrc_Buffer);
    GPDMA_LLI_Cfg.DstAddr = (uint32_t)(DMADst_Buffer);
    GPDMA_LLI_Cfg.NextLLI = (uint32_t)(GPDMA_LLI_Cfg);
    GPDMA_LLI_Cfg.Control = DMA_SIZE 
                            |(2<<18) //Source width = 32 bits 
                            |(2<<21) //Destination width = 32 bits
                            |(1<<27) //Dest Increment
                            |*/

    GPDMA_Init();

    GPDMA_Channel_CFG_Type GPDMAChannelCfg;

    GPDMAChannelCfg.ChannelNum = 7;//Low canal Priority
    GPDMAChannelCfg.TransferSize = DMA_SIZE;//16 datos
    GPDMAChannelCfg.TransferWidth = GPDMA_WIDTH_WORD; //Datos son de 32 bits
    GPDMAChannelCfg.SrcMemAddr = (uint32_t)DMASrc_Buffer;
    GPDMAChannelCfg.DstMemAddr = (uint32_t)DMADst_Buffer;
    GPDMAChannelCfg.TransferType = GPDMA_TRANSFERTYPE_M2M;

    GPDMAChannelCfg.SrcConn = 0; //unused
    GPDMAChannelCfg.DstConn = 0; //unused
    GPDMAChannelCfg.DMALLI = 0;//Unused

    GPDMA_Setup(&GPDMAChannelCfg); //Configura DMA
}

void Buffer_Verify(){
    uint8_t i;
    uint32_t *src_addr = (uint32_t*)DMASrc_Buffer;
    uint32_t *dst_addr = (uint32_t*)DMADst_Buffer;
    
    for(int i=0; i < DMA_SIZE; i++)
    {
        if( *src_addr++ != *dest_addr++)
        {
            while (1)
            {}
            
        }
    } 
}

void DMA_IRQHandler(){
    //Check DMA interrupt channel 7
    if (GPDMA_IntGetStatus(GPDMA_STAT_INT , 7) //Check interrupt status channel 7
    {
        //Check terminal counter status
        if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 7)
        {
            GPDMA_ClearIntPending(GPDMA_STAT_INTTC, 7);
            Channel0_TC++; //Sumo uno a bandera de interrupcion counter
        }
   
          //Check counter status
        if (GPDMA_IntGetStatus(GPDMA_STAT_INTERR , 7)
        {
            GPDMA_ClearIntPending(GPDMA_STAT_INTERR , 7); //Clear error counter interrupt pending
            Channel0_EC++;
        }  
    }
}
