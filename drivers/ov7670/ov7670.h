#ifndef __OV7670_H
#define __OV7670_H

#include "stm32f4xx.h"

// DMA Stream parameters definitions. You can modify these parameters to select
// a different DMA Stream and/or channel.
// But note that only DMA2 Streams are capable of Memory to Memory transfers.
#define DMA_CameraToRAM_Stream   		DMA2_Stream7
#define DMA_Camera_Channel         		DMA_Channel_1
#define DMA_Camera_STREAM_CLOCK    		RCC_AHB1Periph_DMA2
#define DMA_Camera_STREAM_IRQ      		DMA2_Stream7_IRQn
#define DMA_Camera_IT_TCIF         		DMA_IT_TCIF7
#define DMA_Camera_STREAM_IRQHANDLER    DMA2_Stream7_IRQHandler
#define DCMI_DR_ADDRESS       0x50050028
#define picture_x 176
#define picture_y 144

__IO uint16_t RAM_Buffer[picture_x * picture_y];

uint8_t ov7670_get(uint8_t reg);
uint8_t ov7670_set(uint8_t reg, uint8_t data);
int ov7670_init();
void DCMI_init();
void DMA_init();

#endif
