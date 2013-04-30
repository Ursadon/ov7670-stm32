#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_dcmi.h"
#include "ov7670reg.h"
#include "misc.h"
#include <stdio.h>
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
#define TIMEOUT_MAX              10000
#define HSI_STARTUP_TIMEOUT		 10000
#define picture_x 176
#define picture_y 144

__IO uint16_t RAM_Buffer[picture_x * picture_y];

void usart_init(void) {
	/* USARTx configured as follow:
	 - BaudRate = 115200 baud
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - Hardware flow control disabled (RTS and CTS signals)
	 - Receive and transmit enabled
	 */
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Enable UART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);

	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	USART_OverSampling8Cmd(USART2, ENABLE);
	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 1200000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(USART2, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(USART2, ENABLE);

}
void usPrintChar(char c) {
	uint8_t ch;
	ch = c;
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(USART2, (uint8_t) ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {
	}
}
void usartSendWord(uint16_t word) {
	uint8_t b1 = word & 0xFF;
	uint8_t b2 = word >> 8;
	usPrintChar(b1);
	usPrintChar(b2);
}



static ErrorStatus MCO_init(void) {
	ErrorStatus status = ERROR;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_4);
	return (status);
}

void delay(unsigned int ms) {
	//4694 = 1 ms
	while (ms > 1) {
		ms--;
		asm("nop");
	}
}

void DCMI_init() {
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	DCMI_InitTypeDef DCMI_InitStructure;
	//DCMI_CROPInitTypeDef DCMI_CROPInitStructure;

	__IO
	uint32_t Timeout = TIMEOUT_MAX;

	RCC_AHB1PeriphClockCmd(
			RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC
					| RCC_AHB1Periph_GPIOE, ENABLE);

	// Connect DCMI pins to AF13
	// PORTA
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI); // HSYNC
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI); // PCLK
	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_DCMI); // D0
	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_DCMI); // D1
	// PORTB
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI); // D5
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI); // VSYNC
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_DCMI); // D6
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_DCMI); // D7				   -
	// PORTC
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_DCMI); // D4
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI); // D0
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI); // D1
	// PORTE
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_DCMI); // D2
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_DCMI); // D3
	// DCMI GPIO configuration
	// D0..D1(PA9/10), HSYNC(PA4), PCLK(PA6)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// D5..D7(PB6/8/9), VSYNC(PB7)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8
			| GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// D4(PC11)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// D2..D3(PE0/1)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//---------------------------------------------------------------------------------------
	//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI); //d0
	//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI);	//d1
	//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_DCMI);	//d2

	//---------------------------------------------------------------------------------------
	// Configures the DCMI to interface with the OV7670 camera module
	// Enable DCMI clock
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);
	// Reinitialize
	DCMI_DeInit();

	DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_Continuous;
	DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_Low;
	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
	DCMI_Init(&DCMI_InitStructure);

	RCC_AHB1PeriphClockCmd(DMA_Camera_STREAM_CLOCK, ENABLE);
	DMA_DeInit(DMA_CameraToRAM_Stream);
	while (DMA_GetCmdStatus(DMA_CameraToRAM_Stream) != DISABLE) {
	}
	DMA_InitStructure.DMA_Channel = DMA_Camera_Channel;
	DMA_InitStructure.DMA_PeripheralBaseAddr = DCMI_DR_ADDRESS;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) RAM_Buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = picture_x * picture_y * 2 / 4;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA_CameraToRAM_Stream, &DMA_InitStructure);

	Timeout = TIMEOUT_MAX;
	while ((DMA_GetCmdStatus(DMA_CameraToRAM_Stream) != ENABLE)
			&& (Timeout-- > 0)) {
	}

	// Check if a timeout condition occurred
	if (Timeout == 0) {
		// Manage the error: to simplify the code enter an infinite loop
		while (1) {
			// Dopísa program
		}
	}
}

int main() {
	SystemInit();
	usart_init();
	I2CInit();
	MCO_init();
	DCMI_init();
    ov7670_init();

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DCMI_ITConfig(DCMI_IT_FRAME, ENABLE);
	DCMI_ITConfig(DCMI_IT_VSYNC, ENABLE);
	DCMI_ITConfig(DCMI_IT_LINE, ENABLE);
	// Enable DCMI Capture mode
	DCMI_Cmd(ENABLE);
	DCMI_CaptureCmd(ENABLE);

	// DMA Stream enable
	DMA_Cmd(DMA_CameraToRAM_Stream, ENABLE);
	unsigned int datachar[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }, it = 0, cmd, arg,
			data, jk;
	while (1) {
		while (USART_GetFlagStatus(USART2, USART_IT_RXNE) != RESET) {
			it++;
			if (it == 8) {
				it = 0;
			}
			datachar[it] = USART_ReceiveData(USART2);
			if (datachar[it] == '\n') {
				asm("nop");
			}
			if (datachar[it] == '\n' && datachar[it - 1] == '\r') {
				cmd = datachar[1];
				arg = datachar[2];
				data = datachar[3];
				switch (cmd) {
				case '1':
					DCMI_Cmd(ENABLE);
					DCMI_CaptureCmd(ENABLE);
					DMA_Cmd(DMA_CameraToRAM_Stream, ENABLE);
					it = 0;
					break;
				case '2':
					ov7670_set(arg, data);
					it = 0;
					break;
				default:
					it = 0;
					break;
				}
				it = 0;
				for (jk = 0; jk < 8; jk++) {
					datachar[jk] = 0;
				}
			}
		}
	}
}

int num_dcmi = 0;
int num_dcmi_frame = 0;
int num_dcmi_vsync = 0;
int num_dcmi_line = 0;

void DCMI_IRQHandler(void) {
	num_dcmi++;

	uint16_t i;
	if (DCMI_GetITStatus(DCMI_IT_FRAME)) {
		__disable_irq();
		for (i = 0; i < picture_x * picture_y; i++)
			usartSendWord(RAM_Buffer[i+2]);
		__enable_irq();
		DCMI_Cmd(DISABLE);
		DCMI_CaptureCmd(DISABLE);
		DMA_Cmd(DMA_CameraToRAM_Stream, DISABLE);

		num_dcmi_frame++;
		DCMI_ClearITPendingBit(DCMI_IT_FRAME);
		//printf(" lines: %d\n\r",num_dcmi_line);
	} else if (DCMI_GetFlagStatus(DCMI_FLAG_VSYNCRI) == SET) {
		num_dcmi_vsync++;
		DCMI_ClearFlag(DCMI_FLAG_VSYNCRI);
		//printf(" lines: %d\n\r",num_dcmi_line);
//		printf(" num_dcmi: %d\n\r",num_dcmi);
//		printf(" num_dcmi_frame: %d\n\r",num_dcmi_frame);
//		printf(" num_dcmi_vsync: %d\n\r",num_dcmi_vsync);
//		printf(" num_dcmi_line: %d\n\r",num_dcmi_line);
//		printf(" =========================\n\r");
		num_dcmi = 0;
		num_dcmi_frame = 0;
		num_dcmi_vsync = 0;
		num_dcmi_line = 0;
	} else if (DCMI_GetFlagStatus(DCMI_IT_LINE) == SET) {
		num_dcmi_line++;
		DCMI_ClearFlag(DCMI_IT_LINE);
	}
}
