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
//#define picture_x 160
//#define picture_y 120
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

void I2CInit(void) {

	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Enable UART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitStructure.I2C_ClockSpeed = 400000;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	/* Enable I2C */
	I2C_Cmd(I2C2, ENABLE);
	I2C_Init(I2C2, &I2C_InitStructure);
}

void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction) {
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
		;
	I2C_GenerateSTART(I2Cx, ENABLE);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
		;
	I2C_Send7bitAddress(I2Cx, address, direction);
	if (direction == I2C_Direction_Transmitter) {
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
			;
	} else if (direction == I2C_Direction_Receiver) {
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
			;
	}
}

void I2C_write(I2C_TypeDef* I2Cx, uint8_t data) {
	I2C_SendData(I2Cx, data);
	// ждем I2C EV8_2 --> ask от ведомого
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;
}

uint8_t I2C_read_ack(I2C_TypeDef* I2Cx) {
	uint8_t data;
	// разрешаем ask после приема
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// ждем пока ведомый передаст байт
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
		;
	// возвращаем принятое
	data = I2C_ReceiveData(I2Cx);
	return data;
}

uint8_t I2C_read_nack(I2C_TypeDef* I2Cx) {
	uint8_t data;
	//запрещаем ask после приема
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	// ждем пока ведомый передаст байт
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
		;
	// возвращаем принятое
	data = I2C_ReceiveData(I2Cx);
	return data;
}

void I2C_stop(I2C_TypeDef* I2Cx) {
	//Отправляем STOP на линию
	I2C_GenerateSTOP(I2Cx, ENABLE);
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

uint8_t ov7670_get(uint8_t reg) {
	uint8_t data = 0;
	I2C_start(I2C2, 0x42, I2C_Direction_Transmitter);
	I2C_write(I2C2, reg);
	I2C_stop(I2C2);
	delay(1000);
	I2C_start(I2C2, 0x43, I2C_Direction_Receiver);
	data = I2C_read_nack(I2C2);
	I2C_stop(I2C2);
	delay(1000);
	return data;
}
uint8_t ov7670_set(uint8_t reg, uint8_t data) {
	I2C_start(I2C2, 0x42, I2C_Direction_Transmitter);
	I2C_write(I2C2, reg);
	I2C_write(I2C2, data);
	I2C_stop(I2C2);
	delay(1000);
	return 0;
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

	if (ov7670_get(REG_PID) != 0x76) {
		printf(" PANIC! REG_PID != 0x76!\n");
		while (1)
			;
	}
	ov7670_set(REG_COM7, COM7_RESET); /* reset to default values */
	ov7670_set(REG_CLKRC, 0x01);
	ov7670_set(REG_COM7, COM7_FMT_VGA | COM7_YUV); /* output format: YUCV */
	int hstart = 456, hstop = 24, vstart = 14, vstop = 494;
	unsigned char v;
	ov7670_set(REG_HSTART, (hstart >> 3) & 0xff);
	ov7670_set(REG_HSTOP, (hstop >> 3) & 0xff);
	v = ov7670_get(REG_HREF);
	v = (v & 0xc0) | ((hstop & 0x7) << 3) | (hstart & 0x7);
	ov7670_set(REG_HREF, v);

	ov7670_set(REG_VSTART, (vstart >> 2) & 0xff);
	ov7670_set(REG_VSTOP, (vstop >> 2) & 0xff);
	v = ov7670_get(REG_VREF);
	v = (v & 0xf0) | ((vstop & 0x3) << 2) | (vstart & 0x3);
	ov7670_set(REG_VREF, v);
	ov7670_set(REG_COM3, COM3_SCALEEN | COM3_DCWEN);
	ov7670_set(REG_COM14, COM14_DCWEN | 0x01); // divide by 4 [RFC]
	ov7670_set(0x73, 0xf1);
	ov7670_set(0xa2, 0x52);
	ov7670_set(0x7b, 0x1c);
	ov7670_set(0x7c, 0x28);
	ov7670_set(0x7d, 0x3c);
	ov7670_set(0x7f, 0x69);
	ov7670_set(REG_COM9, 0x38);
	ov7670_set(0xa1, 0x0b);
	ov7670_set(0x74, 0x19);
	ov7670_set(0x9a, 0x80);
	ov7670_set(0x43, 0x14);
	ov7670_set(REG_COM13, 0xc0);
	ov7670_set(0x70, 0x3A);
	ov7670_set(0x71, 0x35);
	ov7670_set(0x72, 0x11); // downsample by 2

	/* Gamma curve values */
	ov7670_set(0x7a, 0x20);
	ov7670_set(0x7b, 0x10);
	ov7670_set(0x7c, 0x1e);
	ov7670_set(0x7d, 0x35);
	ov7670_set(0x7e, 0x5a);
	ov7670_set(0x7f, 0x69);
	ov7670_set(0x80, 0x76);
	ov7670_set(0x81, 0x80);
	ov7670_set(0x82, 0x88);
	ov7670_set(0x83, 0x8f);
	ov7670_set(0x84, 0x96);
	ov7670_set(0x85, 0xa3);
	ov7670_set(0x86, 0xaf);
	ov7670_set(0x87, 0xc4);
	ov7670_set(0x88, 0xd7);
	ov7670_set(0x89, 0xe8);

	/* AGC and AEC parameters.  Note we start by disabling those features,
	 then turn them only after tweaking the values. */
	ov7670_set(REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT);
	ov7670_set(REG_GAIN, 0);
	ov7670_set(REG_AECH, 0);
	ov7670_set(REG_COM4, 0x40); /* magic reserved bit */
	ov7670_set(REG_COM9, 0x18); /* 4x gain + magic rsvd bit */
	ov7670_set(REG_BD50MAX, 0x05);
	ov7670_set(REG_BD60MAX, 0x07);
	ov7670_set(REG_AEW, 0x95);
	ov7670_set(REG_AEB, 0x33);
	ov7670_set(REG_VPT, 0xe3);
	ov7670_set(REG_HAECC1, 0x78);
	ov7670_set(REG_HAECC2, 0x68);
	ov7670_set(0xa1, 0x03); /* magic */
	ov7670_set(REG_HAECC3, 0xd8);
	ov7670_set(REG_HAECC4, 0xd8);
	ov7670_set(REG_HAECC5, 0xf0);
	ov7670_set(REG_HAECC6, 0x90);
	ov7670_set(REG_HAECC7, 0x94);
	ov7670_set(REG_COM8,
			COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT | COM8_AGC | COM8_AEC);

	/* Almost all of these are magic "reserved" values.  */
	ov7670_set(REG_COM5, 0x61);
	ov7670_set(REG_COM6, 0x4b);
	ov7670_set(0x16, 0x02);
	ov7670_set(REG_MVFP, 0x07);
	ov7670_set(0x21, 0x02);
	ov7670_set(0x22, 0x91);
	ov7670_set(0x29, 0x07);
	ov7670_set(0x33, 0x0b);
	ov7670_set(0x35, 0x0b);
	ov7670_set(0x37, 0x1d);
	ov7670_set(0x38, 0x71);
	ov7670_set(0x39, 0x2a);
	ov7670_set(REG_COM12, 0x78);
	ov7670_set(0x4d, 0x40);
	ov7670_set(0x4e, 0x20);
	ov7670_set(REG_GFIX, 0);
	ov7670_set(0x6b, 0x4a);
	ov7670_set(0x74, 0x10);
	ov7670_set(0x8d, 0x4f);
	ov7670_set(0x8e, 0);
	ov7670_set(0x8f, 0);
	ov7670_set(0x90, 0);
	ov7670_set(0x91, 0);
	ov7670_set(0x96, 0);
	ov7670_set(0x9a, 0);
	ov7670_set(0xb0, 0x84);
	ov7670_set(0xb1, 0x0c);
	ov7670_set(0xb2, 0x0e);
	ov7670_set(0xb3, 0x82);
	ov7670_set(0xb8, 0x0a);

	/* Matrix coefficients */
	ov7670_set(0x4f, 0x80);
	ov7670_set(0x50, 0x80);
	ov7670_set(0x51, 0);
	ov7670_set(0x52, 0x22);
	ov7670_set(0x53, 0x5e);
	ov7670_set(0x54, 0x80);
	ov7670_set(0x58, 0x9e);

	/* More reserved magic, some of which tweaks white balance */
	ov7670_set(0x43, 0x0a);
	ov7670_set(0x44, 0xf0);
	ov7670_set(0x45, 0x34);
	ov7670_set(0x46, 0x58);
	ov7670_set(0x47, 0x28);
	ov7670_set(0x48, 0x3a);
	ov7670_set(0x59, 0x88);
	ov7670_set(0x5a, 0x88);
	ov7670_set(0x5b, 0x44);
	ov7670_set(0x5c, 0x67);
	ov7670_set(0x5d, 0x49);
	ov7670_set(0x5e, 0x0e);
	ov7670_set(0x6c, 0x0a);
	ov7670_set(0x6d, 0x55);
	ov7670_set(0x6e, 0x11);
	ov7670_set(0x6f, 0x9f); /* "9e for advance AWB" */
	ov7670_set(0x6a, 0x40);
	ov7670_set(REG_BLUE, 0x40);
	ov7670_set(REG_RED, 0x60);
	ov7670_set(REG_COM8,
			COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT | COM8_AGC | COM8_AEC
					| COM8_AWB);

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
			usartSendWord(RAM_Buffer[i]);
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
