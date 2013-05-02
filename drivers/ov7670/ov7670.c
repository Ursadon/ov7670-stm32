#include "drivers/ov7670/ov7670.h"
#include "drivers/ov7670/ov7670reg.h"
#include "drivers/i2c/i2c.h"
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"

void delayx(unsigned int ms) {
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
	delayx(1000);
	I2C_start(I2C2, 0x43, I2C_Direction_Receiver);
	data = I2C_read_nack(I2C2);
	I2C_stop(I2C2);
	delayx(1000);
	return data;
}

uint8_t ov7670_set(uint8_t reg, uint8_t data) {
	I2C_start(I2C2, 0x42, I2C_Direction_Transmitter);
	I2C_write(I2C2, reg);
	I2C_write(I2C2, data);
	I2C_stop(I2C2);
	delayx(1000);
	return 0;
}

int ov7670_init() {
	int hstart = 456, hstop = 24, vstart = 14, vstop = 494;
	unsigned char v;

	if (ov7670_get(REG_PID) != 0x76) {
		return 1;
	}
	ov7670_set(REG_COM7, COM7_RESET); /* reset to default values */
	ov7670_set(REG_CLKRC, 0x01);
	ov7670_set(REG_COM7, COM7_FMT_VGA | COM7_YUV); /* output format: YUCV */

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
	ov7670_set(REG_COM14, COM14_DCWEN | 0x01);
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
	ov7670_set(0x72, 0x11);

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
	return 0;
}
void DMA_init() {
	DMA_InitTypeDef DMA_InitStructure;
	__IO
	uint32_t Timeout = HSE_STARTUP_TIMEOUT;

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

	Timeout = HSE_STARTUP_TIMEOUT;
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
}

void DCMI_init() {
	GPIO_InitTypeDef GPIO_InitStructure;

	DCMI_InitTypeDef DCMI_InitStructure;
	//DCMI_CROPInitTypeDef DCMI_CROPInitStructure;

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
}
