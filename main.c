#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dcmi.h"

#include "drivers/ov7670/ov7670.h"
#include "drivers/ov7670/ov7670reg.h"
#include "drivers/i2c/i2c.h"
#include "misc.h"
#include <stdio.h>

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

void delay(unsigned int ms) {
	//4694 = 1 ms
	while (ms > 1) {
		ms--;
		asm("nop");
	}
}

static ErrorStatus MCO_init(void) {
	ErrorStatus status = ERROR;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	RCC_MCO2Config(RCC_MCO2Source_PLLCLK, RCC_MCO2Div_5);

	delay(9999);
	return (status);
}
static uint32_t speed = 10;
void PWM_init(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_DeInit(TIM1);
	// Start Timer 1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	//Enable port E clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	// Configure PE9 as output for PWM
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Connect PE9 to Timer 1 channel 1
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);

	TIM_TimeBaseStructure.TIM_Period = 716;
	TIM_TimeBaseStructure.TIM_Prescaler = 3906;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = speed;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	/* TIM1 enable counter */
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

int main() {
	SystemInit();
	speed = 36;
	PWM_init();

	usart_init();
	MCO_init();
	//I2CInit();
	//ov7670_init();
	//DCMI_init();
	//DMA_init();

	//set_pos(50);

	unsigned int datachar[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }, it = 0, cmd, arg,
			data, jk;
	while (1) {
		while (USART_GetFlagStatus(USART2, USART_IT_RXNE) != RESET) {
//			it++;
//			if (it == 8) {
//				it = 0;
//			}
//			datachar[it] = USART_ReceiveData(USART2);
//			if (datachar[it] == '\n') {
//				asm("nop");
//			}
//			if (datachar[it] == '\n' && datachar[it - 1] == '\r') {
//				cmd = datachar[1];
//				arg = datachar[2];
//				data = datachar[3];
//				switch (cmd) {
//				case '1':
//					DCMI_Cmd(ENABLE);
//					DCMI_CaptureCmd(ENABLE);
//					DMA_Cmd(DMA_CameraToRAM_Stream, ENABLE);
//					it = 0;
//					break;
//				case '2':
//					ov7670_set(arg, data);
//					it = 0;
//					break;
//				case '3':
//					PWM_init(arg * 3);
//					it = 0;
//					break;
//				default:
//					break;
//				}
//				it = 0;
//				for (jk = 0; jk < 8; jk++) {
//					datachar[jk] = 0;
//				}
//			}
			speed =  USART_ReceiveData(USART2);
			PWM_init();
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
			usartSendWord(RAM_Buffer[i + 2]);
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
