#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dcmi.h"

#include "drivers/ov7670/ov7670.h"
#include "drivers/ov7670/ov7670reg.h"
#include "drivers/i2c/i2c.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "misc.h"
#include <stdio.h>
#include <string.h> /* memset */
#include <unistd.h> /* close */

static uint32_t speed = 10;
xSemaphoreHandle xSemPictureReady = NULL;

enum wake_packet {
	header = 0, address, command, num_bytes, data, crc
};
const unsigned char FEND = 0xC0, // Frame END
		FESC = 0xDB, // Frame ESCape
		TFEND = 0xDC, // Transposed Frame END
		TFESC = 0xDD; // Transposed Frame ESCape

const unsigned char crc8Table[256] = { 0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6,
		0x97, 0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E, 0x43, 0x72, 0x21,
		0x10, 0x87, 0xB6, 0xE5, 0xD4, 0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C,
		0x6D, 0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11, 0x3F, 0x0E, 0x5D,
		0x6C, 0xFB, 0xCA, 0x99, 0xA8, 0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63,
		0x52, 0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB, 0x3D, 0x0C, 0x5F,
		0x6E, 0xF9, 0xC8, 0x9B, 0xAA, 0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22,
		0x13, 0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6, 0xA5,
		0x94, 0x03, 0x32, 0x61, 0x50, 0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D,
		0x2C, 0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95, 0xF8, 0xC9, 0x9A,
		0xAB, 0x3C, 0x0D, 0x5E, 0x6F, 0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7,
		0xD6, 0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED, 0xC3, 0xF2, 0xA1,
		0x90, 0x07, 0x36, 0x65, 0x54, 0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F,
		0xAE, 0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17, 0xFC, 0xCD, 0x9E,
		0xAF, 0x38, 0x09, 0x5A, 0x6B, 0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3,
		0xD2, 0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28, 0x06, 0x37, 0x64,
		0x55, 0xC2, 0xF3, 0xA0, 0x91, 0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1,
		0xD0, 0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69, 0x04, 0x35, 0x66,
		0x57, 0xC0, 0xF1, 0xA2, 0x93, 0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B,
		0x2A, 0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1A,
		0x2B, 0xBC, 0x8D, 0xDE, 0xEF, 0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24,
		0x15, 0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC };

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
	uint16_t data;
	data = word;
	uint8_t b1 = data & 0xFF;
	uint8_t b2 = data >> 8;
	USART_SendData(USART2, (uint8_t) b1);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {
		taskYIELD();
	}

	USART_SendData(USART2, (uint8_t) b2);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {
		taskYIELD();
	}

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
void vLED(void *pvParameters) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14
			| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	for (;;) {
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		vTaskDelay(500);
		GPIO_SetBits(GPIOD, GPIO_Pin_13);
		vTaskDelay(500);
		GPIO_SetBits(GPIOD, GPIO_Pin_14);
		vTaskDelay(500);
		GPIO_SetBits(GPIOD, GPIO_Pin_15);
		vTaskDelay(500);
		GPIO_ResetBits(GPIOD,
				GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
		vTaskDelay(500);
	}
}

void vGetPicture(void *pvParameters) {
	uint16_t i;
	DCMI_CaptureCmd(ENABLE);
	// TODO: fix workaround
	DMA_Cmd(DMA_CameraToRAM_Stream, ENABLE);
	vSemaphoreCreateBinary(xSemPictureReady);
	for (;;) {
		if (xSemaphoreTake(xSemPictureReady, (portTickType) 1) == pdTRUE) {
			for (i = 0; i < picture_x * picture_y; i++) {
				usartSendWord(RAM_Buffer[i + 2]);
			}
		}
		taskYIELD();
	}
}

void packet_parser(uint8_t address, uint8_t cmd, uint8_t numofbytes, uint8_t *packet, uint8_t packet_crc) {
	uint8_t i;
	uint8_t crc = 0xFF;
	crc = crc8Table[crc ^ FEND];
	crc = crc8Table[crc ^ address];
	crc = crc8Table[crc ^ cmd];
	crc = crc8Table[crc ^ numofbytes];
	for (i = 0; i < numofbytes; i++) {
		crc = crc8Table[crc ^ packet[i]];
	}
	if (crc != packet_crc) {
		// CRC mismatch!
		return;
	}
	// Parse
	if (cmd == 1) {
		DCMI_CaptureCmd(ENABLE);
		// TODO: fix workaround
		DMA_Cmd(DMA_CameraToRAM_Stream, ENABLE);
	}
	if (cmd == 2) {
		ov7670_set(packet[0],packet[1]);
	}
}

void vScanUsart(void *pvParameters) {
	unsigned int usart_rx_buff, usart_rx_buff_temp;
	uint8_t wake_packet_status = header;
	uint8_t packet_started = 0;
	uint8_t wake_cmd = 0;
	uint8_t wake_data[256];
	uint8_t wake_header[4];
	uint8_t wake_data_iterator = 0;
	uint8_t wake_numofbytes = 0;
	for (;;) {
		if ((USART2->SR & USART_FLAG_RXNE) != (u16) RESET) {
			usart_rx_buff = USART_ReceiveData(USART2);
			if (usart_rx_buff == FEND) {
				// Получили FEND, но пакет ещё не окончен - сбрасываем данные
				if (packet_started == 1) {
					wake_data_iterator = 0;
					wake_packet_status = header;
					usart_rx_buff_temp = 0xFF;
					memset(&wake_data[0], 0, sizeof(wake_data));
					memset(&wake_header[0], 0, sizeof(wake_header));
				}
				// Получили FEND = пакет начался
				packet_started = 1;
			} else if (packet_started == 1) {
				// Byte destuffing
				if (usart_rx_buff == TFEND && usart_rx_buff_temp == FESC) {
					usart_rx_buff = FEND;
				} else if (usart_rx_buff == TFESC
						&& usart_rx_buff_temp == FESC) {
					usart_rx_buff = FESC;
				}
				usart_rx_buff_temp = usart_rx_buff;

				// Заполняем массив с заголовком
				switch (wake_packet_status) {
				case header:
					wake_header[address] = usart_rx_buff;
					wake_packet_status = command;
					break;
				case command:
					wake_header[command] = usart_rx_buff;
					wake_cmd = usart_rx_buff;
					wake_packet_status = num_bytes;
					break;
				case num_bytes:
					wake_header[num_bytes] = usart_rx_buff;
					wake_numofbytes = usart_rx_buff;
					wake_packet_status = data;
					break;
				case data:
					if (wake_data_iterator < wake_header[num_bytes]) {
						wake_data[wake_data_iterator] = usart_rx_buff;
						wake_data_iterator++;
					} else if (wake_data_iterator >= wake_header[num_bytes]) {
						wake_header[crc] = usart_rx_buff;
						// Изъять пакет, очистить переменныe
						packet_parser(wake_header[address], wake_cmd, wake_numofbytes, wake_data,
								wake_header[crc]);
						wake_data_iterator = 0;
						wake_packet_status = header;
						packet_started = 0;
						wake_cmd = 0;
						wake_numofbytes = 0;
						usart_rx_buff_temp = 0xFF;
						memset(&wake_data[0], 0, sizeof(wake_data));
						memset(&wake_header[0], 0, sizeof(wake_header));
					}
					break;
				default:
					break;
				}
			}
		}
		taskYIELD();
	}
}

int main() {
	SystemInit();
	// Booting...
	usart_init();
	MCO_init();
	I2CInit();
	ov7670_init();
	DCMI_init();
	DMA_init();
	// Boot completed!

//	speed = 36;
//	PWM_init();

	xTaskCreate( vLED, ( signed char * ) "vLED", configMINIMAL_STACK_SIZE, NULL,
			0, ( xTaskHandle * ) NULL);
	xTaskCreate( vGetPicture, ( signed char * ) "vGetPicture",
			configMINIMAL_STACK_SIZE, NULL, 0, ( xTaskHandle * ) NULL);
	xTaskCreate( vScanUsart, ( signed char * ) "vScanUsart",
			configMINIMAL_STACK_SIZE, NULL, 0, ( xTaskHandle * ) NULL);
	vTaskStartScheduler();
	return 0;
}

void DCMI_IRQHandler(void) {
	if (DCMI_GetITStatus(DCMI_IT_FRAME)) {
		xSemaphoreGiveFromISR( xSemPictureReady, NULL);
		// TODO: fix workaround
		DMA_Cmd(DMA_CameraToRAM_Stream, DISABLE);
		DCMI_ClearITPendingBit(DCMI_IT_FRAME);
	}
}
