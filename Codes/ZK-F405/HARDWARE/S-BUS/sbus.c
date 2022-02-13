
#include "stm32f4xx.h" 
#include <stdbool.h>

//#include "debug_assert.h"
#include "sbus.h"
#include "delay.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define CPPM_TIMER                   TIM9
#define CPPM_TIMER_RCC               RCC_APB2Periph_TIM9
#define CPPM_GPIO_RCC                RCC_AHB1Periph_GPIOA
#define CPPM_GPIO_PORT               GPIOA
#define CPPM_GPIO_PIN                GPIO_Pin_3
#define CPPM_GPIO_SOURCE             GPIO_PinSource3
#define CPPM_GPIO_AF                 GPIO_AF_TIM9

#define CPPM_TIM_PRESCALER           (168-1) 

//SBUS硬件初始化
void SBUSInit(void)
{
	SBUS_Configuration();
}

/**
  * @name   SBUS_Configuration
  * @brief  Configure SBUS(Usart2) clock, gpio and nvic:
  *         SBUS_RX  USART2_RX  PA3
  * @param  None
  * @retval None
  */
void SBUS_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(CPPM_GPIO_RCC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = CPPM_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
//	/*PA9连接AF7，复用为USART1_TX*/
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	/*PA3连接AF?，复用为USART2_RX*/
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);


	//  波特率100000 8个数据位 偶校验位 2个停止位
	USART_InitStructure.USART_BaudRate = 100000;
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;

	USART_Init(USART2, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

	USART_Cmd(USART2, ENABLE);
}


uint8_t USART2_RX_BUF[26];

/**
  * @name   USART2_IRQHandler
  * @brief  This function handles USART2 Handler
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	uint8_t res;
	uint8_t clear = 0;
	static uint8_t Rx_Sta = 1;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		res =USART2->DR;
		USART2_RX_BUF[Rx_Sta++] = res;
	}
	else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		clear = USART2->SR;
		clear = USART2->DR;
		USART2_RX_BUF[0] = Rx_Sta - 1;
		Rx_Sta = 1;
	}
}
uint16_t CH[18];  // 通道值
uint8_t  rc_flag = 0;

void Sbus_Data_Count(uint8_t *buf)
{
	CH[ 0] = ((int16_t)buf[ 2] >> 0 | ((int16_t)buf[ 3] << 8 )) & 0x07FF;
	CH[ 1] = ((int16_t)buf[ 3] >> 3 | ((int16_t)buf[ 4] << 5 )) & 0x07FF;
	CH[ 2] = ((int16_t)buf[ 4] >> 6 | ((int16_t)buf[ 5] << 2 )  | (int16_t)buf[ 6] << 10 ) & 0x07FF;
	CH[ 3] = ((int16_t)buf[ 6] >> 1 | ((int16_t)buf[ 7] << 7 )) & 0x07FF;
	CH[ 4] = ((int16_t)buf[ 7] >> 4 | ((int16_t)buf[ 8] << 4 )) & 0x07FF;
	CH[ 5] = ((int16_t)buf[ 8] >> 7 | ((int16_t)buf[ 9] << 1 )  | (int16_t)buf[10] <<  9 ) & 0x07FF;
	CH[ 6] = ((int16_t)buf[10] >> 2 | ((int16_t)buf[11] << 6 )) & 0x07FF;
	CH[ 7] = ((int16_t)buf[11] >> 5 | ((int16_t)buf[12] << 3 )) & 0x07FF;
	
	CH[ 8] = ((int16_t)buf[13] << 0 | ((int16_t)buf[14] << 8 )) & 0x07FF;
	CH[ 9] = ((int16_t)buf[14] >> 3 | ((int16_t)buf[15] << 5 )) & 0x07FF;
	CH[10] = ((int16_t)buf[15] >> 6 | ((int16_t)buf[16] << 2 )  | (int16_t)buf[17] << 10 ) & 0x07FF;
	CH[11] = ((int16_t)buf[17] >> 1 | ((int16_t)buf[18] << 7 )) & 0x07FF;
	CH[12] = ((int16_t)buf[18] >> 4 | ((int16_t)buf[19] << 4 )) & 0x07FF;
	CH[13] = ((int16_t)buf[19] >> 7 | ((int16_t)buf[20] << 1 )  | (int16_t)buf[21] <<  9 ) & 0x07FF;
	CH[14] = ((int16_t)buf[21] >> 2 | ((int16_t)buf[22] << 6 )) & 0x07FF;
	CH[15] = ((int16_t)buf[22] >> 5 | ((int16_t)buf[23] << 3 )) & 0x07FF;
}





