
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


static xQueueHandle captureQueue;
static uint16_t prevCapureVal;
static bool captureFlag;
static bool isAvailible;


//cppm

static uint8_t currChannel = 0;
uint16_t rcData[CH_NUM];//捕获PPM的通道信号值
rcLinkState_t rcLinkState;
failsafeState_t  failsafeState;


void cppmInit(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(CPPM_GPIO_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(CPPM_TIMER_RCC, ENABLE);

	//配置PPM信号输入引脚（PA3）
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = CPPM_GPIO_PIN;
	GPIO_Init(CPPM_GPIO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(CPPM_GPIO_PORT, CPPM_GPIO_SOURCE, CPPM_GPIO_AF);

	//配置定时器1us tick
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = CPPM_TIM_PRESCALER;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(CPPM_TIMER, &TIM_TimeBaseStructure);

	//配置输入捕获
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(CPPM_TIMER, &TIM_ICInitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	captureQueue = xQueueCreate(64, sizeof(uint16_t));

	TIM_ITConfig(CPPM_TIMER, TIM_IT_Update | TIM_IT_CC2, ENABLE);
	TIM_Cmd(CPPM_TIMER, ENABLE);
}


bool cppmIsAvailible(void)
{
	return isAvailible;
}

int cppmGetTimestamp(uint16_t *timestamp)
{
//	ASSERT(timestamp);

	return xQueueReceive(captureQueue, timestamp, 20);
}

void cppmClearQueue(void)
{
	xQueueReset(captureQueue);
}

uint16_t capureVal;
uint16_t capureValDiff;

void TIM1_BRK_TIM9_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if(TIM_GetITStatus(CPPM_TIMER, TIM_IT_CC2) != RESET)
	{
		if(TIM_GetFlagStatus(CPPM_TIMER, TIM_FLAG_CC2OF) != RESET)
		{
		  //TODO: Handle overflow error
		}

		capureVal = TIM_GetCapture2(CPPM_TIMER);
		capureValDiff = capureVal - prevCapureVal;
		prevCapureVal = capureVal;

		xQueueSendFromISR(captureQueue, &capureValDiff, &xHigherPriorityTaskWoken);
 
		captureFlag = true;
		TIM_ClearITPendingBit(CPPM_TIMER, TIM_IT_CC2);
	}

	if(TIM_GetITStatus(CPPM_TIMER, TIM_IT_Update) != RESET)
	{
		// Update input status
		isAvailible = (captureFlag == true);
		captureFlag = false;
		TIM_ClearITPendingBit(CPPM_TIMER, TIM_IT_Update);
	}
}


void rxInit(void)
{
	cppmInit();
}

void ppmTask(void *param)
{
	uint16_t ppm;
	uint32_t currentTick;
	while(1)
	{
		currentTick = getSysTickCnt();
		
		if(cppmGetTimestamp(&ppm) == pdTRUE)//20ms阻塞式获取PPM脉冲值
		{
			if (cppmIsAvailible() && ppm < 2100)//判断PPM帧结束
			{
				if(currChannel < CH_NUM)
				{
					rcData[currChannel] = ppm;
				}
				currChannel++;
			}
			else//接收完一帧数据
			{
				currChannel = 0;
				rcLinkState.linkState = true;
				if (rcData[THROTTLE] < 950 || rcData[THROTTLE] > 2100)//无效脉冲，说明接收机输出了失控保护的值
					rcLinkState.invalidPulse = true;
				else
					rcLinkState.invalidPulse = false;
				rcLinkState.realLinkTime = currentTick;
			}
		}
		
		if (currentTick - rcLinkState.realLinkTime > 1000)//1S没接收到信号说明遥控器连接失败
		{
			rcLinkState.linkState = false;
		}	
	}
}

//void rxTask(void *param)
//{	
//	u32 tick = 0;
//	u32 lastWakeTime = getSysTickCnt();
//	uint32_t currentTick;
//	
//	while (1)
//	{
//		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));//1KHz运行频率
//		
//		if (RATE_DO_EXECUTE(RATE_50_HZ, tick))
//		{
//			currentTick = getSysTickCnt();
//			
//			//处理遥杆命令和辅助通道模式切换
//			if (rcLinkState.linkState == true)
//			{
////				processRcStickPositions();
////				processRcAUXPositions();
//			}
//			
//			//处理解锁状态下遥控失去连接
//			if (ARMING_FLAG(ARMED))
//			{
//				if (rcLinkState.linkState == false || rcLinkState.invalidPulse == true)//遥控失去连接或无效脉冲
//				{
//					if (failsafeState.failsafeActive == false)
//					{
//						if (currentTick > failsafeState.throttleLowPeriod )//有一段低油门时间（如5秒），说明飞机在地上可直接关闭电机
//						{
//							mwDisarm();
//						}
//						else 
//						{
//							failsafeState.failsafeActive = true;
//							commanderActiveFailsafe();//激活失控保护自动降落
//						}
//					}
//				}
//				else//遥控连接正常
//				{
//					throttleStatus_e throttleStatus = calculateThrottleStatus();
//					if (throttleStatus == THROTTLE_HIGH)
//					{
//						failsafeState.throttleLowPeriod = currentTick + 5000;//5000表示需要低油门的时间（5秒）
//					}
//				}
//			}
//			else
//			{
//				failsafeState.throttleLowPeriod = 0;
//			}
//			
//		}
//		tick++;
//	}
//}

