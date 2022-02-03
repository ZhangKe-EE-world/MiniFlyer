#include "nvic.h"
#include "led.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"		 
#include "task.h"



static u32 sysTickCnt=0;

void nvicInit(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}

extern void xPortSysTickHandler(void);

/********************************************************
 *SysTick_Handler()
 *滴答定时器中断服务函数
*********************************************************/
void  SysTick_Handler(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*系统已经运行*/
    {
        xPortSysTickHandler();	
    }else
	{
		sysTickCnt++;	/*调度开启之前计数*/
	}
}

/********************************************************
*getSysTickCnt()
*调度开启之前 返回 sysTickCnt
*调度开启之前 返回 xTaskGetTickCount()
*********************************************************/
u32 getSysTickCnt(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*系统已经运行*/
		return xTaskGetTickCount();
	else
		return sysTickCnt;
}


