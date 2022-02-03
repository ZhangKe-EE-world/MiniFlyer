#ifndef __LED_H
#define __LED_H
#include "sys.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * LED驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define LED0_ON 		GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define LED0_OFF 		GPIO_SetBits(GPIOB, GPIO_Pin_9)
#define LED0_TOGGLE 	GPIO_ToggleBits(GPIOB, GPIO_Pin_9)


void ledInit(void);
void warningLedON(void);
void warningLedOFF(void);
void warningLedFlash(void);
void warningLedRefresh(void);
void warningLedUpdate(void);

#endif
