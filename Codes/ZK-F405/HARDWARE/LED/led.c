#include "led.h"
#include "delay.h"


typedef enum
{
    WARNING_LED_OFF = 0,
    WARNING_LED_ON,
    WARNING_LED_FLASH
} warningLedState_e;

static warningLedState_e warningLedState = WARNING_LED_OFF;

void ledInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
    GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

void warningLedON(void)
{
    warningLedState = WARNING_LED_ON;
}

void warningLedOFF(void)
{
    warningLedState = WARNING_LED_OFF;
}

void warningLedFlash(void)
{
    warningLedState = WARNING_LED_FLASH;
}

void warningLedRefresh(void)
{
    switch (warningLedState) 
	{
        case WARNING_LED_OFF:
            LED0_OFF;
            break;
        case WARNING_LED_ON:
            LED0_ON;
            break;
        case WARNING_LED_FLASH:
            LED0_TOGGLE;
            break;
    }
}



