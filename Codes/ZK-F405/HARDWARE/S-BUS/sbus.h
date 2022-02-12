#ifndef __RX_H
#define __RX_H
#include "sys.h"
#include <stdbool.h>

#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (RATE_1000_HZ / RATE_HZ)) == 0)

enum 
{
	ROLL = 0,
	PITCH,
	THROTTLE,
	YAW,
	AUX1,
	AUX2,
	AUX3,
	AUX4,
	AUX5,
	AUX6,
	AUX7,
	AUX8,
	CH_NUM
};

extern uint16_t rcData[CH_NUM];

typedef struct 
{
	uint32_t realLinkTime;
	bool linkState;
	bool invalidPulse;
} rcLinkState_t;

typedef struct 
{
    bool rcLinkState;				//遥控连接状态（true连接 false断开）
	bool failsafeActive;			//失控保护是否激活
	uint32_t rcLinkRealTime;		//遥控连接实时时间
	uint32_t throttleLowPeriod;		//遥控需要低油门时间（用于判断飞机是否着陆状态）
} failsafeState_t;


void cppmInit(void);
bool cppmIsAvailible(void);
void cppmClearQueue(void);
int cppmGetTimestamp(uint16_t *timestamp);

void rxInit(void);
void ppmTask(void *param);
bool rxLinkStatus(void);
void rxTask(void *param);

#endif

				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				