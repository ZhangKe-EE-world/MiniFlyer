#ifndef __NVIC_H
#define __NVIC_H
#include "sys.h"


#define NVIC_LOW_PRI  13
#define NVIC_MID_PRI  10
#define NVIC_HIGH_PRI 7


void nvicInit(void);
u32 getSysTickCnt(void);	

#endif /* __NVIC_H */
