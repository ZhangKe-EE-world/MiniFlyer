#ifndef __FS_H
#define __FS_H
#include "sys.h"
#include<stdio.h>

//飞行状态标志位
typedef union
{
    char all;
    struct{
					char FlightUnlock:1; //位域操作
					char RCOnline:1;
					char bits6:6;
					}byte;
}FlightStatedef;


#endif
