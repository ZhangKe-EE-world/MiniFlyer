#ifndef __FS_H
#define __FS_H
#include "sys.h"
#include "stdio.h"


#define DELTA 5
#define RC_L1MIN (312)
#define RC_L1MAX (1912)
#define RC_L2MIN (608)
#define RC_L2MAX (1408)
#define RC_R1MIN (312)
#define RC_R1MAX (1912)
#define RC_OFF	 (40)
#define RC_ON		 (312)
#define RC_RANGE (1600)


//参数宏
#define ESC_MAX (1000-1)
#define ESC_MIN (500-1)


//事件宏
#define Esc_Unlocked 			(0x01<<0)
#define Flight_Unlocked 	(0x01<<1)
#define RC_Connected		 	(0x01<<2)

#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )



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


//PID参数结构体
typedef volatile struct
{
	float desired;     //< set point
	float offset;      //
	float prevError;    //< previous error
	float integ;        //< integral
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	float IntegLimitHigh;       //< integral limit
	float IntegLimitLow;
	float measured;
	float out;
	float OutLimitHigh;
	float OutLimitLow;
}PidObject;


void pidRest(PidObject **pid,const uint8_t len);
void pid_param_Init(void);//PID参数初始化
void pidUpdate(PidObject* pid,const float dt);
void CascadePID(PidObject* pidRate,PidObject* pidAngE,const float dt);  //串级PID
void state_control(float gx,float gy,float gz,float pitch,float roll,float yaw,float dt);






#endif
