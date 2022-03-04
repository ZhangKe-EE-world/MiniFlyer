#ifndef __FS_H
#define __FS_H
#include "sys.h"
#include "stdio.h"
#include "mpu6050_iic.h"
#include "stm32f4xx.h"  

#define DELTA 5
#define RC_L1MIN (312)
#define RC_L1MAX (1912)
#define RC_L1OFF	 (40)

#define RC_L2MIN (216)
#define RC_L2MAX (1812)

#define RC_R1MIN (120)
#define RC_R1MAX (1720)

#define RC_R2MIN (152)
#define RC_R2MAX (1752)

#define RC_RANGE (1600)

#define SET_TIME 600

//参数宏
#define ESC_MAX (8000-1)
#define ESC_MIN (4000-1)

//PID限幅
#define PIDMAX (1000.0f)
#define PIDMIN (-1000.0f)
#define PID_IMAX (500.0f)
#define PID_IMIN (-500.0f)

//欧拉角限制
#define ANGLE_MAX (30.0f)
#define ANGLE_MIN (-30.0f)


//事件宏
#define Esc_Unlocked 			(0x01<<0)
#define Flight_Unlocked 	(0x01<<1)
#define RC_Connected		 	(0x01<<2)

#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

/*参数保存地址配置*/
#define CONFIG_PARAM_SIZE	(1020*1024)
#define CONFIG_PARAM_ADDR 	(FLASH_BASE + CONFIG_PARAM_SIZE)	
//#define CONFIG_PARAM_ADDR 0x08010000

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
void GetAngle(const _st_Mpu *pMpu,_st_AngE *pAngE, float dt);
void state_control(float dt);
//void LockYaw(void);




#endif
