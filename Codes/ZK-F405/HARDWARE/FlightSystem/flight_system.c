#include "stdio.h"

#include "sbus.h"
#include "flight_system.h" 

const float M_PI = 3.1415926535;
const float RtA = 57.2957795f;
const float AtR = 0.0174532925f;
const float Gyro_G = 0.03051756f*2;	  	//陀螺仪初始化量程+-2000度每秒于1 / (65536 / 4000) = 0.03051756*2		
const float Gyro_Gr = 0.0005326f*2;     //面计算度每秒,转换弧度每秒则 2*0.03051756	 * 0.0174533f = 0.0005326*2


PidObject pidRateX; //内环PID数据
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //外环PID数据
PidObject pidRoll;
PidObject pidYaw;

int16_t motor_PWM_Value[4];//

#define MOTOR1 motor_PWM_Value[0] 
#define MOTOR2 motor_PWM_Value[1] 
#define MOTOR3 motor_PWM_Value[2] 
#define MOTOR4 motor_PWM_Value[3] 
uint16_t low_thr_cnt;

PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw   //结构体数组，将每一个数组放一个pid结构体，这样就可以批量操作各个PID的数据了  比如解锁时批量复位pid控制数据，新手明白这句话的作用就可以了
};

//PID在此处修改
void pid_param_Init(void)//PID参数初始化
{
	pidRateX.kp = 3.f;
	pidRateY.kp = 3.f;
	pidRateZ.kp = 1.0f;
	
	pidRateX.ki = 0.05f;
	pidRateY.ki = 0.05f;
	pidRateZ.ki = 0.02f;	
	
	pidRateX.kd = 0.3f;
	pidRateY.kd = 0.3f;
	pidRateZ.kd = 0.3f;	
	
	pidPitch.kp = 10.0f;
	pidRoll.kp = 10.0f;
	pidYaw.kp = 8.0f;	
	
	pidRest(pPidObject,6); //批量复位PID数据，防止上次遗留的数据影响本次控制

	pidYaw.desired =  pidYaw.measured = 0;   //锁定偏航角
	
	
}



/**************************************************************
 *批量复位PID函数
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void pidRest(PidObject **pid,const uint8_t len)
{
	uint8_t i;
	for(i=0;i<len;i++)
	{
		pid[i]->integ = 0;
		pid[i]->prevError = 0;
		pid[i]->out = 0;
		pid[i]->offset = 0;
	}
}


/**************************************************************
 * Update the PID parameters.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if pidSetError() has been used.
 * @return PID algorithm output
 ***************************************************************/	
void pidUpdate(PidObject* pid,const float dt)
{
	float error;
	float deriv;
	
	error = pid->desired - pid->measured; //当前角度与实际角度的误差

	pid->integ += error * dt;	 //误差积分累加值

	//pid->integ = LIMIT(pid->integ,pid->IntegLimitLow,pid->IntegLimitHigh); //进行积分限幅

	deriv = (error - pid->prevError)/dt;  //前后两次误差做微分

	pid->out = pid->kp * error + pid->ki * pid->integ + pid->kd * deriv;//PID输出

	//pid->out = LIMIT(pid->out,pid->OutLimitLow,pid->OutLimitHigh); //输出限幅
	
	pid->prevError = error;  //更新上次的误差
		
}


/**************************************************************
 *  CascadePID
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void CascadePID(PidObject* pidRate,PidObject* pidAngE,const float dt)  //串级PID
{	 
	pidUpdate(pidAngE,dt);    //先计算外环
	pidRate->desired = pidAngE->out;
	pidUpdate(pidRate,dt);    //再计算内环	
}

/**************************************************************
 *  state_control
 * @param[in] 陀螺仪初始数据、融合后的欧拉角、运行间隔
 * @param[out] 
 * @return     
 ***************************************************************/	
void state_control(float gx,float gy,float gz,float pitch,float roll,float yaw,float dt)
{
	u16 Throttle=0;
	Throttle=(ESC_MAX-ESC_MIN)*(CH[2]-RC_L1MIN)/RC_RANGE+ESC_MIN;//获取基础油门值
	
	pidRateX.measured = gx * Gyro_G; //内环测量值 角度/秒
	pidRateY.measured = gy * Gyro_G;
	pidRateZ.measured = gz * Gyro_G;

	pidPitch.measured = pitch; //外环测量值 单位：角度
	pidRoll.measured = roll;
	pidYaw.measured = yaw;
	
	pidUpdate(&pidRoll,dt);    //调用PID处理函数来处理外环	横滚角PID		
	pidRateX.desired = pidRoll.out; //将外环的PID输出作为内环PID的期望值即为串级PID
	pidUpdate(&pidRateX,dt);  //再调用内环

	pidUpdate(&pidPitch,dt);    //调用PID处理函数来处理外环	俯仰角PID	
	pidRateY.desired = pidPitch.out;  
	pidUpdate(&pidRateY,dt); //再调用内环
	
	CascadePID(&pidRateZ,&pidYaw,dt);	//直接调用串级PID函数来处理
	
	MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(Throttle,499,849); //留100给姿态控制
	
	MOTOR1 +=    + pidRateX.out - pidRateY.out + pidRateZ.out;//; 姿态输出分配给各个电机的控制量
	MOTOR2 +=    - pidRateX.out - pidRateY.out - pidRateZ.out ;//;
	MOTOR3 +=    - pidRateX.out + pidRateY.out + pidRateZ.out;
	MOTOR4 +=    + pidRateX.out + pidRateY.out - pidRateZ.out;//;
	
//	MOTOR1 +=    + pidRateX.out - pidRateY.out;
//	MOTOR2 +=    - pidRateX.out - pidRateY.out;
//	MOTOR3 +=    - pidRateX.out + pidRateY.out;
//	MOTOR4 +=    + pidRateX.out + pidRateY.out;
	
	TIM_SetCompare1(TIM3,LIMIT(MOTOR1,499,949));
	TIM_SetCompare2(TIM3,LIMIT(MOTOR2,499,949));
	TIM_SetCompare3(TIM3,LIMIT(MOTOR3,499,949));
	TIM_SetCompare4(TIM3,LIMIT(MOTOR4,499,949));
	printf("1--%d\n2--%d\n3--%d\n4--%d\n",LIMIT(MOTOR1,499,949),LIMIT(MOTOR2,499,949),LIMIT(MOTOR3,499,949),LIMIT(MOTOR4,499,949));

}


