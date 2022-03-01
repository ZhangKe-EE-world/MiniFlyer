#include "stdio.h"

#include "sbus.h"
#include "flight_system.h" 
#include "mpu6050_iic.h"
#include "myMath.h"


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

PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw   //结构体数组，将每一个数组放一个pid结构体，这样就可以批量操作各个PID的数据了  比如解锁时批量复位pid控制数据
};

//PID在此处修改
void pid_param_Init(void)//PID参数初始化
{
	pidRateX.kp = 5.0f;//内环P将四轴从偏差角速度纠正回期望角速度
	pidRateY.kp = 5.0f;
	pidRateZ.kp = 0.6f;
	
	pidRateX.ki = 5.0f;//内环I消除角速度控制静差
	pidRateY.ki = 5.0f;
	pidRateZ.ki = 0.6f;	
	
	pidRateX.kd = 0.07f;//内环D抑制系统运动,在偏差刚刚出现时产生很大的控制作用，加快系统响应速度，减少调整时间，从而改善系统快速性，并且有助于减小超调，克服振荡，从而提高系统稳定性，但不能消除静态偏差
	pidRateY.kd = 0.07f;
	pidRateZ.kd = 0.00225f;	
	

	pidRoll.kp = 0.5f;
	pidPitch.kp = 0.5f;//外环P将四轴从偏差角度纠正回期望角度
	pidYaw.kp = 0.1f;	


	pidRoll.ki = 0.0f;
	pidPitch.ki = 0.0f;//外环I消除角度控制静差
	pidYaw.ki = 0.0f;	
	
	
	pidRateX.IntegLimitHigh  = PID_IMAX;
	pidRateY.IntegLimitHigh = PID_IMAX;
	pidRateZ.IntegLimitHigh= PID_IMAX;
	
	pidRateX.OutLimitHigh  = PIDMAX;
	pidRateY.OutLimitHigh = PIDMAX;
	pidRateZ.OutLimitHigh= PIDMAX;
	
	pidPitch.IntegLimitHigh  = PID_IMAX;
	pidRoll.IntegLimitHigh = PID_IMAX;
	pidYaw.IntegLimitHigh= PID_IMAX;
	
	pidPitch.OutLimitHigh  = PIDMAX;
	pidRoll.OutLimitHigh = PIDMAX;
	pidYaw.OutLimitHigh= PIDMAX;
	
	pidRateX.IntegLimitLow  = PID_IMIN;
	pidRateY.IntegLimitLow = PID_IMIN;
	pidRateZ.IntegLimitLow= PID_IMIN;
	
	pidRateX.OutLimitLow  = PIDMIN;
	pidRateY.OutLimitLow = PIDMIN;
	pidRateZ.OutLimitLow= PIDMIN;
	
	pidPitch.IntegLimitLow  = PID_IMIN;
	pidRoll.IntegLimitLow = PID_IMIN;
	pidYaw.IntegLimitLow= PID_IMIN;
	
	pidPitch.OutLimitLow  = PIDMIN;
	pidRoll.OutLimitLow = PIDMIN;
	pidYaw.OutLimitLow= PIDMIN;
	
	pidRest(pPidObject,6); //批量复位PID数据，防止上次遗留的数据影响本次控制

	Angle.yaw = pidYaw.desired =  pidYaw.measured = 0;   //锁定偏航角
	
	
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

	pid->integ = LIMIT(pid->integ,pid->IntegLimitLow,pid->IntegLimitHigh); //进行积分限幅

	deriv = (error - pid->prevError)/dt;  //前后两次误差做微分

	pid->out = pid->kp * error + pid->ki * pid->integ + pid->kd * deriv;//PID输出

	pid->out = LIMIT(pid->out,pid->OutLimitLow,pid->OutLimitHigh); //输出限幅
	
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
 * @param[in] 滤波后的陀螺仪数据、融合后的欧拉角、运行间隔
 * @param[out] 
 * @return     
 ***************************************************************/	
void state_control(float dt)
{
	u16 Throttle=0;
	Throttle=(ESC_MAX-ESC_MIN)*(CH[2]-RC_L1MIN)/RC_RANGE+ESC_MIN;//获取基础油门值
	MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(Throttle,ESC_MIN,ESC_MAX-PIDMAX);//留给姿态控制一定量
	
	pidPitch.desired=(ANGLE_MAX-ANGLE_MIN)*(CH[1]-RC_R1MIN)/RC_RANGE+ANGLE_MIN;//根据最大角范围和遥控器设定角度
	pidRoll.desired=-((ANGLE_MAX-ANGLE_MIN)*(CH[0]-RC_R1MIN)/RC_RANGE+ANGLE_MIN);//根据最大角范围和遥控器设定角度
	
	if(pidPitch.desired>-2&&pidPitch.desired<2)
	{
		pidPitch.desired=0;
	}
	if(pidRoll.desired>-2&&pidRoll.desired<2)
	{
		pidRoll.desired=0;
	}
	
	printf("desire pitch is %f\ndesire roll is %f\n",pidPitch.desired,pidRoll.desired);
	
	if(CH[5]>=1780)//拨杆ch6下拨开启PID自稳
	{
		pidRateX.measured = MPU6050.gyroX * Gyro_G; //内环测量值 角度/秒
		pidRateY.measured = MPU6050.gyroY * Gyro_G;
		pidRateZ.measured = MPU6050.gyroZ * Gyro_G;

		pidPitch.measured = Angle.pitch; //外环测量值 单位：角度
		pidRoll.measured = Angle.roll;
		pidYaw.measured = Angle.yaw;
		
		pidUpdate(&pidRoll,dt);    //调用PID处理函数来处理外环	横滚角PID		
		pidRateX.desired = pidRoll.out; //将外环的PID输出作为内环PID的期望值即为串级PID
		pidUpdate(&pidRateX,dt);  //再调用内环

		pidUpdate(&pidPitch,dt);    //调用PID处理函数来处理外环	俯仰角PID	
		pidRateY.desired = pidPitch.out;  
		pidUpdate(&pidRateY,dt); //再调用内环

		CascadePID(&pidRateZ,&pidYaw,dt);	//直接调用串级PID函数来处理

		if(0)printf("PIDX：%f\tPIDY：%f\tPIDZ：%f\t\n",pidRateX.out,pidRateY.out,pidRateZ.out);
		
		MOTOR1 +=    + pidRateX.out - pidRateY.out - pidRateZ.out;// 姿态输出分配给各个电机的控制量
		MOTOR2 +=    - pidRateX.out - pidRateY.out + pidRateZ.out ;//
		MOTOR3 +=    - pidRateX.out + pidRateY.out - pidRateZ.out;
		MOTOR4 +=    + pidRateX.out + pidRateY.out + pidRateZ.out;//
	}
	else
	{
		pidRest(pPidObject,6);
	}



	
	TIM_SetCompare1(TIM3,LIMIT(MOTOR1,ESC_MIN,ESC_MAX));
	TIM_SetCompare2(TIM3,LIMIT(MOTOR2,ESC_MIN,ESC_MAX));
	TIM_SetCompare3(TIM3,LIMIT(MOTOR3,ESC_MIN,ESC_MAX));
	TIM_SetCompare4(TIM3,LIMIT(MOTOR4,ESC_MIN,ESC_MAX));
	if(1)printf("1--%d\t2--%d\t3--%d\t4--%d\t\n",LIMIT(MOTOR1,ESC_MIN,ESC_MAX),LIMIT(MOTOR2,ESC_MIN,ESC_MAX),LIMIT(MOTOR3,ESC_MIN,ESC_MAX),LIMIT(MOTOR4,ESC_MIN,ESC_MAX));

}


