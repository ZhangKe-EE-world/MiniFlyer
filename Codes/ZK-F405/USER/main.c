#include "sys.h"
#include "delay.h"
#include "string.h"



#include "FreeRTOS.h"
#include "task.h"
#include "timer.h"
//#include "queue.h"
//#include "semphr.h"
#include "event_groups.h"
#include "timers.h"

#include "myiic.h"
#include "led.h"
#include "mpu6050_iic.h"
#include "bmp280.h"
#include "pwm.h"
#include "Upper.h"
#include "sbus.h"
#include "flight_system.h"






//开始任务句柄
TaskHandle_t startTaskHandle;
static void startTask(void *arg);



//电调初始化
//任务优先级
#define ESCinit_TASK_PRIO		6
//任务堆栈大小	
#define ESCinit_STK_SIZE 		256  
//任务句柄
TaskHandle_t ESCinitTask_Handler;
//任务函数
void escinit_task(void *pvParameters);


//传感器数据获取和处理
//任务优先级
#define SENSORS_TASK_PRIO		5
//任务堆栈大小	
#define SENSORS_STK_SIZE 		512  
//任务句柄
TaskHandle_t SENSORS_Task_Handler;
//任务函数
void sensors_task(void *pvParameters);

//遥控数据获取和处理
//任务优先级
#define RC_TASK_PRIO		4
//任务堆栈大小	
#define RC_STK_SIZE 		512 
//任务句柄
TaskHandle_t RC_Task_Handler;
//任务函数
void RC_task(void *pvParameters);

//任务时间统计任务
//任务优先级
#define RUNTIMESTATS_TASK_PRIO	3
//任务堆栈大小	
#define RUNTIMESTATS_STK_SIZE 	512  
//任务句柄
TaskHandle_t RunTimeStats_Handler;
//任务函数
void RunTimeStats_task(void *pvParameters);




//事件句柄
static EventGroupHandle_t RCtask_Handle =NULL; 





//全局变量及标志位声明
FlightStatedef FlightSystemFlag;//飞行状态
char RunTimeInfo[400];		//保存任务运行时间信息


int main() 
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	delay_init(168);	/*delay初始化*/
	ledInit();			/*led初始化*/
	uart_init(500000);
	TIM3_PWM_Init();/*PWM初始化为50Hz*/
	SBUSInit();
	IIC_Init();
	while(	MPU_Init()	);
	Bmp_Init ();
	pid_param_Init();
	OffsetInit();
	
	//全局标志初始化
	FlightSystemFlag.all=0;
	
	xTaskCreate(startTask, "START_TASK", 300, NULL, 2, &startTaskHandle);	/*创建起始任务*/
	
	vTaskStartScheduler();	/*开启任务调度*/

	while(1){};
}




/*创建任务*/
void startTask(void *arg)
{
	taskENTER_CRITICAL();	/*进入临界区*/
	
	//创建事件句柄
	RCtask_Handle = xEventGroupCreate(); 
	
		//创建电调初始化任务
	xTaskCreate((TaskFunction_t )escinit_task,     	
							(const char*    )"escinit_task",   	
							(uint16_t       )ESCinit_STK_SIZE, 
							(void*          )NULL,				
							(UBaseType_t    )ESCinit_TASK_PRIO,	
							(TaskHandle_t*  )&ESCinitTask_Handler); 

		//创建传感器数据获取任务			
	xTaskCreate((TaskFunction_t )sensors_task,     	
							(const char*    )"sensors_task",   	
							(uint16_t       )SENSORS_STK_SIZE, 
							(void*          )NULL,				
							(UBaseType_t    )SENSORS_TASK_PRIO,	
							(TaskHandle_t*  )&SENSORS_Task_Handler); 	
							
											
	//创建RunTimeStats任务
	xTaskCreate((TaskFunction_t )RunTimeStats_task,     
							(const char*    )"RunTimeStats_task",   
							(uint16_t       )RUNTIMESTATS_STK_SIZE,
							(void*          )NULL,
							(UBaseType_t    )RUNTIMESTATS_TASK_PRIO,
							(TaskHandle_t*  )&RunTimeStats_Handler); 							
							
	//创建RC任务
	xTaskCreate((TaskFunction_t )RC_task,     
							(const char*    )"RC_task",   
							(uint16_t       )RC_STK_SIZE,
							(void*          )NULL,
							(UBaseType_t    )RC_TASK_PRIO,
							(TaskHandle_t*  )&RC_Task_Handler); 							
														
							
							
							
							
							
							
							
							
							
							
	printf("Free heap: %d bytes\n", xPortGetFreeHeapSize());			/*打印剩余堆栈大小*/
							
	vTaskDelete(startTaskHandle);										/*删除开始任务*/

	taskEXIT_CRITICAL();	/*退出临界区*/
} 






//ESC初始化任务函数 
void escinit_task(void *pvParameters)
{
	while(1)
	{
		LED0_ON;
		TIM_SetCompare4(TIM3,ESC_MAX);
		TIM_SetCompare3(TIM3,ESC_MAX);
		TIM_SetCompare2(TIM3,ESC_MAX);
		TIM_SetCompare1(TIM3,ESC_MAX);


		vTaskDelay(4000);//可酌情减短延时时间
		TIM_SetCompare4(TIM3,ESC_MIN);
		TIM_SetCompare3(TIM3,ESC_MIN);
		TIM_SetCompare2(TIM3,ESC_MIN);
		TIM_SetCompare1(TIM3,ESC_MIN);
		//油门行程校准和电调解锁
		
		xEventGroupSetBits(RCtask_Handle,Esc_Unlocked);//标志电调已解锁
		LED0_OFF;

		vTaskDelete (ESCinitTask_Handler);
	}   
}




//传感器处理任务
void sensors_task(void *pvParameters)
{
	double	BMP_Pressure;
	u8 report=0;

	u32 lastWakeTime = getSysTickCnt();
	while(1)
	{
		//5ms运行一次

		vTaskDelayUntil(&lastWakeTime, 5);

		BMP_Pressure = BMP280_Get_Pressure();
		MpuGetData(); //读取mpu数据并滤波
		GetAngle(&MPU6050,&Angle,0.005f);//加速度计和陀螺仪数据解算为欧拉角
		if(FlightSystemFlag.byte.FlightUnlock==1&&CH[2]>(RC_L1MIN-DELTA))//当解锁并且油门值大于最小值进入主状态控制
		{
			state_control(0.005f);
		}
		if(0)//调试用
		{
			printf("YAW：%f\nPITCH：%f\nROLL：%f\n",Angle.yaw,Angle.pitch,Angle.roll);
		}
		if(report)mpu6050_send_data(MPU6050.accX,MPU6050.accY,MPU6050.accZ,MPU6050.gyroX,MPU6050.gyroY,MPU6050.gyroZ);//用自定义帧发送加速度和陀螺仪数据，report决定是否开启
		if(report)usart1_report_imu(MPU6050.accX,MPU6050.accY,MPU6050.accZ,MPU6050.gyroX,MPU6050.gyroY,MPU6050.gyroZ,(int)(Angle.roll*100),(int)(Angle.pitch*100),(int)(Angle.yaw*10));

	}
}



//RunTimeStats任务
void RunTimeStats_task(void *pvParameters)
{
	u8 report=0;
	while(1)
	{
		//等电调解锁后开启
		xEventGroupWaitBits(RCtask_Handle, /* 事件对象句柄 */ 
												Esc_Unlocked,/* 接收任务感兴趣的事件 */ 
												pdFALSE, /* 退出时不清除事件位 */ 
												pdTRUE, /* 满足感兴趣的所有事件 */ 
												portMAX_DELAY);/* 指定超时时间,一直等 */ 
		vTaskDelay (500);
		if(report)
		{
			vTaskDelay (500);
			memset(RunTimeInfo,0,400);				//信息缓冲区清零
			vTaskGetRunTimeStats(RunTimeInfo);		//获取任务运行时间信息
			printf("任务名\t\t\t运行时间\t运行所占百分比\r\n");
			printf("%s\r\n",RunTimeInfo);
			
		}
		if(FlightSystemFlag.byte.FlightUnlock==1)
		{
			LED0_TOGGLE;
		}
		else
		{
//			LED0_OFF;
		}
	}
}

//RC任务
void RC_task(void *pvParameters)
{
	u8 t=0;
	u8 report=0;//遥控器通道上报开关
	u16 FlightUnlockCnt=0;
	u16 FlightLockCnt=0;
	u16 OffsetCnt=0;
	u32 lastWakeTime = getSysTickCnt();
	while(1)
	{
		//等电调解锁后开启
		xEventGroupWaitBits(RCtask_Handle, /* 事件对象句柄 */ 
												Esc_Unlocked,/* 接收任务感兴趣的事件 */ 
												pdFALSE, /* 退出时不清除事件位 */ 
												pdTRUE, /* 满足感兴趣的所有事件 */ 
												portMAX_DELAY);/* 指定超时时间,一直等 */ 
		vTaskDelayUntil(&lastWakeTime, 5);
		Sbus_Data_Count(USART2_RX_BUF);
		if(FlightSystemFlag.byte.FlightUnlock==0)//飞行锁定时
		{
			if(CH[2]<=(RC_L1MIN+DELTA)&&CH[3]>=(RC_L2MAX-DELTA))//左摇杆往右下打3s即可解锁飞行
			{
				FlightUnlockCnt++;
				if(FlightUnlockCnt>=SET_TIME)
				{
					xEventGroupSetBits(RCtask_Handle,Flight_Unlocked);//标志飞行已解锁
					FlightSystemFlag.byte.FlightUnlock=1;
					printf("Flight unlocked!!\n");
					FlightUnlockCnt=0;
				}
			}
			else
			{
				FlightUnlockCnt=0;
				if(CH[2]>=(RC_L1MAX-DELTA)&&CH[3]<=(RC_L2MIN+DELTA))//左摇杆往左上打3s
				{
					OffsetCnt++;
					if(OffsetCnt>=SET_TIME)
					{
						vTaskSuspend(SENSORS_Task_Handler);
						MpuGetOffset(); //校准
						OffsetCnt=0;
						LED0_ON;
						printf("offseted!\n");
						vTaskResume(SENSORS_Task_Handler);
					}
				}
				else
				{
					OffsetCnt=0;
				}
			}
		}
		else//飞行解锁时
		{
			if(CH[2]<=(RC_L1MIN+DELTA)&&CH[3]<=(RC_L2MIN+DELTA))//左摇杆往左下打3s即可锁定飞行
			{
				FlightLockCnt++;
				if(FlightLockCnt>=SET_TIME)
				{
					xEventGroupClearBits(RCtask_Handle,Flight_Unlocked);//清解锁标志
					FlightSystemFlag.byte.FlightUnlock=0;
					TIM_SetCompare1(TIM3,499);
					TIM_SetCompare2(TIM3,499);
					TIM_SetCompare3(TIM3,499);
					TIM_SetCompare4(TIM3,499);
					LED0_OFF;
					printf("Flight locked!!\n");
					FlightLockCnt=0;
				}
			}
			else
			{
				FlightLockCnt=0;
			}
		}
		if(CH[2]<=(RC_L1OFF+DELTA))//遥控器不在线时
		{
			FlightSystemFlag.byte.RCOnline=0;
		}
		else
			if(CH[2]>=(RC_L1MIN-DELTA))
			{
				FlightSystemFlag.byte.RCOnline=1;
			}
			else
			{
				printf("unknowing state!\n");
			}

		
		//通道数据上传
		if(report)
		{
			for(t=0;t<18;t++)
			{
				printf("%d--%d ",t,CH[t]);
			}
			printf("\n");
		}
	}
}






























void vApplicationIdleHook( void )
{

}
















