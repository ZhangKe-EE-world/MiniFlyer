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
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
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

//事件宏
#define Esc_Unlocked 			(0x01<<0)
#define Flight_Unlocked 	(0x01<<1)
#define RC_Connected		 	(0x01<<2)



//参数宏
#define ESC_MAX (1000-1)
#define ESC_MIN (500-1)

#define DELTA 5
#define RC_L1MIN (312+DELTA)
#define RC_L1MAX (1912-DELTA)
#define RC_L2MIN (608+DELTA)
#define RC_L2MAX (1408-DELTA)
#define RC_R1MIN (312+DELTA)
#define RC_R1MAX (1912-DELTA)


FlightStatedef FlightSystemFlag;//飞行状态
char RunTimeInfo[400];		//保存任务运行时间信息

int main() 
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	delay_init(168);	/*delay初始化*/
	ledInit();			/*led初始化*/
	uart_init(500000);
	TIM3_PWM_Init();/*PWM初始化为50Hz*/
	SBUSInit();//SBUS
	IIC_Init();
	while(	MPU_Init()	);
	Bmp_Init ();
	while(mpu_dmp_init())
	{
		printf("dmp 初始化失败！-----%d\n",mpu_dmp_init());
	}

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

		//创建传感器获取任务			
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

		TIM_SetCompare1(TIM3,ESC_MAX);
		TIM_SetCompare2(TIM3,ESC_MAX);
		TIM_SetCompare3(TIM3,ESC_MAX);
		TIM_SetCompare4(TIM3,ESC_MAX);

		vTaskDelay(1000);
		TIM_SetCompare1(TIM3,ESC_MIN);
		TIM_SetCompare2(TIM3,ESC_MIN);
		TIM_SetCompare3(TIM3,ESC_MIN);
		TIM_SetCompare4(TIM3,ESC_MIN);
		vTaskDelay(1000);
		//油门行程校准和电调解锁
		
		xEventGroupSetBits(RCtask_Handle,Esc_Unlocked);//标志电调已解锁
		LED0_OFF;
		vTaskDelete (ESCinitTask_Handler);
	}   
}




//传感器处理任务
void sensors_task(void *pvParameters)
{
	double	BMP_Pressure,BMP_Temperature;
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据 
	u8 report=0;	
	u32 lastWakeTime = getSysTickCnt();
	while(1)
	{
		//5ms运行一次,IIC速率较慢且底层采用的是while，导致该任务运行占用较长时间
		vTaskDelayUntil(&lastWakeTime, 5);
		BMP_Temperature = BMP280_Get_Temperature();
		BMP_Pressure = BMP280_Get_Pressure();
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)//需要速率足够大才能防止dmp队列溢出，故取队列频率最好和传感器DMP解算速率协调
		{ 
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//用自定义帧发送加速度和陀螺仪原始数据，report决定是否开启
			if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
		}
	}
}



//RunTimeStats任务
void RunTimeStats_task(void *pvParameters)
{
	u8 report=0;
	while(1)
	{
		if(report)
		{
			memset(RunTimeInfo,0,400);				//信息缓冲区清零
			vTaskGetRunTimeStats(RunTimeInfo);		//获取任务运行时间信息
			printf("任务名\t\t\t运行时间\t运行所占百分比\r\n");
			printf("%s\r\n",RunTimeInfo);
			vTaskDelay (2000);
		}
	}
}

//RC任务
void RC_task(void *pvParameters)
{
	u8 t=0;
	u8 report=0;
	u16 FlightUnlockCnt=0;
	u16 FlightLockCnt=0;
	u32 lastWakeTime = getSysTickCnt();
	while(1)
	{
		//等电调解锁后开启
		xEventGroupWaitBits(RCtask_Handle, /* 事件对象句柄 */ 
												Esc_Unlocked,/* 接收任务感兴趣的事件 */ 
												pdFALSE, /* 退出时不清除事件位 */ 
												pdTRUE, /* 满足感兴趣的所有事件 */ 
												portMAX_DELAY);/* 指定超时时间,一直等 */ 
		vTaskDelayUntil(&lastWakeTime, 50);
		Sbus_Data_Count(USART2_RX_BUF);
		
		if(FlightSystemFlag.byte.FlightUnlock==0)//飞行锁定时
		{
			if(CH[2]<=RC_L1MIN&&CH[3]>=RC_L2MAX)//左摇杆往右下打3s即可解锁飞行
			{
				FlightUnlockCnt++;
				if(FlightUnlockCnt>=60)
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
			}
		}
		else//飞行解锁时
		{
			if(CH[2]<=RC_L1MIN&&CH[3]<=RC_L2MIN)//左摇杆往左下打3s即可锁定飞行
			{
				FlightLockCnt++;
				if(FlightLockCnt>=60)
				{
					xEventGroupClearBits(RCtask_Handle,Flight_Unlocked);//清解锁标志
					FlightSystemFlag.byte.FlightUnlock=0;
					printf("Flight locked!!\n");
					FlightLockCnt=0;
				}
			}
			else
			{
				FlightLockCnt=0;
			}
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
















