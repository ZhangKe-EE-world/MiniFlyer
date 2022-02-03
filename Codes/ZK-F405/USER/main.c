#include "sys.h"
#include "delay.h"
#include "string.h"



#include "FreeRTOS.h"
#include "task.h"
#include "timer.h"
//#include "queue.h"
//#include "semphr.h"
//#include "event_groups.h"
#include "timers.h"

#include "myiic.h"
#include "led.h"
#include "mpu6050_iic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "bmp280.h"
#include "pwm.h"
#include "Upper.h"









TaskHandle_t startTaskHandle;
static void startTask(void *arg);



//电调初始化
//任务优先级
#define ESCinit_TASK_PRIO		10
//任务堆栈大小	
#define ESCinit_STK_SIZE 		256  
//任务句柄
TaskHandle_t ESCinitTask_Handler;
//任务函数
void escinit_task(void *pvParameters);


//传感器数据获取和处理
//任务优先级
#define SENSORS_TASK_PRIO		4
//任务堆栈大小	
#define SENSORS_STK_SIZE 		1024  
//任务句柄
TaskHandle_t SENSORS_Task_Handler;
//任务函数
void sensors_task(void *pvParameters);


//任务优先级
#define RUNTIMESTATS_TASK_PRIO	3
//任务堆栈大小	
#define RUNTIMESTATS_STK_SIZE 	1024  
//任务句柄
TaskHandle_t RunTimeStats_Handler;
//任务函数
void RunTimeStats_task(void *pvParameters);




char RunTimeInfo[400];		//保存任务运行时间信息

int main() 
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	delay_init(168);	/*delay初始化*/
	ledInit();			/*led初始化*/
	uart_init(500000);
	TIM3_PWM_Init();/*PWM初始化为50Hz*/
	IIC_Init();
	while(	MPU_Init()	);
	Bmp_Init ();
		while(mpu_dmp_init())
		{
			printf("dmp 初始化失败！-----%d\n",mpu_dmp_init());
		}
		printf("dmp 初始化成功！-----%d\n",mpu_dmp_init());
	
	xTaskCreate(startTask, "START_TASK", 300, NULL, 2, &startTaskHandle);	/*创建起始任务*/
	
	vTaskStartScheduler();	/*开启任务调度*/

	while(1){};
}




/*创建任务*/
void startTask(void *arg)
{
	taskENTER_CRITICAL();	/*进入临界区*/
	
	
	
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

		TIM_SetCompare1(TIM3,999);
		TIM_SetCompare2(TIM3,999);
		TIM_SetCompare3(TIM3,999);
		TIM_SetCompare4(TIM3,999);

		vTaskDelay(1000);
		TIM_SetCompare1(TIM3,499);
		TIM_SetCompare2(TIM3,499);
		TIM_SetCompare3(TIM3,499);
		TIM_SetCompare4(TIM3,499);
		vTaskDelay(1000);
		//油门行程校准和解锁

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
	u8 report=1;	
	u32 lastWakeTime = getSysTickCnt();
	while(1)
	{
		//4ms运行一次
		vTaskDelayUntil(&lastWakeTime, 4);
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
	u8 report=1;
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
































void vApplicationIdleHook( void )
{

}
















