#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "mpu6050.h"
#include "i2c.h"
#include "time.h"
#include "Filter.h"
#include "tasks.h"
#include "data_struct.h"
#include "hmc5883l.h"
#include "pwm_in.h"
#include "motor.h"
#include "pid_control.h"

int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);//设置系统中断优先级分组3
	
	delay_init(84);  				//初始化延时函数
	
	uart2_init(115200);			//初始化串口波特率为115200，用于与上位机之间通信
	
	LED_Init();							//初始化LED
	
	I2c_Soft_Init();				//初始化IIC总线
	
	MPU6050_Init(42);   		//加速度计、陀螺仪初始化，设置42hz低通滤波
	
	HMC5883L_Init();				//地磁计HMC5883L初始化，进行地磁计校准
	
	MPU6050_Offset_Init();	//MPU6050加速度计、陀螺仪零偏校准

	PWM_IN_Init();					//接收机采集初始化
	
	Motor_Init(2500-1, 84-1);	//电机初始化,周期为2.5ms的4路PWM输出
	
	TIM5_Init(1000-1,84-1);	//用于产生1ms一次的计时中断，用作时基
	
	Calculate_FilteringCoefficient(0.002f,10.f);//计算IIR滤波器参数
	
	PID_Init();							//初始化PID参数
	
	while(1)
	{
		
		if( loop.cnt_1ms >= 1 )
		{
			loop.cnt_1ms = 0;
			duty_1ms();										//执行数据通信任务
		}
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			duty_2ms();										//执行MPU6050数据采集任务、姿态解算和PID内环控制任务
		}

		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;	
			duty_5ms();										//执行接收机数据处理任务、PID外环控制任务
		}

		if( loop.cnt_15ms >= 15 )
		{
			loop.cnt_15ms = 0;						//执行地磁计数据采集任务
			duty_15ms();
		}		
		
	}
}
