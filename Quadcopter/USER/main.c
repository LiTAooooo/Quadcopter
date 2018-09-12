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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);//����ϵͳ�ж����ȼ�����3
	
	delay_init(84);  				//��ʼ����ʱ����
	
	uart2_init(115200);			//��ʼ�����ڲ�����Ϊ115200����������λ��֮��ͨ��
	
	LED_Init();							//��ʼ��LED
	
	I2c_Soft_Init();				//��ʼ��IIC����
	
	MPU6050_Init(42);   		//���ٶȼơ������ǳ�ʼ��������42hz��ͨ�˲�
	
	HMC5883L_Init();				//�شż�HMC5883L��ʼ�������еشż�У׼
	
	MPU6050_Offset_Init();	//MPU6050���ٶȼơ���������ƫУ׼

	PWM_IN_Init();					//���ջ��ɼ���ʼ��
	
	Motor_Init(2500-1, 84-1);	//�����ʼ��,����Ϊ2.5ms��4·PWM���
	
	TIM5_Init(1000-1,84-1);	//���ڲ���1msһ�εļ�ʱ�жϣ�����ʱ��
	
	Calculate_FilteringCoefficient(0.002f,10.f);//����IIR�˲�������
	
	PID_Init();							//��ʼ��PID����
	
	while(1)
	{
		
		if( loop.cnt_1ms >= 1 )
		{
			loop.cnt_1ms = 0;
			duty_1ms();										//ִ������ͨ������
		}
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			duty_2ms();										//ִ��MPU6050���ݲɼ�������̬�����PID�ڻ���������
		}

		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;	
			duty_5ms();										//ִ�н��ջ����ݴ�������PID�⻷��������
		}

		if( loop.cnt_15ms >= 15 )
		{
			loop.cnt_15ms = 0;						//ִ�еشż����ݲɼ�����
			duty_15ms();
		}		
		
	}
}
