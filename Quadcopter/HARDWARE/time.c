#include "time.h"
#include "Tasks.h"

//ʹ�ö�ʱ��5���������ж�
//����ֵ arr
//Ԥ��Ƶ psc
//��Ҫ���ڲ�����ʱ�жϣ�1msһ�Σ���Ϊ��������while(1)ѭ������ĵ���ʱ��

void TIM5_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_Period=arr;
	TIM_TimeBaseInitStruct.TIM_Prescaler=psc;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=TIM5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=2;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);		//ʹ�ܶ�ʱ��5�ĸ����ж�
	
	TIM_Cmd(TIM5,ENABLE);							//ʹ�ܶ�ʱ��5
}



u16 Count_1ms=0,Count_2ms=0,Count_4ms=0,Count_5ms=0,Count_14ms=0;
void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update))		//ȷ���Ƿ�������������ж�
	{
		loop.cnt_1ms++;
		loop.cnt_2ms++;
		loop.cnt_5ms++;
		loop.cnt_15ms++;
	}
	
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);	//����жϱ�־λ
}



u16 IMU_Update_Start_cnt;
u16 IMU_Update_End_cnt;
float Get_IMU_Update_Time(void)
{
	float time;
	IMU_Update_End_cnt = TIM_GetCounter(TIM1);				//��ȡ��ǰ��ʱ������ֵ
	if(IMU_Update_End_cnt >= IMU_Update_Start_cnt)
	{
		time = (IMU_Update_End_cnt - IMU_Update_Start_cnt) / 1000000.0f;		//��λΪs
	}
	else
	{
		time = (0xffff - IMU_Update_Start_cnt + IMU_Update_End_cnt) /1000000.0f;
	}
	IMU_Update_Start_cnt = IMU_Update_End_cnt;				//�Ӵ�ʱ��ʼ���¼���
	
	return time;
}
