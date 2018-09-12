#include "motor.h"
#include "delay.h"

u16 Motor1,Motor2,Motor3,Motor4;

void Motor_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM3);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=arr;
	TIM_TimeBaseInitStruct.TIM_Prescaler=psc;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse=1000;
	
	TIM_OC1Init(TIM3,&TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM3,&TIM_OCInitStruct);
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	TIM_OC3Init(TIM3,&TIM_OCInitStruct);
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM3,&TIM_OCInitStruct);
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3,ENABLE);
	
	TIM_Cmd(TIM3,ENABLE);

	Motor1 = 1000, Motor2 = 1000, Motor3 = 1000, Motor4 = 1000;
	Set_Motor(Motor1, Motor2, Motor3, Motor4);
	
}

void Set_Motor(u16 motor1,u16 motor2,u16 motor3,u16 motor4)
{
	TIM_SetCompare1(TIM3,motor1);
	TIM_SetCompare2(TIM3,motor2);
	TIM_SetCompare3(TIM3,motor3);
	TIM_SetCompare4(TIM3,motor4);
}

int Limit_Motor(int value)
{
	if(value > Motor_Max_Limit)
		value=Motor_Max_Limit;
	else if(value < Motor_Min_Limit)
		value=Motor_Min_Limit;
	return value;
}
