#include "pwm_in.h"
#include "data_struct.h"

void PWM_IN_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM1_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  		//使能TIM1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能GPIOA时钟	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11; //初始化PA8、PA9、PA10、PA11引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 					//下拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1); 		//将PA8、PA9、PA10、PA11复用为TIM1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_TIM1); 
	  
	TIM_TimeBaseStructure.TIM_Prescaler= 84 - 1 ;  						//定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=0xFFFF;   								//自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		//边沿采样时钟分频
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	

	//初始化TIM1输入捕获参数
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1;									//通道1
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;			//最初设置为上升沿捕获
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //映射到TI1上
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 					//配置输入分频,不分频 
  TIM1_ICInitStructure.TIM_ICFilter = 0x00;													//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_2; 								//通道2
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;			//最初设置为上升沿捕获
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //映射到TI1上
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 					//配置输入分频,不分频 
  TIM1_ICInitStructure.TIM_ICFilter = 0x00;													//IC2F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_3; 								//通道3
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;			//上升沿捕获
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //映射到TI1上
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;					  //配置输入分频,不分频 
  TIM1_ICInitStructure.TIM_ICFilter = 0x00;													//IC3F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_4; 								//通道4
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;			//上升沿捕获
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //映射到TI1上
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 					//配置输入分频,不分频 
  TIM1_ICInitStructure.TIM_ICFilter = 0x00;													//IC4F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
		
	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);//允许更新中断 ,允许CC1IE、CC2IE、CC3IE、CC4IE捕获中断
	
  TIM_Cmd(TIM1,ENABLE ); 																						//使能定时器5

	//TIM1更新中断和捕获中断优先级设置
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn | TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;						//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;									//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;										//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);																		//根据指定的参数初始化VIC寄存器
		
}



u8 PWMin_CH1_STA = 0, PWMin_CH2_STA = 0, PWMin_CH3_STA = 0, PWMin_CH4_STA = 0;					//输入捕获状态标志, 0：当前捕获到上升沿 1：当前捕获到下降沿

u16 RC_PWM_IN[4];							//采集结果存放在该数组内

u16 CH1_Rising,CH1_Falling, CH2_Rising,CH2_Falling, CH3_Rising,CH3_Falling, CH4_Rising,CH4_Falling;		//用于保存边沿捕获（上升/下降沿）时,定时器的瞬时计数值


void TIM1_CC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)						//捕获1发生捕获事件
	{	
		if(!(PWMin_CH1_STA))																			//捕获到上升沿
		{
			CH1_Rising = TIM_GetCapture1(TIM1);												//读取此时的定时器计数值
			PWMin_CH1_STA = 1;																				//改变捕获标志
			TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Falling);				//改为下降沿捕获
		}
		
		else																											//捕获到下降沿
		{
			CH1_Falling = TIM_GetCapture1(TIM1);											//读取此时的定时器计数值
			if(CH1_Falling >= CH1_Rising)															//判断下降沿捕获计数值与上升沿捕获计数值的大小关系
			{
				RC_PWM_IN[0] = CH1_Falling - CH1_Rising;									//如果前者>后者,说明在上升沿到下降沿捕获期间TIM1的计数值没有溢出
			}
			
			else
			{
				RC_PWM_IN[0] = CH1_Falling + 65536 - CH1_Rising;					//如果前者<=后者,则TIM1曾溢出,此时需要做相应处理
			}
			
			PWMin_CH1_STA = 0;																				//改变捕获标志			
			TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Rising);				//重新改为上升沿捕获,以便进行下一次捕获
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); 									//清除捕获1中断标志位
	}
	
	
	else if(TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)				//捕获2发生捕获事件,处理方式与捕获1类似
	{	
		if(!(PWMin_CH2_STA))
		{
			CH2_Rising = TIM_GetCapture2(TIM1);
			PWMin_CH2_STA = 1;
			TIM_OC2PolarityConfig(TIM1,TIM_ICPolarity_Falling);
		}
		else
		{
			CH2_Falling = TIM_GetCapture2(TIM1);
			if(CH2_Falling >= CH2_Rising)
			{
				RC_PWM_IN[1] = CH2_Falling - CH2_Rising;
			}
			else
			{
				RC_PWM_IN[1] = CH2_Falling + 65536 - CH2_Rising;
			}
			PWMin_CH2_STA = 0;
			TIM_OC2PolarityConfig(TIM1,TIM_ICPolarity_Rising);
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
	}	
	
	
	
	else if(TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)				//捕获3发生捕获事件,处理方式与捕获1类似
	{	
		if(!(PWMin_CH3_STA))
		{
			CH3_Rising = TIM_GetCapture3(TIM1);
			PWMin_CH3_STA = 1;
			TIM_OC3PolarityConfig(TIM1,TIM_ICPolarity_Falling);
		}
		else
		{
			CH3_Falling = TIM_GetCapture3(TIM1);
			if(CH3_Falling >= CH3_Rising)
			{
				RC_PWM_IN[2] = CH3_Falling - CH3_Rising;
			}
			else
			{
				RC_PWM_IN[2] = CH3_Falling + 65536 - CH3_Rising;
			}
			PWMin_CH3_STA = 0;
			TIM_OC3PolarityConfig(TIM1,TIM_ICPolarity_Rising);
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
	}
	
	
	
	else if(TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)				//捕获4发生捕获事件,处理方式与捕获1类似
	{	
		if(!(PWMin_CH4_STA))
		{
			CH4_Rising = TIM_GetCapture4(TIM1);
			PWMin_CH4_STA = 1;
			TIM_OC4PolarityConfig(TIM1,TIM_ICPolarity_Falling);
		}
		else
		{
			CH4_Falling = TIM_GetCapture4(TIM1);
			if(CH4_Falling >= CH4_Rising)
			{
				RC_PWM_IN[3] = CH4_Falling - CH4_Rising;
			}
			else
			{
				RC_PWM_IN[3] = CH4_Falling + 65536 - CH4_Rising;
			}
			PWMin_CH4_STA = 0;
			TIM_OC4PolarityConfig(TIM1,TIM_ICPolarity_Rising);
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC4); 
	}
}


void Rc_PWM_To_Angle(void)																				//将从接收机采集到的信号转换成期望值
{
	if(RC_PWM_IN[0] < 1000)RC_PWM_IN[0] = 1000;					//roll
	if(RC_PWM_IN[0] > 2000)RC_PWM_IN[0] = 2000;
	
	if(RC_PWM_IN[1] < 1000)RC_PWM_IN[1] = 1000;					//pitch
	if(RC_PWM_IN[1] > 2000)RC_PWM_IN[1] = 2000;

	if(RC_PWM_IN[2] < 1000)RC_PWM_IN[2] = 1000;					//thr
	if(RC_PWM_IN[2] > 2000)RC_PWM_IN[2] = 2000;
	
	if(RC_PWM_IN[3] < 1000)RC_PWM_IN[3] = 1000;					//yaw
	if(RC_PWM_IN[3] > 2000)RC_PWM_IN[3] = 2000;
	
	Rc_expect.roll = (float)(RC_PWM_IN[0] - 1500) * 0.05f;					//期望roll的范围设置为-25° ~ +25°
	Rc_expect.pitch = (float)(RC_PWM_IN[1] - 1500) * 0.05f;					//期望pitch的范围设置为-25° ~ +25°
	Rc_expect.thr = RC_PWM_IN[2];																		//期望thr的范围不做改变,为1000 ~ 2000
	Rc_expect.yaw = (float)(RC_PWM_IN[3] - 1500) * 0.05f;						//期望yaw的范围设置为-25° ~ +25°
}

