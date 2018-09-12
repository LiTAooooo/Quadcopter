#include "pwm_in.h"
#include "data_struct.h"

void PWM_IN_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM1_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  		//ʹ��TIM1ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��GPIOAʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11; //��ʼ��PA8��PA9��PA10��PA11����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 					//����
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1); 		//��PA8��PA9��PA10��PA11����ΪTIM1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_TIM1); 
	  
	TIM_TimeBaseStructure.TIM_Prescaler= 84 - 1 ;  						//��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=0xFFFF;   								//�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		//���ز���ʱ�ӷ�Ƶ
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	

	//��ʼ��TIM1���벶�����
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1;									//ͨ��1
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;			//�������Ϊ�����ز���
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //ӳ�䵽TI1��
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 					//���������Ƶ,����Ƶ 
  TIM1_ICInitStructure.TIM_ICFilter = 0x00;													//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_2; 								//ͨ��2
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;			//�������Ϊ�����ز���
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //ӳ�䵽TI1��
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 					//���������Ƶ,����Ƶ 
  TIM1_ICInitStructure.TIM_ICFilter = 0x00;													//IC2F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_3; 								//ͨ��3
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;			//�����ز���
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //ӳ�䵽TI1��
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;					  //���������Ƶ,����Ƶ 
  TIM1_ICInitStructure.TIM_ICFilter = 0x00;													//IC3F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_4; 								//ͨ��4
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;			//�����ز���
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //ӳ�䵽TI1��
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 					//���������Ƶ,����Ƶ 
  TIM1_ICInitStructure.TIM_ICFilter = 0x00;													//IC4F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
		
	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);//��������ж� ,����CC1IE��CC2IE��CC3IE��CC4IE�����ж�
	
  TIM_Cmd(TIM1,ENABLE ); 																						//ʹ�ܶ�ʱ��5

	//TIM1�����жϺͲ����ж����ȼ�����
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn | TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;						//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;									//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;										//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);																		//����ָ���Ĳ�����ʼ��VIC�Ĵ���
		
}



u8 PWMin_CH1_STA = 0, PWMin_CH2_STA = 0, PWMin_CH3_STA = 0, PWMin_CH4_STA = 0;					//���벶��״̬��־, 0����ǰ���������� 1����ǰ�����½���

u16 RC_PWM_IN[4];							//�ɼ��������ڸ�������

u16 CH1_Rising,CH1_Falling, CH2_Rising,CH2_Falling, CH3_Rising,CH3_Falling, CH4_Rising,CH4_Falling;		//���ڱ�����ز�������/�½��أ�ʱ,��ʱ����˲ʱ����ֵ


void TIM1_CC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)						//����1���������¼�
	{	
		if(!(PWMin_CH1_STA))																			//����������
		{
			CH1_Rising = TIM_GetCapture1(TIM1);												//��ȡ��ʱ�Ķ�ʱ������ֵ
			PWMin_CH1_STA = 1;																				//�ı䲶���־
			TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Falling);				//��Ϊ�½��ز���
		}
		
		else																											//�����½���
		{
			CH1_Falling = TIM_GetCapture1(TIM1);											//��ȡ��ʱ�Ķ�ʱ������ֵ
			if(CH1_Falling >= CH1_Rising)															//�ж��½��ز������ֵ�������ز������ֵ�Ĵ�С��ϵ
			{
				RC_PWM_IN[0] = CH1_Falling - CH1_Rising;									//���ǰ��>����,˵���������ص��½��ز����ڼ�TIM1�ļ���ֵû�����
			}
			
			else
			{
				RC_PWM_IN[0] = CH1_Falling + 65536 - CH1_Rising;					//���ǰ��<=����,��TIM1�����,��ʱ��Ҫ����Ӧ����
			}
			
			PWMin_CH1_STA = 0;																				//�ı䲶���־			
			TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Rising);				//���¸�Ϊ�����ز���,�Ա������һ�β���
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); 									//�������1�жϱ�־λ
	}
	
	
	else if(TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)				//����2���������¼�,����ʽ�벶��1����
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
	
	
	
	else if(TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)				//����3���������¼�,����ʽ�벶��1����
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
	
	
	
	else if(TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)				//����4���������¼�,����ʽ�벶��1����
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


void Rc_PWM_To_Angle(void)																				//���ӽ��ջ��ɼ������ź�ת��������ֵ
{
	if(RC_PWM_IN[0] < 1000)RC_PWM_IN[0] = 1000;					//roll
	if(RC_PWM_IN[0] > 2000)RC_PWM_IN[0] = 2000;
	
	if(RC_PWM_IN[1] < 1000)RC_PWM_IN[1] = 1000;					//pitch
	if(RC_PWM_IN[1] > 2000)RC_PWM_IN[1] = 2000;

	if(RC_PWM_IN[2] < 1000)RC_PWM_IN[2] = 1000;					//thr
	if(RC_PWM_IN[2] > 2000)RC_PWM_IN[2] = 2000;
	
	if(RC_PWM_IN[3] < 1000)RC_PWM_IN[3] = 1000;					//yaw
	if(RC_PWM_IN[3] > 2000)RC_PWM_IN[3] = 2000;
	
	Rc_expect.roll = (float)(RC_PWM_IN[0] - 1500) * 0.05f;					//����roll�ķ�Χ����Ϊ-25�� ~ +25��
	Rc_expect.pitch = (float)(RC_PWM_IN[1] - 1500) * 0.05f;					//����pitch�ķ�Χ����Ϊ-25�� ~ +25��
	Rc_expect.thr = RC_PWM_IN[2];																		//����thr�ķ�Χ�����ı�,Ϊ1000 ~ 2000
	Rc_expect.yaw = (float)(RC_PWM_IN[3] - 1500) * 0.05f;						//����yaw�ķ�Χ����Ϊ-25�� ~ +25��
}

