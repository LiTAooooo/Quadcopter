#ifndef _TIME_H_
#define _TIME_H_
#include "sys.h"

/******************************************************************************
							全局变量声明
*******************************************************************************/ 

void TIM5_Init(u16 arr,u16 psc);	//初始化定时器5
float Get_IMU_Update_Time(void);
#endif
