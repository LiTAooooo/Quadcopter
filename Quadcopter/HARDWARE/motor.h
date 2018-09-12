#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "sys.h"

#define Motor_Max_Limit		2000
#define Motor_Min_Limit		1000

void Motor_Init(u16 arr, u16 psc);
void Set_Motor(u16 motor1,u16 motor2,u16 motor3,u16 motor4);
int Limit_Motor(int value);				//电机PWM输出限幅

extern u16 Motor1,Motor2,Motor3,Motor4;
#endif
