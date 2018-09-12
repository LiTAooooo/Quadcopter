#ifndef _TASKS_H_
#define _TASKS_H_

#include "sys.h"

typedef struct
{
	s16 cnt_1ms;
	s16 cnt_2ms;
	s16 cnt_5ms;
	s16 cnt_15ms;
}loop_t;

extern loop_t loop;

void duty_1ms(void);		//数据通信任务
void duty_2ms(void);		//MPU6050数据采集任务、姿态解算和PID内环控制任务
void duty_5ms(void);		//PID外环控制任务
void duty_15ms(void);		//地磁计数据采集任务

#endif

