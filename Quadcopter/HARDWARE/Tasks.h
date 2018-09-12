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

void duty_1ms(void);		//����ͨ������
void duty_2ms(void);		//MPU6050���ݲɼ�������̬�����PID�ڻ���������
void duty_5ms(void);		//PID�⻷��������
void duty_15ms(void);		//�شż����ݲɼ�����

#endif

