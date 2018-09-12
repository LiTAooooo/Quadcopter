#ifndef _MYMATH_H_
#define _MYMATH_H_

#include "sys.h"
#include "data_struct.h"

#define Pi	3.1415927f
#define Radian_to_Angle	   57.2957795f
#define RawData_to_Angle	0.0610351f	//���²�����Ӧ2000��ÿ��
#define RawData_to_Radian	0.0010653f

void Get_Radian(struct _gyro *Gyro_in,struct _SI_float *Gyro_out);	
uint8_t Get_Checksum(uint8_t mydata[]);
float invSqrt(float x);

#endif

