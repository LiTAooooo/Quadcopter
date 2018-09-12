#ifndef _DATA_STRUCT_H_
#define _DATA_STRUCT_H_
#include "sys.h"

/******************************************************************************
							�ṹ������
*******************************************************************************/ 
/* MPU6050--���ٶȼƽṹ�� */
struct _acc
{
	int16_t x;
	int16_t y;
	int16_t z;
};
extern struct _acc acc;
extern struct _acc filter_acc;
extern struct _acc offset_acc;

/* MPU6050--�����ǽṹ�� */
struct _gyro
{
	int16_t x;
	int16_t y;
	int16_t z;
};
extern struct _gyro gyro;
extern struct _gyro filter_gyro;
extern struct _gyro offset_gyro;

/* HMC5883L--�شżƽṹ�� */
struct _mag
{
	int16_t x;
	int16_t y;
	int16_t z;
};

extern struct _mag mag;
extern struct _mag filter_mag;
extern struct _mag offset_mag;

/* float�ṹ�� */
struct _SI_float
{
	float x;
	float y;
	float z;
};
extern struct _SI_float SI_acc;	
extern struct _SI_float SI_gyro;

/* ��̬����--�Ƕ�ֵ */
struct _out_angle
{
	float yaw;
	float roll;
	float pitch;
};
extern struct _out_angle out_angle;

/*ң������ֵ*/
struct _rc_expect
{
	float yaw;
	float roll;
	float pitch;
	u16 thr;
};
extern struct _rc_expect Rc_expect;

/*����PID�ṹ��*/
struct _pid_control_
{
	float Kp;
	float Ki;
	float Kd;
	float Error;
	float PreError;
	double Integ;
	float Deriv;
	float Output;
	double Integ_Max;				//��������
	double Integ_Min;				//��������
};

extern struct _pid_control_	Pitch_Angle_PID;
extern struct _pid_control_	Roll_Angle_PID;
extern struct _pid_control_	Yaw_Angle_PID;

extern struct _pid_control_	Pitch_Rate_PID;
extern struct _pid_control_	Roll_Rate_PID;
extern struct _pid_control_	Yaw_Rate_PID;

extern float Pitch,Roll,Yaw;
#endif
