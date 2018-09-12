#ifndef _DATA_STRUCT_H_
#define _DATA_STRUCT_H_
#include "sys.h"

/******************************************************************************
							结构体声明
*******************************************************************************/ 
/* MPU6050--加速度计结构体 */
struct _acc
{
	int16_t x;
	int16_t y;
	int16_t z;
};
extern struct _acc acc;
extern struct _acc filter_acc;
extern struct _acc offset_acc;

/* MPU6050--陀螺仪结构体 */
struct _gyro
{
	int16_t x;
	int16_t y;
	int16_t z;
};
extern struct _gyro gyro;
extern struct _gyro filter_gyro;
extern struct _gyro offset_gyro;

/* HMC5883L--地磁计结构体 */
struct _mag
{
	int16_t x;
	int16_t y;
	int16_t z;
};

extern struct _mag mag;
extern struct _mag filter_mag;
extern struct _mag offset_mag;

/* float结构体 */
struct _SI_float
{
	float x;
	float y;
	float z;
};
extern struct _SI_float SI_acc;	
extern struct _SI_float SI_gyro;

/* 姿态解算--角度值 */
struct _out_angle
{
	float yaw;
	float roll;
	float pitch;
};
extern struct _out_angle out_angle;

/*遥控期望值*/
struct _rc_expect
{
	float yaw;
	float roll;
	float pitch;
	u16 thr;
};
extern struct _rc_expect Rc_expect;

/*串级PID结构体*/
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
	double Integ_Max;				//积分上限
	double Integ_Min;				//积分下限
};

extern struct _pid_control_	Pitch_Angle_PID;
extern struct _pid_control_	Roll_Angle_PID;
extern struct _pid_control_	Yaw_Angle_PID;

extern struct _pid_control_	Pitch_Rate_PID;
extern struct _pid_control_	Roll_Rate_PID;
extern struct _pid_control_	Yaw_Rate_PID;

extern float Pitch,Roll,Yaw;
#endif
