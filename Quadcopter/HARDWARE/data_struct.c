#include "data_struct.h"

struct _acc  acc;			//原始数据
struct _gyro gyro;
struct _mag  mag;

////////////////////////////////////////////
struct _acc  filter_acc;	//滤波且减去零偏后的数据
struct _gyro filter_gyro;
struct _mag  filter_mag;

////////////////////////////////////////////
struct _acc  offset_acc;	//零偏数据
struct _gyro offset_gyro;
struct _mag offset_mag;


////////////////////////////////////////////
struct _SI_float  SI_acc;	//加速度数据（m/s2）
struct _SI_float  SI_gyro;	//角速度数据（rad）

struct _out_angle out_angle;//姿态解算-角度值
struct _rc_expect Rc_expect;//遥控期望值


////////////////////////////////////////////
struct _pid_control_	Pitch_Angle_PID;			//外环PID
struct _pid_control_	Roll_Angle_PID;
struct _pid_control_	Yaw_Angle_PID;

struct _pid_control_	Pitch_Rate_PID;				//内环PID
struct _pid_control_	Roll_Rate_PID;
struct _pid_control_	Yaw_Rate_PID;

float Pitch,Roll,Yaw;												//用于保存PID内环输出值

