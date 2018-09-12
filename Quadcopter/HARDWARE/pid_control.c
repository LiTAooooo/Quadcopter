#include "pid_control.h"
#include "motor.h"
#include "data_struct.h"
#include "mymath.h"

/*PID参数初始化*/
void PID_Init(void)
{
//外环
	Pitch_Angle_PID.Kp = 1.2;
	Pitch_Angle_PID.Ki = 0;
	Pitch_Angle_PID.Kd = 0;
	Pitch_Angle_PID.Integ_Max = 1000;
	Pitch_Angle_PID.Integ_Min = -1000;
	
	Roll_Angle_PID.Kp = 1.2;
	Roll_Angle_PID.Ki = 0;
	Roll_Angle_PID.Kd = 0;
	Roll_Angle_PID.Integ_Max = 1000;
	Roll_Angle_PID.Integ_Min = -1000;
	
	Yaw_Angle_PID.Kp = 0;
	Yaw_Angle_PID.Ki = 0;
	Yaw_Angle_PID.Kd = 0;
	Yaw_Angle_PID.Integ_Max = 1000;
	Yaw_Angle_PID.Integ_Min = -1000;

//内环
	Pitch_Rate_PID.Kp = 1.8;
	Pitch_Rate_PID.Ki = 0.012;
	Pitch_Rate_PID.Kd = 1.0;
	Pitch_Rate_PID.Integ_Max = 5000;
	Pitch_Rate_PID.Integ_Min = -5000;
	
	Roll_Rate_PID.Kp = 1.8;
	Roll_Rate_PID.Ki = 0.012;
	Roll_Rate_PID.Kd = 1.0;
	Roll_Rate_PID.Integ_Max = 5000;
	Roll_Rate_PID.Integ_Min = -5000;
	
	Yaw_Rate_PID.Kp = 1.8;
	Yaw_Rate_PID.Ki = 0.025;
	Yaw_Rate_PID.Kd = 1.0;
	Yaw_Rate_PID.Integ_Max = 5000;
	Yaw_Rate_PID.Integ_Min = -5000;
}


void PID_Position_Cal(struct _pid_control_ * PID, float Expect, float Measure)
{
	PID->Error = Expect - Measure;			//计算误差
	PID->Integ += PID->Error;						//积分累加
	
	if(PID->Integ > PID->Integ_Max){PID->Integ = PID->Integ_Max;}				//积分限幅
	else if(PID->Integ < PID->Integ_Min){PID->Integ = PID->Integ_Min;}
	
	PID->Deriv = PID->Error - PID->PreError;			//微分项
	PID->PreError = PID->Error;					//更新误差
	
	if(Rc_expect.thr <= 1200)						//当油门较小时,积分清零
	{
		PID->Integ = 0;
	}
	PID->Output = PID->Kp * PID->Error + PID->Ki * PID->Integ + PID->Kd * PID->Deriv;
}

void Pid_Inner_Loop(void)								//PID内环控制
{
	//PID_Position_Cal(&Pitch_Rate_PID, (Rc_expect.pitch * 10), (SI_gyro.y * Radian_to_Angle));								//pitch  -250~250
	PID_Position_Cal(&Pitch_Rate_PID, Pitch_Angle_PID.Output, (SI_gyro.y * Radian_to_Angle));								//pitch
	
	//PID_Position_Cal(&Roll_Rate_PID, (Rc_expect.roll * 10), -(SI_gyro.x * Radian_to_Angle));
	PID_Position_Cal(&Roll_Rate_PID, Roll_Angle_PID.Output, -(SI_gyro.x * Radian_to_Angle));									//roll
	PID_Position_Cal(&Yaw_Rate_PID, -(Rc_expect.yaw * 10), -(SI_gyro.z * Radian_to_Angle));										//yaw
	
	Pitch = Pitch_Rate_PID.Output;
	Roll = Roll_Rate_PID.Output;
	Yaw = Yaw_Rate_PID.Output;
	
	if(Rc_expect.thr < 1050)							//防止未加油门时，由于四轴倾斜或旋转导致电机转动
	{
		Motor1 = 1000;
		Motor2 = 1000;
		Motor3 = 1000;
		Motor4 = 1000;
	}
	else	
	{
		Motor1 = (u16)Limit_Motor(Rc_expect.thr + Pitch + Roll - Yaw);			//左前方电机M1，顺时针旋转
		Motor2 = (u16)Limit_Motor(Rc_expect.thr + Pitch - Roll + Yaw);			//右前方电机M2，逆时针旋转
		Motor3 = (u16)Limit_Motor(Rc_expect.thr - Pitch + Roll + Yaw);			//左后方电机M3，逆时针旋转
		Motor4 = (u16)Limit_Motor(Rc_expect.thr - Pitch - Roll - Yaw);			//右后方电机M4，顺时针旋转
	}
	
	Set_Motor(Motor1, Motor2, Motor3, Motor4);														//将Motor值输出到电机
}



void Pid_Outer_Loop(void)								//PID外环控制
{
	PID_Position_Cal(&Pitch_Angle_PID, -(Rc_expect.pitch), out_angle.pitch);								//pitch
	PID_Position_Cal(&Roll_Angle_PID, Rc_expect.roll, out_angle.roll);											//roll
}

