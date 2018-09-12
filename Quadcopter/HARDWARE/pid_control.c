#include "pid_control.h"
#include "motor.h"
#include "data_struct.h"
#include "mymath.h"

/*PID������ʼ��*/
void PID_Init(void)
{
//�⻷
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

//�ڻ�
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
	PID->Error = Expect - Measure;			//�������
	PID->Integ += PID->Error;						//�����ۼ�
	
	if(PID->Integ > PID->Integ_Max){PID->Integ = PID->Integ_Max;}				//�����޷�
	else if(PID->Integ < PID->Integ_Min){PID->Integ = PID->Integ_Min;}
	
	PID->Deriv = PID->Error - PID->PreError;			//΢����
	PID->PreError = PID->Error;					//�������
	
	if(Rc_expect.thr <= 1200)						//�����Ž�Сʱ,��������
	{
		PID->Integ = 0;
	}
	PID->Output = PID->Kp * PID->Error + PID->Ki * PID->Integ + PID->Kd * PID->Deriv;
}

void Pid_Inner_Loop(void)								//PID�ڻ�����
{
	//PID_Position_Cal(&Pitch_Rate_PID, (Rc_expect.pitch * 10), (SI_gyro.y * Radian_to_Angle));								//pitch  -250~250
	PID_Position_Cal(&Pitch_Rate_PID, Pitch_Angle_PID.Output, (SI_gyro.y * Radian_to_Angle));								//pitch
	
	//PID_Position_Cal(&Roll_Rate_PID, (Rc_expect.roll * 10), -(SI_gyro.x * Radian_to_Angle));
	PID_Position_Cal(&Roll_Rate_PID, Roll_Angle_PID.Output, -(SI_gyro.x * Radian_to_Angle));									//roll
	PID_Position_Cal(&Yaw_Rate_PID, -(Rc_expect.yaw * 10), -(SI_gyro.z * Radian_to_Angle));										//yaw
	
	Pitch = Pitch_Rate_PID.Output;
	Roll = Roll_Rate_PID.Output;
	Yaw = Yaw_Rate_PID.Output;
	
	if(Rc_expect.thr < 1050)							//��ֹδ������ʱ������������б����ת���µ��ת��
	{
		Motor1 = 1000;
		Motor2 = 1000;
		Motor3 = 1000;
		Motor4 = 1000;
	}
	else	
	{
		Motor1 = (u16)Limit_Motor(Rc_expect.thr + Pitch + Roll - Yaw);			//��ǰ�����M1��˳ʱ����ת
		Motor2 = (u16)Limit_Motor(Rc_expect.thr + Pitch - Roll + Yaw);			//��ǰ�����M2����ʱ����ת
		Motor3 = (u16)Limit_Motor(Rc_expect.thr - Pitch + Roll + Yaw);			//��󷽵��M3����ʱ����ת
		Motor4 = (u16)Limit_Motor(Rc_expect.thr - Pitch - Roll - Yaw);			//�Һ󷽵��M4��˳ʱ����ת
	}
	
	Set_Motor(Motor1, Motor2, Motor3, Motor4);														//��Motorֵ��������
}



void Pid_Outer_Loop(void)								//PID�⻷����
{
	PID_Position_Cal(&Pitch_Angle_PID, -(Rc_expect.pitch), out_angle.pitch);								//pitch
	PID_Position_Cal(&Roll_Angle_PID, Rc_expect.roll, out_angle.roll);											//roll
}

