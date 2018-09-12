#include "data_struct.h"

struct _acc  acc;			//ԭʼ����
struct _gyro gyro;
struct _mag  mag;

////////////////////////////////////////////
struct _acc  filter_acc;	//�˲��Ҽ�ȥ��ƫ�������
struct _gyro filter_gyro;
struct _mag  filter_mag;

////////////////////////////////////////////
struct _acc  offset_acc;	//��ƫ����
struct _gyro offset_gyro;
struct _mag offset_mag;


////////////////////////////////////////////
struct _SI_float  SI_acc;	//���ٶ����ݣ�m/s2��
struct _SI_float  SI_gyro;	//���ٶ����ݣ�rad��

struct _out_angle out_angle;//��̬����-�Ƕ�ֵ
struct _rc_expect Rc_expect;//ң������ֵ


////////////////////////////////////////////
struct _pid_control_	Pitch_Angle_PID;			//�⻷PID
struct _pid_control_	Roll_Angle_PID;
struct _pid_control_	Yaw_Angle_PID;

struct _pid_control_	Pitch_Rate_PID;				//�ڻ�PID
struct _pid_control_	Roll_Rate_PID;
struct _pid_control_	Yaw_Rate_PID;

float Pitch,Roll,Yaw;												//���ڱ���PID�ڻ����ֵ

