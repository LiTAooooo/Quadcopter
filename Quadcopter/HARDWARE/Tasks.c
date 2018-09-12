#include "Tasks.h"
#include "mpu6050.h"
#include "Filter.h"
#include "mymath.h"
#include "IMU.h"
#include "hmc5883l.h"
#include "data_transfer.h"
#include "pwm_in.h"
#include "pid_control.h"

loop_t loop;

//ִ������ͨ������
void duty_1ms()
{
	Data_Exchange();			//�����ϴ�
}


//ִ��MPU6050���ݲɼ�������̬�����PID�ڻ���������
void duty_2ms()
{
	
	MPU6050_Read(); 								//��ȡmpu6�ᴫ����ԭʼ���ݱ��浽mpu6050_buffer
	
	MPU6050_Compose();							//6050���ݺϳ�
	
	ACC_IIR_Filter(&acc,&filter_acc);		//��acc��IIR�˲�
	
	Gyro_Filter(&gyro,&filter_gyro);		//��gyro�������˲�
	
	Get_Radian(&filter_gyro,&SI_gyro);	//���������ݵĵ�λתΪ����/s
	
	IMUupdate(SI_gyro.x,SI_gyro.y,SI_gyro.z,filter_acc.x,filter_acc.y,filter_acc.z,filter_mag.x,filter_mag.y,filter_mag.z);		//��̬����
	
	Pid_Inner_Loop();								//PID�ڻ�����

}

//ִ��PID�⻷��������
void duty_5ms()
{
	Rc_PWM_To_Angle();								//���ӽ��ջ��ɼ������ź�ת��������ֵ
	
	Pid_Outer_Loop();									//PID�⻷����
}


//ִ�еشż����ݲɼ�����
void duty_15ms()
{
	HMC5883L_Offset();	//�شż�����У׼��������ƫֵ
	
	HMC58X3_getRaw();		//��ȡ���������ݣ�����������ģʽ�����Ϊ75HZ��ԭʼ�����Զ�������mag���˲���������Զ����浽filter_mag
}


