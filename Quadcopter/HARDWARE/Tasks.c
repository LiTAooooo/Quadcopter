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

//执行数据通信任务
void duty_1ms()
{
	Data_Exchange();			//数据上传
}


//执行MPU6050数据采集任务、姿态解算和PID内环控制任务
void duty_2ms()
{
	
	MPU6050_Read(); 								//读取mpu6轴传感器原始数据保存到mpu6050_buffer
	
	MPU6050_Compose();							//6050数据合成
	
	ACC_IIR_Filter(&acc,&filter_acc);		//对acc做IIR滤波
	
	Gyro_Filter(&gyro,&filter_gyro);		//对gyro做窗口滤波
	
	Get_Radian(&filter_gyro,&SI_gyro);	//陀螺仪数据的单位转为弧度/s
	
	IMUupdate(SI_gyro.x,SI_gyro.y,SI_gyro.z,filter_acc.x,filter_acc.y,filter_acc.z,filter_mag.x,filter_mag.y,filter_mag.z);		//姿态解算
	
	Pid_Inner_Loop();								//PID内环控制

}

//执行PID外环控制任务
void duty_5ms()
{
	Rc_PWM_To_Angle();								//将从接收机采集到的信号转换成期望值
	
	Pid_Outer_Loop();									//PID外环控制
}


//执行地磁计数据采集任务
void duty_15ms()
{
	HMC5883L_Offset();	//地磁计数据校准，计算零偏值
	
	HMC58X3_getRaw();		//读取磁力计数据，在连续测量模式下输出为75HZ，原始数据自动保存在mag，滤波后的数据自动保存到filter_mag
}


