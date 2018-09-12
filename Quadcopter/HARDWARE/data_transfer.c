#include "data_transfer.h"
#include "usart.h"
#include "data_struct.h"
#include "pwm_in.h"
#include "motor.h"


/********************************************数据发送部分************************************************/

dt_flag_t f;					//需要发送数据的标志
u8 data_to_send[50];	//发送数据缓存
u8 checkdata_to_send,checksum_to_send;


//Send_Data函数是协议中所有发送数据功能使用到的发送函数
void Send_Data(u8 *dataToSend , u8 length)
{
	u8 i;
	for(i=0;i<length;i++)
		usart2_send_char(dataToSend[i]);
}


static void Send_Check(u8 head, u8 check_sum)			//飞控返回数据帧校验信息--当开启数据校验时需要用到
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	Send_Data(data_to_send, 7);
}

static void Send_Msg(u8 id, u8 data)		//向上位机返回校准的结果（成功or失败），包括加速度、陀螺仪、地磁计
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEE;
	data_to_send[3]=2;
	data_to_send[4]=id;
	data_to_send[5]=data;
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	Send_Data(data_to_send, 7);
}


u16 cnt = 0;
u8 senser_cnt 	= 10;
u8 status_cnt 	= 15;
u8 rcdata_cnt 	= 20;
u8 motopwm_cnt	= 20;
//Data_Exchange函数处理各种数据发送请求
void Data_Exchange(void)
{
	
	if((cnt % senser_cnt) == (senser_cnt-1))								//10ms执行一次	向上位机发送传感器数据
		f.send_senser = 1;

	
	if((cnt % status_cnt) == (status_cnt-1))								//15ms执行一次	向上位机发送姿态角（roll、pitch、yaw）
		f.send_status = 1;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))								//20ms执行一次	向上位机发送遥控控制信息
		f.send_rcdata = 1;	

	
	if( ++cnt == 60)																				//所有任务周期的最小公倍数
		cnt = 0;
//------------------------------------------------------------------------------------------------------------------
	if(f.msg_id)																						//向上位机返回校准结果 id=1：加速度计,id=2：陀螺仪,id=3：地磁计 data:1,校准成功
	{
		Send_Msg(f.msg_id,f.msg_data);
		f.msg_id = 0;
	}
	
	
//在每个Data_Exchange中，只有一个数据上传任务能够执行，以免执行时间过久（周期长的任务优先执行）
	
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_check)														//向上位机返回校验信息
	{
		f.send_check = 0;
		Send_Check(checkdata_to_send,checksum_to_send);
	}	
	
	else if(f.send_rcdata)																						//20ms	向上位机发送遥控控制信息,发送的顺序依次为：THR、YAW、ROL、PIT的控制量
	{
		f.send_rcdata = 0;
		Send_RCData(RC_PWM_IN[2],RC_PWM_IN[3],RC_PWM_IN[0],RC_PWM_IN[1], Motor1, Motor2, Motor3, Motor4);	
		Send_PID_Out(Pitch, Roll, Yaw);														//向上位机发送PID内环输出信息
	}		
	
	else if(f.send_status)																						//15ms	向上位机发送姿态角（roll、pitch、yaw）
	{
		f.send_status = 0;
		Send_Status(out_angle.roll,out_angle.pitch,out_angle.yaw,0,0,1);
	}	
	
	
	else if(f.send_senser)																						//10ms	向上位机发送经滤波且校准后的传感器数据
	{
		f.send_senser = 0;
		Send_Senser(filter_acc.x,filter_acc.y,filter_acc.z,filter_gyro.x,filter_gyro.y,filter_gyro.z,filter_mag.x,filter_mag.y,filter_mag.z);
	}
	
	else if(f.send_pid1)
	{
		f.send_pid1 = 0;
		Send_PID(1,Roll_Rate_PID.Kp,Roll_Rate_PID.Ki,Roll_Rate_PID.Kd,Pitch_Rate_PID.Kp,Pitch_Rate_PID.Ki,Pitch_Rate_PID.Kd,Yaw_Rate_PID.Kp,Yaw_Rate_PID.Ki,Yaw_Rate_PID.Kd);
	}		

	else if(f.send_pid2)
	{
		f.send_pid2 = 0;
		Send_PID(2,Roll_Angle_PID.Kp,Roll_Angle_PID.Ki,Roll_Angle_PID.Kd,Pitch_Angle_PID.Kp,Pitch_Angle_PID.Ki,Pitch_Angle_PID.Kd,Yaw_Angle_PID.Kp,Yaw_Angle_PID.Ki,Yaw_Angle_PID.Kd);
	}
	
	else if(f.send_pid3)
	{
		f.send_pid3 = 0;
		Send_PID(3,0,Roll_Rate_PID.Integ_Max*0.001f,0, 0,Pitch_Rate_PID.Integ_Max*0.001f,0, 0,Yaw_Rate_PID.Integ_Max*0.001f,0);
	}
	
	else if(f.send_pid4)
	{
		f.send_pid4 = 0;
		Send_PID(4,0,Roll_Angle_PID.Integ_Max*0.001f,0,0,Pitch_Angle_PID.Integ_Max*0.001f,0,0,Yaw_Angle_PID.Integ_Max*0.001f,0);
	}
	
}


void Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)				//发送姿态角
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}



void Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)							//发送传感器数据
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
/////////////////////////////////////////////
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data(data_to_send, _cnt);
}

void Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit, u16 motor1, u16 motor2, u16 motor3, u16 motor4)																											//发送遥控控制数据
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=20;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(motor1);
	data_to_send[_cnt++]=BYTE0(motor1);
	data_to_send[_cnt++]=BYTE1(motor2);
	data_to_send[_cnt++]=BYTE0(motor2);
	data_to_send[_cnt++]=BYTE1(motor3);
	data_to_send[_cnt++]=BYTE0(motor3);
	data_to_send[_cnt++]=BYTE1(motor4);
	data_to_send[_cnt++]=BYTE0(motor4);
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
	
}



void Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4)									//发送电机控制数据
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}



void Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)		//发送PID数据
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	Send_Data(data_to_send, _cnt);
	
}

void Send_PID_Out(s16 pitch, s16 roll, s16 yaw)			//向上位机发送PID内环结果pitch roll yaw
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x0B;
	data_to_send[_cnt++]=6;
	
	_temp = pitch;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = roll;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = yaw;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data(data_to_send, _cnt);
}


/********************************************数据接收部分************************************************/

//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析（在usart2接收中断中被调用）
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
	
}



extern uint8_t	GYRO_Offset;		//陀螺仪校准标志
extern uint8_t	ACC_Offset;			//加速度计校准标志
extern uint8_t  MAG_Offset;			//地磁计校准标志

//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
void Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);										//计算sum
	
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头,必须以0XAAAF开头
	
	
	
	if(*(data_buf+2)==0X01)							 //功能字为0x01,用于接收校准命令
	{
		
		if(*(data_buf+4)==0X01)
		{
			ACC_Offset = 1;												//进行加速度计校准
		}
		
		else if(*(data_buf+4)==0X02)
		{
			GYRO_Offset = 1;											//进行陀螺仪数据校准
		}
		
		else if(*(data_buf+4)==0X04)
		{
			MAG_Offset = !MAG_Offset;							//进行地磁计数据校准,若想完成一次完整的校准,前后需发送两次校准信号,第一次为校准开启,第二次为校准关闭
		}
		
	}
	
	
	else if(*(data_buf+2)==0X02)							 //功能字为0x02，用于接收PID读取命令
	{
		
		if(*(data_buf+4)==0X01)									//读取PID命令
		{
			f.send_pid1 = 1;											
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;													//PID发送标志设置，设置完成后，在数据传送部分会自动发出

		}
		
	}

/***************读取写入PID信息****************/
	
	else if(*(data_buf+2)==0X10)							//功能字为0x10，用于接收PID1、PID2、PID3
  {
		Roll_Rate_PID.Kp = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		Roll_Rate_PID.Ki = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		Roll_Rate_PID.Kd = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		
		Pitch_Rate_PID.Kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		Pitch_Rate_PID.Ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
		Pitch_Rate_PID.Kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );		
		
		Yaw_Rate_PID.Kp = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
		Yaw_Rate_PID.Ki = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
		Yaw_Rate_PID.Kd = 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		
		Send_Check(0X10,sum);
  }
	
	
  else if(*(data_buf+2)==0X11)							//功能字为0x11，用于接收PID4、PID5、PID6
  {
		Roll_Angle_PID.Kp = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		Roll_Angle_PID.Ki = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		Roll_Angle_PID.Kd = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		
		Pitch_Angle_PID.Kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		Pitch_Angle_PID.Ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
		Pitch_Angle_PID.Kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );		
		
		Yaw_Angle_PID.Kp = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
		Yaw_Angle_PID.Ki = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
		Yaw_Angle_PID.Kd = 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		
		Send_Check(0X11,sum);
  }
	
	
  else if(*(data_buf+2)==0X12)							//功能字为0x12，用于接收PID7、PID8、PID9
  {	
		Roll_Rate_PID.Integ_Max = ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		
		Roll_Rate_PID.Integ_Min = -( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		
		Pitch_Rate_PID.Integ_Max = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );

		Pitch_Rate_PID.Integ_Min = -( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );		
		
		Yaw_Rate_PID.Integ_Max = ( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );

		Yaw_Rate_PID.Integ_Min = -( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );

		Send_Check(0X12,sum);
  }
	
	
	else if(*(data_buf+2)==0X13)							//功能字为0x13，用于接收PID10、PID11、PID12
	{

		Roll_Angle_PID.Integ_Max = ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );

		Roll_Angle_PID.Integ_Min = -( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		
		Pitch_Angle_PID.Integ_Max = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );

		Pitch_Angle_PID.Integ_Min = -( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );		
		
		Yaw_Angle_PID.Integ_Max = ( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );

		Yaw_Angle_PID.Integ_Min = -( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
		
		Send_Check(0X13,sum);
	}

}
