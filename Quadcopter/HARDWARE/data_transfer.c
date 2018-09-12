#include "data_transfer.h"
#include "usart.h"
#include "data_struct.h"
#include "pwm_in.h"
#include "motor.h"


/********************************************���ݷ��Ͳ���************************************************/

dt_flag_t f;					//��Ҫ�������ݵı�־
u8 data_to_send[50];	//�������ݻ���
u8 checkdata_to_send,checksum_to_send;


//Send_Data������Э�������з������ݹ���ʹ�õ��ķ��ͺ���
void Send_Data(u8 *dataToSend , u8 length)
{
	u8 i;
	for(i=0;i<length;i++)
		usart2_send_char(dataToSend[i]);
}


static void Send_Check(u8 head, u8 check_sum)			//�ɿط�������֡У����Ϣ--����������У��ʱ��Ҫ�õ�
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

static void Send_Msg(u8 id, u8 data)		//����λ������У׼�Ľ�����ɹ�orʧ�ܣ����������ٶȡ������ǡ��شż�
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
//Data_Exchange��������������ݷ�������
void Data_Exchange(void)
{
	
	if((cnt % senser_cnt) == (senser_cnt-1))								//10msִ��һ��	����λ�����ʹ���������
		f.send_senser = 1;

	
	if((cnt % status_cnt) == (status_cnt-1))								//15msִ��һ��	����λ��������̬�ǣ�roll��pitch��yaw��
		f.send_status = 1;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))								//20msִ��һ��	����λ������ң�ؿ�����Ϣ
		f.send_rcdata = 1;	

	
	if( ++cnt == 60)																				//�����������ڵ���С������
		cnt = 0;
//------------------------------------------------------------------------------------------------------------------
	if(f.msg_id)																						//����λ������У׼��� id=1�����ٶȼ�,id=2��������,id=3���شż� data:1,У׼�ɹ�
	{
		Send_Msg(f.msg_id,f.msg_data);
		f.msg_id = 0;
	}
	
	
//��ÿ��Data_Exchange�У�ֻ��һ�������ϴ������ܹ�ִ�У�����ִ��ʱ����ã����ڳ�����������ִ�У�
	
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_check)														//����λ������У����Ϣ
	{
		f.send_check = 0;
		Send_Check(checkdata_to_send,checksum_to_send);
	}	
	
	else if(f.send_rcdata)																						//20ms	����λ������ң�ؿ�����Ϣ,���͵�˳������Ϊ��THR��YAW��ROL��PIT�Ŀ�����
	{
		f.send_rcdata = 0;
		Send_RCData(RC_PWM_IN[2],RC_PWM_IN[3],RC_PWM_IN[0],RC_PWM_IN[1], Motor1, Motor2, Motor3, Motor4);	
		Send_PID_Out(Pitch, Roll, Yaw);														//����λ������PID�ڻ������Ϣ
	}		
	
	else if(f.send_status)																						//15ms	����λ��������̬�ǣ�roll��pitch��yaw��
	{
		f.send_status = 0;
		Send_Status(out_angle.roll,out_angle.pitch,out_angle.yaw,0,0,1);
	}	
	
	
	else if(f.send_senser)																						//10ms	����λ�����;��˲���У׼��Ĵ���������
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


void Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)				//������̬��
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



void Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)							//���ʹ���������
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

void Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit, u16 motor1, u16 motor2, u16 motor3, u16 motor4)																											//����ң�ؿ�������
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



void Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4)									//���͵����������
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



void Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)		//����PID����
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

void Send_PID_Out(s16 pitch, s16 roll, s16 yaw)			//����λ������PID�ڻ����pitch roll yaw
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


/********************************************���ݽ��ղ���************************************************/

//Data_Receive_Prepare������Э��Ԥ����������Э��ĸ�ʽ�����յ������ݽ���һ�θ�ʽ�Խ�������ʽ��ȷ�Ļ��ٽ������ݽ�������usart2�����ж��б����ã�
//�˺������������ϸ�ʽ������֡�󣬻����е������ݽ�������
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



extern uint8_t	GYRO_Offset;		//������У׼��־
extern uint8_t	ACC_Offset;			//���ٶȼ�У׼��־
extern uint8_t  MAG_Offset;			//�شż�У׼��־

//Data_Receive_Anl������Э�����ݽ������������������Ƿ���Э���ʽ��һ������֡���ú��������ȶ�Э�����ݽ���У��
//У��ͨ��������ݽ��н�����ʵ����Ӧ����
//�˺������Բ����û����е��ã��ɺ���Data_Receive_Prepare�Զ�����
void Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);										//����sum
	
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ,������0XAAAF��ͷ
	
	
	
	if(*(data_buf+2)==0X01)							 //������Ϊ0x01,���ڽ���У׼����
	{
		
		if(*(data_buf+4)==0X01)
		{
			ACC_Offset = 1;												//���м��ٶȼ�У׼
		}
		
		else if(*(data_buf+4)==0X02)
		{
			GYRO_Offset = 1;											//��������������У׼
		}
		
		else if(*(data_buf+4)==0X04)
		{
			MAG_Offset = !MAG_Offset;							//���еشż�����У׼,�������һ��������У׼,ǰ���跢������У׼�ź�,��һ��ΪУ׼����,�ڶ���ΪУ׼�ر�
		}
		
	}
	
	
	else if(*(data_buf+2)==0X02)							 //������Ϊ0x02�����ڽ���PID��ȡ����
	{
		
		if(*(data_buf+4)==0X01)									//��ȡPID����
		{
			f.send_pid1 = 1;											
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;													//PID���ͱ�־���ã�������ɺ������ݴ��Ͳ��ֻ��Զ�����

		}
		
	}

/***************��ȡд��PID��Ϣ****************/
	
	else if(*(data_buf+2)==0X10)							//������Ϊ0x10�����ڽ���PID1��PID2��PID3
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
	
	
  else if(*(data_buf+2)==0X11)							//������Ϊ0x11�����ڽ���PID4��PID5��PID6
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
	
	
  else if(*(data_buf+2)==0X12)							//������Ϊ0x12�����ڽ���PID7��PID8��PID9
  {	
		Roll_Rate_PID.Integ_Max = ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		
		Roll_Rate_PID.Integ_Min = -( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		
		Pitch_Rate_PID.Integ_Max = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );

		Pitch_Rate_PID.Integ_Min = -( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );		
		
		Yaw_Rate_PID.Integ_Max = ( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );

		Yaw_Rate_PID.Integ_Min = -( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );

		Send_Check(0X12,sum);
  }
	
	
	else if(*(data_buf+2)==0X13)							//������Ϊ0x13�����ڽ���PID10��PID11��PID12
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
