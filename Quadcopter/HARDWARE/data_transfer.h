#ifndef _DATA_TRANSFER_H_
#define _DATA_TRANSFER_H_

#include "sys.h"

//数据拆分宏定义，在发送大于1字节的数据类型时,比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


typedef struct
{
    u8 msg_id;
    u8 msg_data;
    u8 send_status;
    u8 send_senser;
    u8 send_pid1;
    u8 send_pid2;
    u8 send_pid3;
    u8 send_pid4;
    u8 send_rcdata;
    u8 send_motopwm;
		u8 send_check;
} dt_flag_t;


extern dt_flag_t f;

void Data_Exchange(void);
void Data_Receive_Prepare(u8 data);
void Data_Receive_Anl(u8 *data_buf,u8 num);
void Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z);
void Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit, u16 motor1, u16 motor2, u16 motor3, u16 motor4);
void Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4);
void Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void Send_PID_Out(s16 pitch, s16 roll, s16 yaw);
#endif

