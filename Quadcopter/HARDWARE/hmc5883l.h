#ifndef _HMC5883L_H_
#define _HMC5883L_H_

#include "sys.h"

#define WINDOW_LEN 3			//�������ڵĳ���

#define HMC58X3_ADDR      0x3C // 7 bit address of the HMC58X3 used with the Wire library

#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_R_XM 3
#define HMC58X3_R_XL 4

#define HMC58X3_R_YM (7)  //!< Register address for YM.
#define HMC58X3_R_YL (8)  //!< Register address for YL.
#define HMC58X3_R_ZM (5)  //!< Register address for ZM.
#define HMC58X3_R_ZL (6)  //!< Register address for ZL.

#define HMC58X3_R_STATUS 9
#define HMC58X3_R_IDA 10
#define HMC58X3_R_IDB 11
#define HMC58X3_R_IDC 12

void HMC5883L_Init(void);	//��ʼ��
void HMC58X3_getID(u8 id[3]);	//��оƬID
void HMC58X3_getRaw(void);	//��ȡ���������ݣ�ԭʼ�����Զ�������mag���˲���������Զ����浽filter_mag
void HMC5883L_Self_Test(void);	//�Բ�
void HMC5883L_Offset(void);	//�شż�У׼����
#endif
