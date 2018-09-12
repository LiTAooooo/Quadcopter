#include "hmc5883l.h"
#include "usart.h"
#include "i2c.h"
#include "delay.h"
#include "data_struct.h"
#include "data_transfer.h"

int16_t  HMC5883_FIFO[3][WINDOW_LEN]; //�������˲�

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
*��  ��:	   	 ����һ�����ݵ�FIFO����
�������:  		 �������������Ӧ��ADCֵ
�������:  		 ��
*******************************************************************************/
void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
{
	u8 i ;
	int32_t sum=0;

//���������˲�,���ڳ���Ϊ WINDOW_LEN
	for(i=1;i<WINDOW_LEN;i++){								
		HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];				//���ݻ�����λ
		HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
		HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
	}

	HMC5883_FIFO[0][WINDOW_LEN-1]=x;									//���µ����ݷ������
	HMC5883_FIFO[1][WINDOW_LEN-1]=y;
	HMC5883_FIFO[2][WINDOW_LEN-1]=z;
//-----------------------------------------------------------------------------------
	sum=0;
	for(i=0;i<WINDOW_LEN;i++){	//ȡ�����ڵ�ֵ���������ȡƽ��
   		sum+=HMC5883_FIFO[0][i];
	}
	
	filter_mag.x = sum/WINDOW_LEN;	//�õ��˲��������
	filter_mag.x -= offset_mag.x;		//�˲����ȥ��ƫ����

	sum=0;
	for(i=0;i<WINDOW_LEN;i++){
   		sum+=HMC5883_FIFO[1][i];
	}
	
	filter_mag.y = sum/WINDOW_LEN;
	filter_mag.y -= offset_mag.y;

	sum=0;
	for(i=0;i<WINDOW_LEN;i++){
   		sum+=HMC5883_FIFO[2][i];
	}
	
	filter_mag.z = sum/WINDOW_LEN;
	filter_mag.z -= offset_mag.z;
} //HMC58X3_newValues



/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void HMC58X3_writeReg(u8 reg, u8 val)
*��  ��:	     дHMC5883L�ļĴ���
�������:      reg  �Ĵ�����ַ
							 val   Ҫд���ֵ	
�������:      ��
*******************************************************************************/
void HMC58X3_writeReg(u8 reg, u8 val) {
  IIC_Write_oneByte(HMC58X3_ADDR,reg,val);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_setDOR(u8 DOR)
*��  ��:	    ���� 5883L�� �����������
�������:     ����ֵ
0 -> 0.75Hz  |   1 -> 1.5Hz
2 -> 3Hz     |   3 -> 7.5Hz
4 -> 15Hz    |   5 -> 30Hz
6 -> 75Hz  
�������:    ��
*******************************************************************************/
void HMC58X3_setDOR(u8 DOR) {
  if (DOR>6) return;
  HMC58X3_writeReg(HMC58X3_R_CONFA,(DOR<<2)|0x60);	//�ڸı�������ʵ�ʱ�򲻸ı��������Ϊ8��
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void HMC58X3_setMode(u8 mode)
*��  ��:	     ���� 5883L�Ĺ���ģʽ
�������:      ģʽ
�������:      ��
*******************************************************************************/
void HMC58X3_setMode(u8 mode) {
  if (mode > 2) {										//����ģʽ(mode=2��3)
    return;
  }
  HMC58X3_writeReg(HMC58X3_R_MODE, mode);
  delay_us(100);
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void HMC58X3_init(u8 setmode)
*��  ��:	   	 ���� 5883L�Ĺ���ģʽ
�������:      ģʽ
�������:      ��
*******************************************************************************/
void HMC58X3_init(u8 setmode) {

  if (setmode) {
    HMC58X3_setMode(0);
  }
  HMC58X3_writeReg(HMC58X3_R_CONFA, 0x70);
	
  HMC58X3_writeReg(HMC58X3_R_CONFB, 0xA0);
}




/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_getID(u8 *id)
*��  ��:	  ��ȡоƬID
�������:   ID��ŵ�����
�������:  	��
*******************************************************************************/
void HMC58X3_getID(u8 *id) 
{
  IIC_Read_oneByte(HMC58X3_ADDR,HMC58X3_R_IDA,&id[0]);  
  IIC_Read_oneByte(HMC58X3_ADDR,HMC58X3_R_IDB,&id[1]);
  IIC_Read_oneByte(HMC58X3_ADDR,HMC58X3_R_IDC,&id[2]);
}// getID().


/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5883_Check()
*��  ��:	  ���HMC5883�Ƿ�������
�������:    
�������:    
*******************************************************************************/
void HMC5883_Check() 
{
  u8 ID_temp[3];
  HMC58X3_getID(ID_temp); //��ID��ʱ���鸳ֵ
  if((ID_temp[0]==0x48)&&(ID_temp[1]==0x34)&&(ID_temp[2]==0x33))//HMC�Ĺ̶�ID��Ϊ�����ֽڣ�16���Ʊ�ʾ�ֱ�Ϊ0x48��0x34��0x33
  printf("HMC5883L check success...\r\n");
  else printf("HMC5883L not found...\r\n");
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_getRaw(void)
*��  ��:	    дHMC5883L�ļĴ���
�������:    	reg  �Ĵ�����ַ
							val   Ҫд���ֵ	
�������:  		��
*******************************************************************************/
void HMC58X3_getRaw(void) {
   u8 vbuff[6]={0};
	 
   IIC_Read_nBytes(HMC58X3_ADDR,HMC58X3_R_XM,6,vbuff);
	 mag.x = ((int16_t)vbuff[0] << 8) | vbuff[1];
	 mag.y = ((int16_t)vbuff[4] << 8) | vbuff[5];
	 mag.z = ((int16_t)vbuff[2] << 8) | vbuff[3];
	 
   HMC58X3_newValues(mag.x, mag.y, mag.z);

}



/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void HMC58X3_FIFO_init(void)
*��  ��:	     ������ȡ50������,�Գ�ʼ��FIFO����
�������:  		 ��
�������:  		 ��
*******************************************************************************/
void HMC58X3_FIFO_init(void)
{
  u8 i;
  for(i=0;i<50;i++){
  HMC58X3_getRaw();
  delay_us(200);
  }
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5883L_Init(void)
*�� 	 �ܣ�		��ʼ�� HMC5883L ʹ֮�������״̬
�������:     	
�������:     ��
*******************************************************************************/
void HMC5883L_Init(void)
{
	HMC58X3_init(0); // Ϊ0ʱ��������mode�Ĵ�����Ĭ��Ϊ��һ����ģʽ��Ϊ1ʱ������Ϊ��������ģʽ
	HMC58X3_setMode(0);	//����mode�Ĵ�������Ϊ��������ģʽ
	HMC58X3_setDOR(6);  //75hz ������	
	
	HMC5883L_Offset();		//�شż�У׼
	
	HMC58X3_FIFO_init();
}



uint8_t MAG_Offset  = 1;		//�شż��ϵ��Զ�����У׼
short MAG_Xmax = 0, MAG_Xmin = 0;
short MAG_Ymax = 0, MAG_Ymin = 0;
short MAG_Zmax = 0, MAG_Zmin = 0;
u8 Offset_buff[6]={0};
/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5883L_Offset(void)
*�� 	 �ܣ�		��HMC5883L��������У׼
�������:     	
�������:     ��
*******************************************************************************/
void HMC5883L_Offset(void)
{
	if(MAG_Offset)			//�ж��Ƿ���ҪУ׼
	{	
		TIM_Cmd(TIM5,DISABLE);		//�رն�ʱ��5,�ڵشż�У׼�ڼ䲻����1ms��ʱ�жϣ���ʱֻ��usart2�����жϿ��Խ�����λ��������
		
		MAG_Xmax = 0,MAG_Xmin=0;
		MAG_Ymax = 0,MAG_Ymin=0;
		MAG_Zmax = 0,MAG_Zmin=0;
		
		while(MAG_Offset)
		{
			IIC_Read_nBytes(HMC58X3_ADDR,HMC58X3_R_XM,6,Offset_buff);
			mag.x = ((int16_t)Offset_buff[0] << 8) | Offset_buff[1];
			mag.y = ((int16_t)Offset_buff[4] << 8) | Offset_buff[5];
			mag.z = ((int16_t)Offset_buff[2] << 8) | Offset_buff[3];
		
			if(mag.x > MAG_Xmax)
			{
				MAG_Xmax = mag.x;
			}
			else if(mag.x < MAG_Xmin)
			{
				MAG_Xmin = mag.x;
			}
		
			if(mag.y > MAG_Ymax)
			{
				MAG_Ymax = mag.y;
			}
			else if(mag.y < MAG_Ymin)
			{
				MAG_Ymin = mag.y;
			}
		
			if(mag.z > MAG_Zmax)
			{
				MAG_Zmax = mag.z;
			}
			else if(mag.z < MAG_Zmin)
			{
				MAG_Zmin = mag.z;
			}
		
			delay_ms(15);							//ÿ15ms��һ�δ��������ݣ���Ϊ�شżƵ����Ƶ��Ϊ75hz
		}
	
		offset_mag.x = (MAG_Xmax + MAG_Xmin) / 2;			//�õ�У׼ֵ
		offset_mag.y = (MAG_Ymax + MAG_Ymin) / 2;
		offset_mag.z = (MAG_Zmax + MAG_Zmin) / 2;
		
		f.msg_id = 3;						//����λ������У׼�ɹ���Ϣ
		f.msg_data = 1;
		
		TIM_Cmd(TIM5,ENABLE);				//���¿�����ʱ��5
	}
}
