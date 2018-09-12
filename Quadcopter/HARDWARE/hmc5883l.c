#include "hmc5883l.h"
#include "usart.h"
#include "i2c.h"
#include "delay.h"
#include "data_struct.h"
#include "data_transfer.h"

int16_t  HMC5883_FIFO[3][WINDOW_LEN]; //磁力计滤波

/**************************实现函数********************************************
*函数原型:	   void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
*功  能:	   	 更新一组数据到FIFO数组
输入参数:  		 磁力计三个轴对应的ADC值
输出参数:  		 无
*******************************************************************************/
void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
{
	u8 i ;
	int32_t sum=0;

//滑动窗口滤波,窗口长度为 WINDOW_LEN
	for(i=1;i<WINDOW_LEN;i++){								
		HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];				//数据滑动移位
		HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
		HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
	}

	HMC5883_FIFO[0][WINDOW_LEN-1]=x;									//最新的数据放在最后
	HMC5883_FIFO[1][WINDOW_LEN-1]=y;
	HMC5883_FIFO[2][WINDOW_LEN-1]=z;
//-----------------------------------------------------------------------------------
	sum=0;
	for(i=0;i<WINDOW_LEN;i++){	//取数组内的值进行求和再取平均
   		sum+=HMC5883_FIFO[0][i];
	}
	
	filter_mag.x = sum/WINDOW_LEN;	//得到滤波后的数据
	filter_mag.x -= offset_mag.x;		//滤波后减去零偏数据

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



/**************************实现函数********************************************
*函数原型:	   void HMC58X3_writeReg(u8 reg, u8 val)
*功  能:	     写HMC5883L的寄存器
输入参数:      reg  寄存器地址
							 val   要写入的值	
输出参数:      无
*******************************************************************************/
void HMC58X3_writeReg(u8 reg, u8 val) {
  IIC_Write_oneByte(HMC58X3_ADDR,reg,val);
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_setDOR(u8 DOR)
*功  能:	    设置 5883L的 数据输出速率
输入参数:     速率值
0 -> 0.75Hz  |   1 -> 1.5Hz
2 -> 3Hz     |   3 -> 7.5Hz
4 -> 15Hz    |   5 -> 30Hz
6 -> 75Hz  
输出参数:    无
*******************************************************************************/
void HMC58X3_setDOR(u8 DOR) {
  if (DOR>6) return;
  HMC58X3_writeReg(HMC58X3_R_CONFA,(DOR<<2)|0x60);	//在改变输出速率的时候不改变采样数（为8）
}

/**************************实现函数********************************************
*函数原型:	   void HMC58X3_setMode(u8 mode)
*功  能:	     设置 5883L的工作模式
输入参数:      模式
输出参数:      无
*******************************************************************************/
void HMC58X3_setMode(u8 mode) {
  if (mode > 2) {										//闲置模式(mode=2或3)
    return;
  }
  HMC58X3_writeReg(HMC58X3_R_MODE, mode);
  delay_us(100);
}


/**************************实现函数********************************************
*函数原型:	   void HMC58X3_init(u8 setmode)
*功  能:	   	 设置 5883L的工作模式
输入参数:      模式
输出参数:      无
*******************************************************************************/
void HMC58X3_init(u8 setmode) {

  if (setmode) {
    HMC58X3_setMode(0);
  }
  HMC58X3_writeReg(HMC58X3_R_CONFA, 0x70);
	
  HMC58X3_writeReg(HMC58X3_R_CONFB, 0xA0);
}




/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getID(u8 *id)
*功  能:	  读取芯片ID
输入参数:   ID存放的数组
输出参数:  	无
*******************************************************************************/
void HMC58X3_getID(u8 *id) 
{
  IIC_Read_oneByte(HMC58X3_ADDR,HMC58X3_R_IDA,&id[0]);  
  IIC_Read_oneByte(HMC58X3_ADDR,HMC58X3_R_IDB,&id[1]);
  IIC_Read_oneByte(HMC58X3_ADDR,HMC58X3_R_IDC,&id[2]);
}// getID().


/**************************实现函数********************************************
*函数原型:	  void HMC5883_Check()
*功  能:	  检测HMC5883是否已连接
输入参数:    
输出函数:    
*******************************************************************************/
void HMC5883_Check() 
{
  u8 ID_temp[3];
  HMC58X3_getID(ID_temp); //对ID临时数组赋值
  if((ID_temp[0]==0x48)&&(ID_temp[1]==0x34)&&(ID_temp[2]==0x33))//HMC的固定ID号为三个字节，16进制表示分别为0x48，0x34，0x33
  printf("HMC5883L check success...\r\n");
  else printf("HMC5883L not found...\r\n");
}


/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getRaw(void)
*功  能:	    写HMC5883L的寄存器
输入参数:    	reg  寄存器地址
							val   要写入的值	
输出参数:  		无
*******************************************************************************/
void HMC58X3_getRaw(void) {
   u8 vbuff[6]={0};
	 
   IIC_Read_nBytes(HMC58X3_ADDR,HMC58X3_R_XM,6,vbuff);
	 mag.x = ((int16_t)vbuff[0] << 8) | vbuff[1];
	 mag.y = ((int16_t)vbuff[4] << 8) | vbuff[5];
	 mag.z = ((int16_t)vbuff[2] << 8) | vbuff[3];
	 
   HMC58X3_newValues(mag.x, mag.y, mag.z);

}



/**************************实现函数********************************************
*函数原型:	   void HMC58X3_FIFO_init(void)
*功  能:	     连续读取50次数据,以初始化FIFO数组
输入参数:  		 无
输出参数:  		 无
*******************************************************************************/
void HMC58X3_FIFO_init(void)
{
  u8 i;
  for(i=0;i<50;i++){
  HMC58X3_getRaw();
  delay_us(200);
  }
}

/**************************实现函数********************************************
*函数原型:	  void HMC5883L_Init(void)
*功 	 能：		初始化 HMC5883L 使之进入可用状态
输入参数:     	
输出参数:     无
*******************************************************************************/
void HMC5883L_Init(void)
{
	HMC58X3_init(0); // 为0时，不设置mode寄存器，默认为单一测量模式，为1时，设置为连续测量模式
	HMC58X3_setMode(0);	//设置mode寄存器，改为连续测量模式
	HMC58X3_setDOR(6);  //75hz 更新率	
	
	HMC5883L_Offset();		//地磁计校准
	
	HMC58X3_FIFO_init();
}



uint8_t MAG_Offset  = 1;		//地磁计上电自动进入校准
short MAG_Xmax = 0, MAG_Xmin = 0;
short MAG_Ymax = 0, MAG_Ymin = 0;
short MAG_Zmax = 0, MAG_Zmin = 0;
u8 Offset_buff[6]={0};
/**************************实现函数********************************************
*函数原型:	  void HMC5883L_Offset(void)
*功 	 能：		对HMC5883L进行数据校准
输入参数:     	
输出参数:     无
*******************************************************************************/
void HMC5883L_Offset(void)
{
	if(MAG_Offset)			//判断是否需要校准
	{	
		TIM_Cmd(TIM5,DISABLE);		//关闭定时器5,在地磁计校准期间不进入1ms定时中断，此时只有usart2接收中断可以接收上位机的命令
		
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
		
			delay_ms(15);							//每15ms读一次传感器数据，因为地磁计的输出频率为75hz
		}
	
		offset_mag.x = (MAG_Xmax + MAG_Xmin) / 2;			//得到校准值
		offset_mag.y = (MAG_Ymax + MAG_Ymin) / 2;
		offset_mag.z = (MAG_Zmax + MAG_Zmin) / 2;
		
		f.msg_id = 3;						//向上位机发送校准成功信息
		f.msg_data = 1;
		
		TIM_Cmd(TIM5,ENABLE);				//重新开启定时器5
	}
}
