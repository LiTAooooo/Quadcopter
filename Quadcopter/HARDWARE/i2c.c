#include "i2c.h"

volatile u8 I2C_FastMode;

void I2c_Soft_delay()
{ 
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	
	if(!I2C_FastMode)
	{
		u8 i = 15;
		while(i--);
	}
}

void I2c_Soft_Init()
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
  RCC_AHB1PeriphClockCmd(RCC_I2C , ENABLE );
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL | I2C_Pin_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIO_I2C, &GPIO_InitStructure);		
}

int I2c_Soft_Start()
{
	SDA_H;
	SCL_H;
	I2c_Soft_delay();
	if(!SDA_read)return 0;	//SDA��Ϊ�͵�ƽ������æ���˳�
	SDA_L;
	I2c_Soft_delay();
	if(SDA_read) return 0;	//SDA��Ϊ�ߵ�ƽ�����߳������˳�
	SDA_L;
	I2c_Soft_delay();
	return 1;	

}

void I2c_Soft_Stop()
{
	SCL_L;
	I2c_Soft_delay();
	SDA_L;
	I2c_Soft_delay();
	SCL_H;
	I2c_Soft_delay();
	SDA_H;
	I2c_Soft_delay();
}

void I2c_Soft_Ask()
{
	SCL_L;
	I2c_Soft_delay();
	SDA_L;
	I2c_Soft_delay();
	SCL_H;
	I2c_Soft_delay();
	SCL_L;
	I2c_Soft_delay();
}

void I2c_Soft_NoAsk()
{
	SCL_L;
	I2c_Soft_delay();
	SDA_H;
	I2c_Soft_delay();
	SCL_H;
	I2c_Soft_delay();
	SCL_L;
	I2c_Soft_delay();
}

int I2c_Soft_WaitAsk(void) 	 //����Ϊ:=1��ACK,=0��ACK
{
  u8 ErrTime = 0;
	SCL_L;
	I2c_Soft_delay();
	SDA_H;			
	I2c_Soft_delay();
	SCL_H;
	I2c_Soft_delay();
	while(SDA_read)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop();
			return 1;
		}
	}
	SCL_L;
	I2c_Soft_delay();
	return 0;
}

void I2c_Soft_SendByte(u8 SendByte) //���ݴӸ�λ����λ//
{
    u8 i=8;
    while(i--)
    {
        SCL_L;
        I2c_Soft_delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        I2c_Soft_delay();
				SCL_H;
				I2c_Soft_delay();
    }
    SCL_L;
}  

//��1���ֽ�,ack=1ʱ,����ACK,ack=0,����NACK
u8 I2c_Soft_ReadByte(u8 ask)  //���ݴӸ�λ����λ//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2c_Soft_delay();
			SCL_H;
      I2c_Soft_delay();	
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;

	if (ask)
		I2c_Soft_Ask();
	else
		I2c_Soft_NoAsk();  
    return ReceiveByte;
} 


// IICдһ���ֽ����ݣ�����MPU6050
u8 IIC_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress<<1);   
	if(I2c_Soft_WaitAsk())
	{
		I2c_Soft_Stop();
		return 1;
	}
	I2c_Soft_SendByte(REG_Address);       
	I2c_Soft_WaitAsk();	
	I2c_Soft_SendByte(REG_data);
	I2c_Soft_WaitAsk();   
	I2c_Soft_Stop(); 
	return 0;
}

// IICдһ���ֽ����ݣ�����HMC5883L
u8 IIC_Write_oneByte(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress);   
	if(I2c_Soft_WaitAsk())
	{
		I2c_Soft_Stop();
		return 1;
	}
	I2c_Soft_SendByte(REG_Address);       
	I2c_Soft_WaitAsk();	
	I2c_Soft_SendByte(REG_data);
	I2c_Soft_WaitAsk();   
	I2c_Soft_Stop(); 
	return 0;
}


// IIC��1�ֽ����ݣ�����MPU6050
u8 IIC_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data)
{      		
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress<<1); 
	if(I2c_Soft_WaitAsk())
	{
		I2c_Soft_Stop();
		return 1;
	}
	I2c_Soft_SendByte(REG_Address);     
	I2c_Soft_WaitAsk();
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress<<1 | 0x01);
	I2c_Soft_WaitAsk();
	*REG_data= I2c_Soft_ReadByte(0);
	I2c_Soft_Stop();
	return 0;
}	

// IIC��1�ֽ����ݣ�����HMC5883L
u8 IIC_Read_oneByte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data)
{      		
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress); 
	if(I2c_Soft_WaitAsk())
	{
		I2c_Soft_Stop();
		return 1;
	}
	I2c_Soft_SendByte(REG_Address);     
	I2c_Soft_WaitAsk();
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress+1);
	I2c_Soft_WaitAsk();
	*REG_data= I2c_Soft_ReadByte(0);
	I2c_Soft_Stop();
	return 0;
}	

// IICдn�ֽ�����
u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress<<1); 
	if(I2c_Soft_WaitAsk())
	{
		I2c_Soft_Stop();
		return 1;
	}
	I2c_Soft_SendByte(REG_Address); 
	I2c_Soft_WaitAsk();
	while(len--) 
	{
		I2c_Soft_SendByte(*buf++); 
		I2c_Soft_WaitAsk();
	}
	I2c_Soft_Stop();
	return 0;
}

// IIC��n�ֽ����ݣ�����MPU6050
u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress<<1); 
	if(I2c_Soft_WaitAsk())
	{
		I2c_Soft_Stop();
		return 1;
	}
	I2c_Soft_SendByte(REG_Address); 
	I2c_Soft_WaitAsk();
	
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress<<1 | 0x01); 
	I2c_Soft_WaitAsk();
	while(len) 
	{
		if(len == 1)
		{
			*buf = I2c_Soft_ReadByte(0);
		}
		else
		{
			*buf = I2c_Soft_ReadByte(1);
		}
		buf++;
		len--;
	}
	I2c_Soft_Stop();
	return 0;
}


// IIC��n�ֽ����ݣ�����HMC5883L
u8 IIC_Read_nBytes(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress); 
	if(I2c_Soft_WaitAsk())
	{
		I2c_Soft_Stop();
		return 1;
	}
	I2c_Soft_SendByte(REG_Address); 
	I2c_Soft_WaitAsk();
	
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress+1); 
	I2c_Soft_WaitAsk();
	while(len) 
	{
		if(len == 1)
		{
			*buf = I2c_Soft_ReadByte(0);
		}
		else
		{
			*buf = I2c_Soft_ReadByte(1);
		}
		buf++;
		len--;
	}
	I2c_Soft_Stop();
	return 0;
}