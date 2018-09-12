#include "mpu6050.h"
#include "delay.h"
#include "usart.h"
#include "data_struct.h"
#include "data_transfer.h"

u8 mpu6050_ok;			//用于记录一些过程的执行是否成功（如IICwriteBit），成功为1，失败为0

void MPU6050_INT_Config()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 ;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
    GPIO_SetBits(GPIOD, GPIO_Pin_7);
}


/**************************实现函数********************************************
*函数原型:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功  能:	  读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
reg	   寄存器地址
bitNum  要修改目标字节的bitNum位
data  为0时, 目标位将被清0 否则将被置位
返回   成功 为1
失败为0
*******************************************************************************/
void IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data) {
    u8 b;
    IIC_Read_nByte(dev, reg, 1, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    mpu6050_ok = !( IIC_Write_1Byte(dev, reg, b) );
}


/**************************实现函数********************************************
*函数原型:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功  能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
reg	   寄存器地址
bitStart  目标字节的起始位
length   位长度
data    存放改变目标字节位的值
返回   成功 为1
失败为0
*******************************************************************************/
void IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b,mask;
    IIC_Read_nByte(dev, reg, 1, &b);
    mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
    data <<= (8 - length);
    data >>= (7 - bitStart);
    b &= mask;
    b |= data;
    IIC_Write_1Byte(dev, reg, b);
}



/**************************实现函数********************************************
*函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
*功  能:	    设置  MPU6050 是否进入睡眠模式
enabled =1   睡觉
enabled =0   工作
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}


/**************************实现函数********************************************
*函数原型:		void MPU6050_setClockSource(uint8_t source)
*功  能:	    设置  MPU6050 的时钟源
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source)
{
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}



/**************************实现函数********************************************
*函数原型:
*功  能:	    设置 采样率
*******************************************************************************/
void MPU6050_set_SMPLRT_DIV(uint16_t hz)
{
    IIC_Write_1Byte(MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV,1000/hz - 1);
}



/** Set full-scale gyroscope range.
* @param range New full-scale gyroscope range value
* @see getFullScaleRange()
* @see MPU6050_GYRO_FS_250
* @see MPU6050_RA_GYRO_CONFIG
* @see MPU6050_GCONFIG_FS_SEL_BIT
* @see MPU6050_GCONFIG_FS_SEL_LENGTH
*/
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG,7, 3, 0x00);   //不自检
}


/**************************实现函数********************************************
*函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*功  能:	    设置  MPU6050 加速度计的最大量程
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG,7, 3, 0x00);   //不自检
}



/**************************实现函数********************************************
*函数原型:
*功  能:	    设置低通滤波截止频率
*******************************************************************************/
void MPU6050_setDLPF(uint8_t mode)
{
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}



/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*功  能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(MPU6050_ADDR, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}


/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*功  能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(MPU6050_ADDR, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}



/**************************实现函数********************************************
*函数原型:		void MPU6050_initialize(void)
*功  能:	    初始化 	MPU6050 以进入可用状态
*******************************************************************************/
void MPU6050_Init(u16 lpf)
{
    u16 default_filter = 1;

    switch(lpf)
    {
    case 5:
        default_filter = MPU6050_DLPF_BW_5;
        break;
    case 10:
        default_filter = MPU6050_DLPF_BW_10;
        break;
    case 20:
        default_filter = MPU6050_DLPF_BW_20;
        break;
    case 42:
        default_filter = MPU6050_DLPF_BW_42;
        break;
    case 98:
        default_filter = MPU6050_DLPF_BW_98;
        break;
    case 188:
        default_filter = MPU6050_DLPF_BW_188;
        break;
    case 256:
        default_filter = MPU6050_DLPF_BW_256;
        break;
    default:
        default_filter = MPU6050_DLPF_BW_42;
        break;
    }
		
    //设备复位
		IIC_Write_1Byte(MPU6050_ADDR,MPU6050_RA_PWR_MGMT_1, 0x80);
		delay_ms(20);
		
	
    MPU6050_setSleepEnabled(0); //进入工作状态
    delay_ms(20);
		
		
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_ZGYRO); //设置时钟  0x6b   0x03	设置Z轴
    delay_ms(20);
		
		
    MPU6050_set_SMPLRT_DIV(1000);  //1000hz				//采样频率设为1000hz
    delay_ms(20);
		
		
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//陀螺仪最大量程 +-2000度每秒
    delay_ms(20);

		
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_8);	//加速度计最大量程 +-8G
    delay_ms(20);

		
    MPU6050_setDLPF(default_filter);  //42hz
    delay_ms(20);
		
    MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C		用到磁力计的时候需要加
    delay_ms(20);

		
    MPU6050_setI2CBypassEnabled(1);	 //主控制器的I2C与	MPU6050的AUXI2C	直通 控制器可以直接访问HMC5883L
    delay_ms(20);
		
}


u8 mpu6050_buffer[14];
//mpu6050读取函数：将mpu6050加速度计、陀螺仪原始数据读取出来，保存到mpu6050_buffer中
void MPU6050_Read(void)
{
    I2C_FastMode = 1;
    IIC_Read_nByte(MPU6050_ADDR,MPU6050_RA_ACCEL_XOUT_H,14,mpu6050_buffer);
}




uint8_t	GYRO_Offset = 1;		//自动校准
uint8_t	ACC_Offset  = 1;
/******************************************************************************
函数原型：	void MPU6050_Offset(void)
功    能：	MPU6050零偏校正
*******************************************************************************/ 
void MPU6050_Offset(void)
{
	if(ACC_Offset)
	{
		static int32_t ACC_X=0,ACC_Y=0,ACC_Z=0;
		static uint8_t count_acc=0;
		if(count_acc==0)
		{
			offset_acc.x = 0;
			offset_acc.y = 0;
			offset_acc.z = 0;
			ACC_X = 0;
			ACC_Y = 0;
			ACC_Z = 0;
			count_acc = 1;
			return;
		}
		else
		{
			count_acc++;
			ACC_X += acc.x;
			ACC_Y += acc.y;
			ACC_Z += acc.z;
		}
		if(count_acc==251)
		{
			count_acc--;
			offset_acc.x = ACC_X / count_acc;
			offset_acc.y = ACC_Y / count_acc;
			offset_acc.z = ACC_Z / count_acc - 4096;
			count_acc = 0;
			ACC_Offset = 0;
			f.msg_id = 1;						//向上位机发送校准成功信息
			f.msg_data = 1;
		}
	}
	
	if(GYRO_Offset)
	{
		static int32_t GYRO_X=0,GYRO_Y=0,GYRO_Z=0;
		static uint8_t count_gyro=0;
		if(count_gyro==0)
		{
			offset_gyro.x = 0;
			offset_gyro.y  = 0;
			offset_gyro.z   = 0;
			GYRO_X = 0;
			GYRO_Y = 0;
			GYRO_Z = 0;
			count_gyro = 1;
			return;
		}
		else
		{
			count_gyro++;
			GYRO_X += gyro.x;
			GYRO_Y += gyro.y;
			GYRO_Z += gyro.z;
		}
		if(count_gyro==251)
		{
			count_gyro--;
			offset_gyro.x = GYRO_X / count_gyro;
			offset_gyro.y = GYRO_Y / count_gyro;
			offset_gyro.z = GYRO_Z / count_gyro;
			count_gyro = 0;
			GYRO_Offset = 0;
			f.msg_id = 2;						//向上位机发送校准成功信息
			f.msg_data = 1;			
		}
	}
}


/******************************************************************************
函数原型：	void MPU6050_Compose(void)
功    能：	合成MPU6050的16位数据
*******************************************************************************/ 
void MPU6050_Compose(void)
{
	MPU6050_Offset();				//加速度计、陀螺仪零偏校准,计算零偏值
	
	acc.x  = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - offset_acc.x;	//减去零偏
	acc.y  = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - offset_acc.y;
	acc.z  = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) - offset_acc.z;

	gyro.x = ((((int16_t)mpu6050_buffer[8])  << 8) | mpu6050_buffer[9])  - offset_gyro.x;
	gyro.y = ((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) - offset_gyro.y;
	gyro.z = ((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) - offset_gyro.z;
	
}



void MPU6050_Offset_Init(void)
{
	while(GYRO_Offset || ACC_Offset)
	{
		MPU6050_Read(); 								//读取mpu6轴传感器原始数据保存到mpu6050_buffer
		MPU6050_Compose();							//6050数据合成
		delay_ms(2);
	}
}
