#include "mpu6050.h"
#include "delay.h"
#include "usart.h"
#include "data_struct.h"
#include "data_transfer.h"

u8 mpu6050_ok;			//���ڼ�¼һЩ���̵�ִ���Ƿ�ɹ�����IICwriteBit�����ɹ�Ϊ1��ʧ��Ϊ0

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


/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*��  ��:	  �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	dev  Ŀ���豸��ַ
reg	   �Ĵ�����ַ
bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
data  Ϊ0ʱ, Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1
ʧ��Ϊ0
*******************************************************************************/
void IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data) {
    u8 b;
    IIC_Read_nByte(dev, reg, 1, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    mpu6050_ok = !( IIC_Write_1Byte(dev, reg, b) );
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*��  ��:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
reg	   �Ĵ�����ַ
bitStart  Ŀ���ֽڵ���ʼλ
length   λ����
data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1
ʧ��Ϊ0
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



/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setSleepEnabled(uint8_t enabled)
*��  ��:	    ����  MPU6050 �Ƿ����˯��ģʽ
enabled =1   ˯��
enabled =0   ����
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setClockSource(uint8_t source)
*��  ��:	    ����  MPU6050 ��ʱ��Դ
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



/**************************ʵ�ֺ���********************************************
*����ԭ��:
*��  ��:	    ���� ������
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
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG,7, 3, 0x00);   //���Լ�
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*��  ��:	    ����  MPU6050 ���ٶȼƵ��������
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG,7, 3, 0x00);   //���Լ�
}



/**************************ʵ�ֺ���********************************************
*����ԭ��:
*��  ��:	    ���õ�ͨ�˲���ֹƵ��
*******************************************************************************/
void MPU6050_setDLPF(uint8_t mode)
{
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}



/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*��  ��:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(MPU6050_ADDR, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*��  ��:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(MPU6050_ADDR, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}



/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_initialize(void)
*��  ��:	    ��ʼ�� 	MPU6050 �Խ������״̬
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
		
    //�豸��λ
		IIC_Write_1Byte(MPU6050_ADDR,MPU6050_RA_PWR_MGMT_1, 0x80);
		delay_ms(20);
		
	
    MPU6050_setSleepEnabled(0); //���빤��״̬
    delay_ms(20);
		
		
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_ZGYRO); //����ʱ��  0x6b   0x03	����Z��
    delay_ms(20);
		
		
    MPU6050_set_SMPLRT_DIV(1000);  //1000hz				//����Ƶ����Ϊ1000hz
    delay_ms(20);
		
		
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//������������� +-2000��ÿ��
    delay_ms(20);

		
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_8);	//���ٶȼ�������� +-8G
    delay_ms(20);

		
    MPU6050_setDLPF(default_filter);  //42hz
    delay_ms(20);
		
    MPU6050_setI2CMasterModeEnabled(0);	 //����MPU6050 ����AUXI2C		�õ������Ƶ�ʱ����Ҫ��
    delay_ms(20);

		
    MPU6050_setI2CBypassEnabled(1);	 //����������I2C��	MPU6050��AUXI2C	ֱͨ ����������ֱ�ӷ���HMC5883L
    delay_ms(20);
		
}


u8 mpu6050_buffer[14];
//mpu6050��ȡ��������mpu6050���ٶȼơ�������ԭʼ���ݶ�ȡ���������浽mpu6050_buffer��
void MPU6050_Read(void)
{
    I2C_FastMode = 1;
    IIC_Read_nByte(MPU6050_ADDR,MPU6050_RA_ACCEL_XOUT_H,14,mpu6050_buffer);
}




uint8_t	GYRO_Offset = 1;		//�Զ�У׼
uint8_t	ACC_Offset  = 1;
/******************************************************************************
����ԭ�ͣ�	void MPU6050_Offset(void)
��    �ܣ�	MPU6050��ƫУ��
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
			f.msg_id = 1;						//����λ������У׼�ɹ���Ϣ
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
			f.msg_id = 2;						//����λ������У׼�ɹ���Ϣ
			f.msg_data = 1;			
		}
	}
}


/******************************************************************************
����ԭ�ͣ�	void MPU6050_Compose(void)
��    �ܣ�	�ϳ�MPU6050��16λ����
*******************************************************************************/ 
void MPU6050_Compose(void)
{
	MPU6050_Offset();				//���ٶȼơ���������ƫУ׼,������ƫֵ
	
	acc.x  = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - offset_acc.x;	//��ȥ��ƫ
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
		MPU6050_Read(); 								//��ȡmpu6�ᴫ����ԭʼ���ݱ��浽mpu6050_buffer
		MPU6050_Compose();							//6050���ݺϳ�
		delay_ms(2);
	}
}
