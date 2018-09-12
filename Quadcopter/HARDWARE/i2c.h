#ifndef _I2C_H_
#define _I2C_H_
#include "sys.h" 

#define SCL_H         GPIO_I2C->BSRRL = I2C_Pin_SCL
#define SCL_L         GPIO_I2C->BSRRH = I2C_Pin_SCL
#define SDA_H         GPIO_I2C->BSRRL = I2C_Pin_SDA
#define SDA_L         GPIO_I2C->BSRRH = I2C_Pin_SDA
#define SCL_read      GPIO_I2C->IDR  & I2C_Pin_SCL
#define SDA_read      GPIO_I2C->IDR  & I2C_Pin_SDA

/***************I2C GPIO∂®“Â******************/
#define GPIO_I2C	GPIOB
#define I2C_Pin_SCL		GPIO_Pin_8
#define I2C_Pin_SDA		GPIO_Pin_9
#define RCC_I2C		RCC_AHB1Periph_GPIOB
/*********************************************/
extern volatile u8 I2C_FastMode;

void I2c_Soft_Init(void);
void I2c_Soft_SendByte(u8 SendByte);
u8 I2c_Soft_ReadByte(u8);

u8 IIC_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
u8 IIC_Write_oneByte(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
u8 IIC_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);
u8 IIC_Read_oneByte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);
u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
u8 IIC_Read_nBytes(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);


#endif
















