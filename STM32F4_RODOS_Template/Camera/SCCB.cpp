/*
 * SCCB.cpp
 *
 *  Created on: 09.12.2015
 *      Author: JackVEnterprises
 */

#include "SCCB.h"

SCCB::SCCB() {
	// TODO Auto-generated constructor stub

}

SCCB::~SCCB() {
	// TODO Auto-generated destructor stub
}

void SCCB::initClock() {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_ClockSecuritySystemCmd(ENABLE);

	/* Enable GPIOs clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);

	/* Configure MCO (PA8) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_4);
}

void SCCB::initI2C() {
	GPIO_InitTypeDef GPIO_InitStructure; // this is for the GPIO pins used as I2C1SDA and I2C1SCL
	I2C_InitTypeDef I2C_InitStructure; // this is for the I2C1 initilization

	/* enable APB1 peripheral clock for I2C1*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	/* enable the peripheral clock for the pins used by
	 PB6 for I2C SCL and PB9 for I2C1_SDL*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* This sequence sets up the I2C1SDA and I2C1SCL pins
	 * so they work correctly with the I2C1 peripheral
	 */
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; // Pins 10(I2C1_SCL) and 11(I2C1_SDA)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // this defines the output type as open drain
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStructure); // now all the values are passed to the GPIO_Init()

	/* The I2C1_SCL and I2C1_SDA pins are now connected to their AF
	 * so that the I2C1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	/* Configure I2C1 */
	I2C_StructInit(&I2C_InitStructure);
	I2C_DeInit(I2C1);

	/* Enable the I2C peripheral */
	I2C_Cmd(I2C1, ENABLE);

	/* Set the I2C structure parameters */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	/* I2C Peripheral Enable */

	I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);
	/* Initialize the I2C peripheral w/ selected parameters */
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
}

int SCCB::i2c_send_data(u8 slave_addr, u16 reg_addr, u8* data, u8 addr_len)
{
  int timeout = 0x7FFFFF;
  int ret = 0;
         /* send i2c*/
  I2C_GenerateSTART(I2C1, ENABLE);
  while( !I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
  {
   if ((timeout--) == 0)
   {
    ret = 1;
    goto exit;
   }
  }
  I2C_Send7bitAddress(I2C1, slave_addr, I2C_Direction_Transmitter);
  while(!(I2C1->SR1 & (1 << 1))) // check ADDR bit
  {
   if ((timeout--) == 0)
   {
    ret = 2;
    goto exit;
   }
  }
  while(!(I2C1->SR2 & (1 << 2)))   // check TRA bit
  {
   if ((timeout--) == 0)
   {
    ret = 3;
    goto exit;
   }
  }

  /* 2 byte reg address */
  if(addr_len == TWO_BYTE_REG_ADDR)
  {
   // MSB
   I2C_SendData(I2C1, (0xFF & (reg_addr >> 8)) );
   while(!(I2C1->SR1 & (1 << 7)))
   {
   if ((timeout--) == 0)
   {
    ret = 4;
    goto exit;
   }
   }

   // LSB
   I2C_SendData(I2C1, (0xFF & reg_addr));
   while(!(I2C1->SR1 & (1 << 7)))
   {
   if ((timeout--) == 0)
   {
    ret = 5;
    goto exit;
   }
   }

  }
  /* 1 byte reg address */
  else
  {
   I2C_SendData(I2C1, (0xFF & reg_addr));
   while(!(I2C1->SR1 & (1 << 7)))
   {
   if ((timeout--) == 0)
   {
    ret = 6;
    goto exit;
   }
   }
  }

  I2C_SendData(I2C1, *data);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
   if ((timeout--) == 0)
   {
    ret = 7;
    goto exit;
   }
  }

                exit:
  I2C_GenerateSTOP(I2C1, ENABLE);
  return ret;
}

int SCCB::i2c_receive_data(u8 slave_addr, u16 reg_addr, u8* data, u8 addr_len)
{
  int timeout = 0x7FFFFF;
  int ret = 0;
  PRINTF("Before first while\n");
   /* send i2c*/
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
  {
   if ((timeout--) == 0)
   {
    ret = 1;
    goto exit;
   }
  }
  PRINTF("Before send7bit\n");

  I2C_Send7bitAddress(I2C1, slave_addr, I2C_Direction_Transmitter);
  while(!(I2C1->SR1 & (1 << 1))) // check ADDR bit
  {
   if ((timeout--) == 0)
   {
    ret = 2;
    goto exit;
   }
  }

  while(!(I2C1->SR2 & (1 << 2)))   // check TRA bit
  {
   if ((timeout--) == 0)
   {
    ret = 3;
    goto exit;
   }
  }

  /* 2 byte reg address */
  if(addr_len == TWO_BYTE_REG_ADDR)
  {
   // MSB
   I2C_SendData(I2C1, (0xFF & (reg_addr >> 8)) );
   while(!(I2C1->SR1 & (1 << 7)))
   {
   if ((timeout--) == 0)
   {
    ret = 4;
    goto exit;
   }
  }

   // LSB
   I2C_SendData(I2C1, (0xFF & reg_addr));
   while(!(I2C1->SR1 & (1 << 7)))
   {
   if ((timeout--) == 0)
   {
    ret = 5;
    goto exit;
   }
  }
  }

  /* 1 byte reg address */
  else
  {
   I2C_SendData(I2C1, (0xFF & reg_addr));
   while(!(I2C1->SR1 & (1 << 7)))
   {
   if ((timeout--) == 0)
   {
    ret = 6;
    goto exit;
   }
  }
  }

  I2C_GenerateSTOP(I2C1, ENABLE);
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
  {
   if ((timeout--) == 0)
   {
    ret = 7;
    goto exit;
   }
  }
  I2C_Send7bitAddress(I2C1, slave_addr, I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) // check ADDR bit
  {
   if ((timeout--) == 0)
   {
    ret = 8;
    goto exit;
   }
  }


  I2C_AcknowledgeConfig(I2C1, DISABLE);
                /* Send STOP Condition */
                I2C_GenerateSTOP(I2C1, ENABLE);

  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
  {
   if ((timeout--) == 0)
   {
    ret = 10;
    goto exit;
   }
  }

  *data = I2C_ReceiveData(I2C1);
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  return ret;

exit:
  I2C_GenerateSTOP(I2C1, ENABLE);
  return ret;
}

int SCCB::camera_read_reg(u8 reg, u8* data)
{
 return i2c_receive_data(0x42, (u16) reg, data, ONE_BYTE_REG_ADDR);
}

int SCCB::camera_write_reg(u8 reg, u8* data)
{
 return i2c_send_data(0x43, (u16) reg, data, ONE_BYTE_REG_ADDR);
}


