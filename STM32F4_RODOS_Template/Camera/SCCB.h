/*
 * SCCB.h
 *
 *  Created on: 09.12.2015
 *      Author: JackVEnterprises
 */

#ifndef CAMERA_SCCB_H_
#define CAMERA_SCCB_H_

#include "rodos.h"
#include "hal.h"
#include "../basics.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"

#define ONE_BYTE_REG_ADDR 0x01
#define TWO_BYTE_REG_ADDR 0x02

class SCCB {
public:
	SCCB();
	virtual ~SCCB();

	void initClock();
	void initI2C();
	int i2c_send_data(u8 slave_addr, u16 reg_addr, u8* data, u8 addr_len);
	int i2c_receive_data(u8 slave_addr, u16 reg_addr, u8* data, u8 addr_len);
	int camera_read_reg(u8 reg, u8* data);
	int camera_write_reg(u8 reg, u8* data);
};

#endif /* CAMERA_SCCB_H_ */
