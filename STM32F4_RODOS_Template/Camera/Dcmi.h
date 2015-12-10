/*
 * Dcmi.h
 *
 *  Created on: 09.12.2015
 *      Author: JackVEnterprises
 */

#ifndef CAMERA_DCMI_H_
#define CAMERA_DCMI_H_

#include "rodos.h"
#include "hal.h"
#include "../basics.h"
#include "misc/stm_misc.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dma.h"




class Dcmi {
public:
	Dcmi(uint32_t imageSize, uint32_t dmaMemoryAddress, uint16_t captureRate, uint16_t captureMode);
	virtual ~Dcmi();

	void InitGPIO();
	void InitDCMI();
	void EnableDCMI();
	void DisableDCMI();

private:
	uint32_t imgSize;
	uint32_t dmaMemAddr;
	uint16_t captRate;
	uint16_t captMode;
};

#endif /* CAMERA_DCMI_H_ */
