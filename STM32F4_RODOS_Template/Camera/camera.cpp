/*
 * camera.cpp
 *
 *  Created on: 08.12.2015
 *      Author: JackVEnterprises
 */

#include "camera.h"

HAL_GPIO Reset(GPIO_010);
HAL_GPIO Power(GPIO_033);

camera::camera(const char* name) : Thread(name),
		dcmi(IMAGESIZE, (uint32_t) DCMI_Buffer, CAPTURERATE, CAPTUREMODE) {
	Reset.init(true,1,1);
	Power.init(true,1,0);
}

camera::~camera() {
	// TODO Auto-generated destructor stub
}

void camera::init() {
	Reset.init(true,1,1);
	Power.init(true,1,0);
	dcmi.InitGPIO();
	dcmi.InitDCMI();
	sccb.initI2C();
	initRegisters();
	dcmi.EnableDCMI();
}

void camera::run() {
	while(1) {
		suspendCallerUntil(NOW() + 1*SECONDS);
		PRINTF("Initialized\n");
	}

}

void camera::initRegisters() {
	u16 i = 0;
	int ret = 0;
	u8 data = 0x80;
	ret = sccb.camera_write_reg(0x12,&data);
	ret = sccb.camera_write_reg(0x12,0x00);

	while (init_registers[i][0] != 0xFF && init_registers[i][1] != 0xFF) {
//		PRINTF("%d\n",i);
		data = init_registers[i][1];
		ret = sccb.camera_write_reg(init_registers[i][0], &data);
		if (ret < 0) {
			//TODO reset I2C and try again
			PRINTF("I2C Error\n");
		}
		i++;
	}
}
