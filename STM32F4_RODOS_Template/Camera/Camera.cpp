/*
 * Camera.cpp
 *
 * Created on: 25.06.2014
 * Author: Andreas Schartel
 *
 */

#include "Camera.h"
#include "stdio.h"
//#include "../TM.h"

//Topic<RawVector2D> cameraTargetTopic(-1, "camera target");
//Topic<bool> cameraFireTopic(-1, "camera fire");

//extern TM tm;


Camera::Camera(const char* name, HAL_UART uart) : Thread(name), SubscriberReceiver<tcStruct>(tm_topic_incoming, "SubRec Electrical for Telecommands") {
	ledo = new HAL_GPIO(GPIO_061);
	reset = new HAL_GPIO(GPIO_010);
	power = new HAL_GPIO(GPIO_033);
	tmUart = new HAL_UART(uart);
	dcmi = new Dcmi(IMAGESIZE, (uint32_t) DCMI_Buffer, FRAMERATE, CAPTUREMODE);
	sendPic = false;
	active = false;
	processData = false;
}

void Camera::InitOV7670() {
	xprintf("starting InitOV7670 init\n");
	uint16_t x = 0;
	int res = 0;
	res = sccb.ov7670_set(0x12, 0x80);
	res = sccb.ov7670_set(0x12, 0x00);

	while (init_registers[x][0] != 0xFF && init_registers[x][1] != 0xFF) {
		//xprintf("init register: status x=%d\n", x);

		res = sccb.ov7670_set((unsigned char) init_registers[x][0],
				(unsigned char) init_registers[x][1]);
		uint8_t read = sccb.ov7670_get((unsigned char) init_registers[x][0]);
		//xprintf("SCCB Init %d: reg 0x%x = 0x%x = 0x%x \n", x,
		//		init_registers[x][0], init_registers[x][1], read);
		if (res) {
			xprintf("ERROR I2C %d\n", res);
		}
		x++;

	}
	xprintf("done with InitOV7670 init\n");
}

void Camera::put(tcStruct &command) {
	if (command.id == 4) {
		this->handleTelecommand(&command);
	}
}

void Camera::handleTelecommand(tcStruct * tc) {
	int command = tc->command;
	PRINTF("Command was %d\n",command);
	switch (command) {
	case 4001:
		this->turnOn();
		break;
	}
}

void Camera::Capture() {
	DCMI_CaptureCmd(ENABLE);
}

void Camera::init() {
	xprintf("starting cam init\n");
	ledo->init(true);
	reset->init(true);
	power->init(true);
	reset->setPins(1);
	power->setPins(0);
	xprintf("Init GPIOs...");
	dcmi->InitGPIO();
	xprintf("Done!\n");
	xprintf("Init DCMI...");
	dcmi->InitDCMI();
	xprintf("Done!\n");
	xprintf("Init I2C...");
	suspendCallerUntil(NOW()+2*MILLISECONDS);
	sccb.I2CInit();
	xprintf("Done!\n");
	xprintf("Init OV7670...");
	suspendCallerUntil(NOW()+2*MILLISECONDS);
	InitOV7670();
	xprintf("Done!\n");
	xprintf("Enable DCMI...");
	suspendCallerUntil(NOW()+2*MILLISECONDS);
	dcmi->EnableDCMI();

	xprintf("Done with cam init!\n");
}

void Camera::run() {
	int counter = 0;
	while (1) {
		if (sendPic) {
			for (int i = 0; i < IMAGESIZE; i++) {
//				counter++;
				PRINTF("%03u", DCMI_Buffer[i]);
//				if (counter == WIDTH) {
//					PRINTF("\n");
//					counter = 0;
//				}
			}
			xprintf("FRAME STOP\n");
			sendPic = false;
			active = false;
		}
		suspendCallerUntil(NOW()+100*MILLISECONDS);
	}
}

void Camera::turnOn(void){
	this->active = true;
	//ledo.setPins(1);
//	xprintf("Cam active\n");
	Capture();
}
void Camera::turnOff(void){
	this->active = false;
	xprintf("Cam inactive\n");
	//ledo.setPins(0);
}

void Camera::sendPicture(bool value) {
	sendPic = value;
}
/***********************************************************************/
