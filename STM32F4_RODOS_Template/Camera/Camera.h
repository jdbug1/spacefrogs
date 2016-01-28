/*
 * Camera.h
 *
 *  Created on: 12.01.2015
 *      Author: Andreas Schartel
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include "rodos.h"
#include "hal.h"
#include "../basics.h"

#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"

#include "Peripheral/Dcmi.h"
#include "Peripheral/Sccb.h"

#include "Peripheral/Misc/initRegister.h"

#define BT_UART UART_IDX2
#define USB_UART UART_IDX3

#define WIDTH						160
#define HEIGHT						121

#define CAPTUREMODE					DCMI_CaptureMode_SnapShot
#define FRAMERATE					DCMI_CaptureRate_All_Frame
//#define CAPTUREMODE				DCMI_CaptureMode_Continuous
//#define FRAMERATE					DCMI_CaptureRate_1of4_Frame
#define DCMI_DR_ADDRESS      		0x50050028
#define IMAGESIZE					(HEIGHT*WIDTH*2)
#define THRESHOLD					165
#define MINPIXELTHRESHOLD			80


class Camera: public Thread, public SubscriberReceiver<tcStruct> {
private:
	Dcmi* dcmi;
	Sccb sccb;
	HAL_GPIO* ledo;
	HAL_GPIO* reset;
	HAL_GPIO* power;
	HAL_UART* tmUart;

	uint8_t DCMI_Buffer[IMAGESIZE];



	bool active;
	bool processData;
	bool sendPic;

	void InitOV7670();
	void Capture();
	void put(tcStruct &command);
	void handleTelecommand(tcStruct *tc);

public:
	Camera(const char* name, HAL_UART uart);
	void init();
	void run();
	void turnOn(void);
	void turnOff(void);
	void sendPicture(bool value);
};

#endif /* CAMERA_H_ */
