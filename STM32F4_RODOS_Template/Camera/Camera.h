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

#define Q1							0.25f
#define HALF						0.5f
#define Q3							0.75f


class Camera: public Thread {
private:
	Dcmi dcmi;
	Sccb sccb;
	HAL_GPIO ledo;
	HAL_GPIO reset;
	HAL_GPIO power;

	uint8_t DCMI_Buffer[IMAGESIZE];

	HAL_UART tmUart;

	bool active;
	bool processData;
	bool sendPic;
	bool activateCamera;

	void InitOV7670();
	void delayx(unsigned int ms);
	void Capture();
	void DetectSatellite();
public:
	Camera(const char* name, HAL_UART uart);
	void init();
	void run();
	uint8_t* getPicture();
	void sendPicture(bool value);
	void ProcessData();
	void turnOn(void);
	void turnOff(void);
	void initialize();
	void initCamera();
};

#endif /* CAMERA_H_ */
