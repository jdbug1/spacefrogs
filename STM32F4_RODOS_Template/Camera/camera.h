/*
 * camera.h
 *
 *  Created on: 08.12.2015
 *      Author: JackVEnterprises
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include "rodos.h"
#include "../basics.h"

#include "SCCB.h"
#include "Dcmi.h"
#include "camera_registers.h"

#define WIDTH						160
#define HEIGHT						121

#define CAPTUREMODE					DCMI_CaptureMode_SnapShot
#define CAPTURERATE					DCMI_CaptureRate_All_Frame
//#define CAPTUREMODE				DCMI_CaptureMode_Continuous
//#define FRAMERATE					DCMI_CaptureRate_1of4_Frame
#define DCMI_DR_ADDRESS      		0x50050028
#define IMAGESIZE					(HEIGHT*WIDTH*2)
#define THRESHOLD					165
#define MINPIXELTHRESHOLD			80

class camera : public Thread {
public:
	camera(const char* name);
	virtual ~camera();
	void init();
	void run();

	void initRegisters();

private:
	SCCB sccb;
	Dcmi dcmi;

	u8 DCMI_Buffer[IMAGESIZE];

};

#endif /* CAMERA_H_ */
