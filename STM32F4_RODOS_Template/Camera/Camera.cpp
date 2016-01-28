
#include "Camera.h"
#include "stdio.h"
//#include "../TM.h"

//Topic<RawVector2D> cameraTargetTopic(-1, "camera target");
//Topic<bool> cameraFireTopic(-1, "camera fire");

//extern TM tm;

Camera::Camera(const char* name, HAL_UART uart) :
		SubscriberReceiver<tcStruct>(tm_topic_incoming, "SubRec Camera for Telecommands"), Thread(name),
		dcmi(IMAGESIZE, (uint32_t) DCMI_Buffer, FRAMERATE, CAPTUREMODE),
		ledo(GPIO_061), //PD13
		reset(GPIO_010), //PA10
		power(GPIO_033), //PC01
		tmUart(uart),
		sendPic(false){
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

void Camera::init() {

	xprintf("starting cam init\n");
	ledo.init(true);
	reset.init(true);
	power.init(true);
	reset.setPins(1);
	power.setPins(0);
	xprintf("Init GPIOs...");
	dcmi.InitGPIO();
	xprintf("Done!\n");
	xprintf("Init DCMI...");
	dcmi.InitDCMI();
	xprintf("Done!\n");
	xprintf("Init I2C...");
	delayx(1000);
	sccb.I2CInit();
	xprintf("Done!\n");
	xprintf("Init OV7670...");
	delayx(1000);
	InitOV7670();
	xprintf("Done!\n");
	xprintf("Enable DCMI...");
	delayx(1000);
	dcmi.EnableDCMI();

	xprintf("Done with cam init!\n");
}

void Camera::sendPicture(bool value) {
	sendPic = value;
	//tm.turnOn();
}

uint8_t* Camera::getPicture() {
	return DCMI_Buffer;
}

void Camera::Capture() {
	DCMI_CaptureCmd(ENABLE);
}


void Camera::DetectSatellite() {

}

void Camera::ProcessData() {
	processData = true;
}

void Camera::run() {
	int counter = 0;
	while (1) {
		if (sendPic) {
			for (int i = 1; i < IMAGESIZE + 1; i += 2) {
				counter++;
				PRINTF("%03u ", DCMI_Buffer[i - 1]);
				if (counter == WIDTH) {
					PRINTF("\n");
					counter = 0;
				}
			}
			sendPic = false;
			active = false;
		}
		suspendCallerUntil(NOW()+100*MILLISECONDS);
	}
/*		PRINTF("Camera taking picture in 1 second!\n");
		suspendCallerUntil(NOW()+1*SECONDS);
		PRINTF("Taking picture!\n");
		turnOn();
		int counter = 0;
		PRINTF("Waiting 5 Seconds!\n");
		suspendCallerUntil(NOW()+5*SECONDS);
		for (int i = 1; i < IMAGESIZE+1; i+=2) {
			counter++;
			PRINTF("%03u ",DCMI_Buffer[i-1]);
			if (counter == WIDTH) {
				PRINTF("\n");
				counter = 0;
			}
		}
		suspendCallerUntil(NOW()+5*SECONDS);
		/*if (processData) {
			processData = false; // Wait till the next frame (interrup) fires processing
			DetectSatellite(); // Perform detection algorithm
			if (sendPic) { // If picture was requested, send
				char tmpVal[4];
				tmUart.write("CAMERA", 6);
				for (int i = 0; i < IMAGESIZE; i += 2) {
					sprintf(tmpVal, "%03u", DCMI_Buffer[i]);
					tmUart.write(tmpVal, 4);
					while (!tmUart.isWriteFinished()) {
					}
				}
				tmUart.write("CAMEND", 6);
				sendPic = false;
			}
			if (active) { // Continue captureing/processing if cam is still active
				Capture();
			}
			suspendCallerUntil(NOW()+200*MILLISECONDS); // Could run even faster but 200ms is suficient for mission mode
		}*/
	//}

}

void Camera::delayx(unsigned int ms) {
	//4694 = 1 ms
	while (ms > 1) {
		ms--;
		asm("nop");
	}
}


void Camera::turnOn(void){
	this->active = true;
	//ledo.setPins(1);
	xprintf("Cam active\n");
	Capture();
}
void Camera::turnOff(void){
	this->active = false;
	xprintf("Cam inactive\n");
	//ledo.setPins(0);
}
/***********************************************************************/
