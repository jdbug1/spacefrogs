#include "Camera.h"
#include "stdio.h"

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
	PRINTF("starting InitOV7670 init\n");
	uint16_t x = 0;
	int res = 0;
	res = sccb.ov7670_set(0x12, 0x80);
	res = sccb.ov7670_set(0x12, 0x00);

	while (init_registers[x][0] != 0xFF && init_registers[x][1] != 0xFF) {
		res = sccb.ov7670_set((unsigned char) init_registers[x][0],
				(unsigned char) init_registers[x][1]);
		uint8_t read = sccb.ov7670_get((unsigned char) init_registers[x][0]);
		if (res) {
			PRINTF("ERROR I2C %d\n", res);
		}
		x++;

	}
	PRINTF("done with InitOV7670 init\n");
}

void Camera::init() {

	PRINTF("starting cam init\n");
	ledo.init(true);
	reset.init(true);
	power.init(true);
	reset.setPins(1);
	power.setPins(0);
	PRINTF("Init GPIOs...");
	dcmi.InitGPIO();
	PRINTF("Done!\n");
	PRINTF("Init DCMI...");
	dcmi.InitDCMI();
	PRINTF("Done!\n");
	PRINTF("Init I2C...");
	delayx(1000);
	sccb.I2CInit();
	PRINTF("Done!\n");
	PRINTF("Init OV7670...");
	delayx(1000);
	InitOV7670();
	PRINTF("Done!\n");
	PRINTF("Enable DCMI...");
	delayx(1000);
	dcmi.EnableDCMI();

	PRINTF("Done with cam init!\n");
}

void Camera::sendPicture(bool value) {
	sendPic = value;
}

uint8_t* Camera::getPicture() {
	return DCMI_Buffer;
}

void Camera::Capture() {
	DCMI_CaptureCmd(ENABLE);
}

bool Camera::ProcessData() {
	return processData;
}

void Camera::run() {
	int counter = 0;
	while (1) {
		if (sendPic) {
			for (int i = 1; i < IMAGESIZE + 1; i++) {
				PRINTF("%03u", DCMI_Buffer[i - 1]);
			}
			PRINTF("FRAME STOP&");
			sendPic = false;
			active = false;
			processData = true;
		}
		suspendCallerUntil(NOW()+100*MILLISECONDS);
	}
}

void Camera::put(tcStruct &command) {
	if (command.id == 4) {
		this->handleTelecommand(&command);

	}
}

void Camera::handleTelecommand(tcStruct * tc) {
	int command = tc->command;
		switch(command) {
		case 4001:
			turnOn();
			break;
		}
}

void Camera::delayx(unsigned int ms) {
	while (ms > 1) {
		ms--;
		asm("nop");
	}
}


void Camera::turnOn(void){
	this->active = true;
	Capture();
}
void Camera::turnOff(void){
	this->active = false;
}


/***********************************************************************/
