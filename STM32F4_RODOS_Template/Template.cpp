/*
 * Template.cpp
 *
 * Created on: 25.06.2014
 * Author: Atheel Redah
 *
 */

#include "rodos.h"
#include <stdio.h>
#include "hal.h"
#include "math.h"
#include "imu.h"
#include "ahrs.h"
#include "motor.h"
//#include "lightsensor.h"
//#include "Camera/camera.h"
//#include "Electrical.h"

static Application module01("Template", 2001);

Topic<imuData> imu_topic(1,"imu");
Topic<RPY> ahrs_topic(2,"ahrs");
Topic<RPY> xm_topic(3,"xm");
Topic<RPY> gyro_topic(4,"gyro");

/* Uncomment if you want to use something */

IMU imu("imu");
AHRS ahrs("ahrs");
MOTOR motor("motor");
//lightsensor light("light");
//Electrical knife("knife");
//camera cam("cam");

namespace RODOS {
extern HAL_UART uart_stdout;
}


#define BT_UART UART_IDX2
#define USB_UART UART_IDX3
#define IMU_I2C I2C_IDX2

HAL_GPIO GreenLED(LED_GREEN);

class Telemetry: public Thread {

public:

	Telemetry(const char* name) : Thread(name) {
	}

	void init() {
		GreenLED.init(true, 1, 0);
	}

	void run() {
		imuData imu;
		RPY ahrs;
		RPY xm;
		RPY gyro;
		while (1) {
			imuBuffer.get(imu);
			ahrsBuffer.get(ahrs);
			gyroBuffer.get(gyro);
			xmBuffer.get(xm);
			if (!imu.calibrating) {
				PRINTF("Acc %5.2f %5.2f %5.2f\n",imu.ax,imu.ay,imu.az);
				PRINTF("Mag %5.2f %5.2f %5.2f\n",imu.mx,imu.my,imu.mz);
				PRINTF("Gyr %5.2f %5.2f %5.2f\n",imu.wx,imu.wy,imu.wz);
//				PRINTF("XM  %5.2f %5.2f %5.2f\n",radToDeg(xm.roll),radToDeg(xm.pitch),radToDeg(xm.heading));
//				PRINTF("Gyr %5.2f %5.2f %5.2f\n",radToDeg(gyro.roll),radToDeg(gyro.pitch),radToDeg(gyro.heading));
				PRINTF("Com %5.2f %5.2f %5.2f\n",radToDeg(ahrs.roll),radToDeg(ahrs.pitch),radToDeg(ahrs.heading));
			}
			GreenLED.setPins(~GreenLED.readPins());
			//PRINTF("Hello Rodos, the time now is %f \r\n",SECONDS_NOW());

            suspendCallerUntil(NOW()+200*MILLISECONDS);
		}
	}
};
//Telemetry Telemetry("Telemetry");

/***********************************************************************/

class Telecommand : public Thread {

	Telecommand(const char* name) : Thread(name) {
		STATE=WAITING;
	}

	void init() {

	}

	void run() {
		int id;
		char msg[3];
		char value;
		char current_char;
		int bytes = 0;
		int channel;
		bool got_id = false;
		while(1) {
			uart_stdout.suspendUntilDataReady();
			uart_stdout.read(&current_char,1);
			switch(STATE) {
			case WAITING:
				//if (current_char == "$") STATE=GET_ID;
				break;
			case GET_ID:
				id = atoi(&current_char);
				break;
			case GET_MSG:
				if (bytes < 3) {
					msg[bytes] = current_char;
					bytes++;
				}
				if (bytes == 3) {
					checkIdentifier(msg);
					bytes = 0;
				}
				break;
			case GET_INT:
				if (id == 3 && !got_id) {
					channel = atoi(&current_char);
					got_id = true;
					break;
				}

			}

		}
	}

	void checkIdentifier(char* identifier) {
		if (compare(identifier,(char*)"CMA")) {
			STATE=GET_BOOL;

		}
		if (compare(identifier, (char*)"CAC")) {
			STATE=GET_BOOL;

		}
		if (compare(identifier, (char*)"CGY")) {
			STATE=GET_BOOL;

		}
		if (compare(identifier, (char*)"SSP")) {
			STATE=GET_INT;

		}
		if (compare(identifier, (char*)"STE")) {
			STATE=GET_INT;

		}

	}

private:
	enum topic_ids {
		IMU = 1,
		AHRS,
		PWM,
	} TOPIC_IDS;

	enum states {
		WAITING,
		GET_ID,
		GET_MSG,
		GET_INT,
		GET_BOOL,
		GET_STOP,
	} STATE;


};
