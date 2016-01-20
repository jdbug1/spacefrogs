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
#include "Electrical.h"
#include "Telemetry.h"
#include "Camera/Camera.h"

namespace RODOS {
extern HAL_UART uart_stdout;
}

#define Teleuart uart_stdout


static Application module01("Template", 2001);


Topic<imuData> imu_topic(5100,"imu");
Topic<imuPublish> ahrs_topic(5200,"ahrs");
Topic<RPY> xm_topic(5300,"xm");
Topic<RPY> gyro_topic(5400,"gyro");
Topic<electricalStruct> electrical_topic(5500,"light");

Topic<tmStructIMU> tm_topic_imu (5002, "Telemtry");
Topic<tmStructElectrical> tm_topic_electrical (5003, "Telemtry");


/* Uncomment if you want to use something */

IMU imu("imu");
Telemetry telemetry("Telemetry");
Electrical electrical("electrical");
Camera camera("camera",Teleuart);



HAL_GPIO GreenLED(LED_GREEN);
HAL_GPIO RedLED(LED_RED);
HAL_GPIO BlueLED(LED_BLUE);
HAL_GPIO OrangeLED(LED_ORANGE);


HAL_I2C HAL_I2C_2(I2C_IDX2);


/***********************************************************************/

class Telecommand : public Thread {

public:
	Telecommand(const char* name) : Thread(name) {
		STATE=WAITING;
	}

	void init() {

	}

	void run() {
		int id;
		char msg[3];
		int value;
		char current_char;
		int bytes = 0;
		int channel;
		bool got_channel = false;
		while(1) {

			Teleuart.suspendUntilDataReady();
			Teleuart.read(&current_char,1);
			switch(STATE) {
				case WAITING:
					if (compare(&current_char,(char*)"$")) {
						STATE=GET_ID;
					} else {
//						PRINTF("ERROR, wrong start-bit! Expected $, got %c! Please try again!\n",current_char);
					}
					break;
				case GET_ID:
					id = atoi(&current_char);
					STATE=GET_MSG;
//					PRINTF("ID: %d ",id);
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
					if (id == 3 && !got_channel) {
						channel = atoi(&current_char);
						got_channel = true;
//						PRINTF("Channel: %d ",channel);
						if (channel == 4) {
							STATE=GET_BOOL;
						}
						break;
					} else {
						if (bytes < 4) {
							msg[bytes] = current_char;
							bytes++;
						}
						if (bytes == 4) {
							value = atoi(msg);
//							PRINTF("Int: %d ",value);
							STATE=GET_STOP;
							bytes = 0;
						}
						break;
					}
				case GET_BOOL:
					value = atoi(&current_char);
//					PRINTF("Bool: %d ",value);
					STATE=GET_STOP;
					break;
				case GET_STOP:
					if (compare(&current_char,(char*)"#")) {
//						PRINTF("DONE\n");
						STATE=WAITING;
						processTelecommand(&id, &channel, &value);
					}
					break;
			}

		}
	}

	void checkIdentifier(char* identifier) {
		/* ID 1 - IMU */
		//Calibrate Magnetometer
		if (compare(identifier, (char*)"CMA")) {
			STATE=GET_BOOL;
			TELECOMMAND=CALIBRATE_MAG;
		}
		//Calibrate Accelerometer
		else if (compare(identifier, (char*)"CAC")) {
			STATE=GET_BOOL;
			TELECOMMAND=CALIBRATE_ACC;
		}
		//Calibrate Gyroscope
		else if (compare(identifier, (char*)"CGY")) {
			STATE=GET_BOOL;
			TELECOMMAND=CALIBRATE_GYR;
		}
		//Send IMU data
		else if (compare(identifier, (char*)"SND")) {
			STATE=GET_BOOL;
			TELECOMMAND=SET_IMU;
		}

		/* ID 2 - AHRS */
		//Activate/Deactivate AHRS-Fusion
		else if (compare(identifier, (char*)"FUS")) {
			STATE=GET_BOOL;
			TELECOMMAND=CALCULATE_AHRS;
		}

		/* ID 3 - Electrical */
		//Set speed MAIN ENGINE
		else if (compare(identifier, (char*)"SPA")) {
			STATE=GET_INT;
			TELECOMMAND=ELECTRICAL_SET_SPEED_MAIN;
		}
		//Set speed deployment 1 - B
		else if (compare(identifier, (char*)"SPB")) {
			STATE=GET_INT;
			TELECOMMAND=ELECTRICAL_SET_SPEED_DEPLOY_B;
		}
		//Set speed deployment 2 - C
		else if (compare(identifier, (char*)"SPC")) {
			STATE=GET_INT;
			TELECOMMAND=ELECTRICAL_SET_SPEED_DEPLOY_C;
		}
		//Steer to
		else if (compare(identifier, (char*)"STE")) {
			STATE=GET_INT;
			TELECOMMAND=ELECTRICAL_STEER_TO;
		}
		//Activate/Deactivate knife
		else if (compare(identifier, (char*) "AKN")) {
			STATE=GET_BOOL;
			TELECOMMAND=ELECTRICAL_ACTIVATE_KNIFE;
		}
		//Activate/Deactivate magnet
		else if (compare(identifier, (char*) "AMA")) {
			STATE=GET_BOOL;
			TELECOMMAND=ELECTRICAL_ACTIVATE_MAGNET;
		}
		//Activate/Deactivate lightsensor
		else if (compare(identifier, (char*) "ALS")) {
			STATE=GET_BOOL;
			TELECOMMAND=ELECTRICAL_ACTIVATE_LIGHTSENSOR;
		}

		/* ID 4 - Camera */
		else if (compare(identifier, (char*) "PIC")) {
			STATE=GET_BOOL;
			TELECOMMAND=TAKE_PICTURE;
		}

		/* ID 5 - Telemetry */
		//Activate/deactivate Telemetry
		else if (compare(identifier, (char*) "TEL")) {
			STATE=GET_BOOL;
			TELECOMMAND=SET_TELEMETRY;
		}

		//default
		else {
			PRINTF("ERROR, wrong syntax! Please try again!\n");
			STATE=WAITING;
		}
//		PRINTF("Msg: %3s ",identifier);
	}

	void processTelecommand(int *id, int *channel, int *value) {
		switch (TELECOMMAND) {
			case CALIBRATE_MAG:
				imu.setCalibrateMagnetometer();
				break;
			case CALIBRATE_ACC:
				imu.setCalibrateAccelerometer();
				break;
			case CALIBRATE_GYR:
				imu.setCalibrateGyroscope();
				break;
			case SET_IMU:
				if (*value == 1) {
					imu.resume();
				} else {
					imu.yield();
				}
				break;
			case CALCULATE_AHRS:
				if (*value == 1) {
//					ahrs.resume();
				} else {
//					ahrs.yield();
				}
				break;
			case ELECTRICAL_STEER_TO:
				//TODO write function
				break;
			case ELECTRICAL_SET_SPEED_MAIN:
				electrical.setMainMotorSpeed(value);
				//motor.setSpeed(channel, value);
				break;
			case ELECTRICAL_SET_SPEED_DEPLOY_B:
				electrical.setDeployment1Speed(value);
				//motor.setSpeed(channel, value);
				break;
			case ELECTRICAL_SET_SPEED_DEPLOY_C:
				electrical.setDeployment2Speed(value);
				//motor.setSpeed(channel, value);
				break;
			case ELECTRICAL_ACTIVATE_KNIFE:
				electrical.setKnife(value);
				break;
			case ELECTRICAL_ACTIVATE_MAGNET:
				electrical.setMagnet(value);
				break;
			case ELECTRICAL_ACTIVATE_LIGHTSENSOR:
				electrical.setLightsensor(value);
				break;
			case SET_TELEMETRY:
				telemetry.setFlag(value);
				break;
			case TAKE_PICTURE:
				if (*value == 1) {
					camera.turnOn();
				}
				break;
		}
	}

private:
	enum topic_ids {
		IMU = 1,
		AHRS,
		PWM,
		CAMERA,
		TELEMETRY,
	};

	enum states {
		WAITING,
		GET_ID,
		GET_MSG,
		GET_INT,
		GET_BOOL,
		GET_STOP,
	} STATE;

	enum telecommands {
		CALIBRATE_MAG,
		CALIBRATE_ACC,
		CALIBRATE_GYR,
		SET_IMU,
		CALCULATE_AHRS,
		ELECTRICAL_STEER_TO,
		ELECTRICAL_SET_SPEED_MAIN,
		ELECTRICAL_SET_SPEED_DEPLOY_B,
		ELECTRICAL_SET_SPEED_DEPLOY_C,
		ELECTRICAL_ACTIVATE_KNIFE,
		ELECTRICAL_ACTIVATE_MAGNET,
		ELECTRICAL_ACTIVATE_LIGHTSENSOR,
		SET_TELEMETRY,
		TAKE_PICTURE,
		ACTIVATE_CAMERA,
	} TELECOMMAND;


};
Telecommand telecommand("telecommand");
