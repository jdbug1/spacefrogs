/*
 * Telemetry.cpp
 *
 *  Only used for debugging tasks
 *  TODO remove before final presentation and if everything else works
 *  @author	Sven Jörissen
 *  @date	13.12.2015
 *
 */

#include "Telemetry.h"

Telemetry::Telemetry(const char* name) : Thread(name) {
	timer = 0;
}

Telemetry::~Telemetry() {

}

void Telemetry::init() {

}

void Telemetry::run() {
	imuData imu;
	RPY xm;
	RPY gyro;
	tmStructElectrical lightValues;
//	suspendCallerUntil();
	while (1) {
		suspendCallerUntil(NOW() + TELEMETRY_SAMPLING_RATE*MILLISECONDS);
		if (timer) {
			imuBuffer.get(imu);
			electricalBuffer.get(lightValues);
			if (!imu.calibrating) {
/*				PRINTF("Acc %5.2f %5.2f %5.2f\n", imu.ax, imu.ay, imu.az);
				PRINTF("Mag %5.2f %5.2f %5.2f\n", imu.mx, imu.my, imu.mz);
				PRINTF("Gyr %5.2f %5.2f %5.2f\n", radToDeg(imu.wx), radToDeg(imu.wy), radToDeg(imu.wz));
*/				PRINTF("XM  %5.2f\n",radToDeg(imu.xm_heading));
				PRINTF("Gyr %5.2f\n",radToDeg(imu.gyro_heading));
				PRINTF("Com %5.2f\n", radToDeg(imu.heading));
			}
			if (lightValues.light_status) {
				PRINTF("Lightsensor: %d\n",lightValues.lightsensor_value);
			}
		}
	}
}

void Telemetry::setFlag(int *flag) {
	timer = *flag;
}


