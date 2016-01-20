/*
 * Telemetry.cpp
 *
 *  Created on: 13.12.2015
 *      Author: JackVEnterprises
 */

#include "Telemetry.h"

Telemetry::Telemetry(const char* name) : Thread(name) {
	timer = 250*MILLISECONDS;
}

Telemetry::~Telemetry() {

}

void Telemetry::init() {

}

void Telemetry::run() {
	imuData imu;
	imuPublish ahrs;
	RPY xm;
	RPY gyro;
	electricalStruct lightValues;
//	suspendCallerUntil();
	while (1) {
		suspendCallerUntil(NOW() + 250*MILLISECONDS);
		if (timer) {
			imuBuffer.get(imu);
			ahrsBuffer.get(ahrs);
			gyroBuffer.get(gyro);
			xmBuffer.get(xm);
			electricalBuffer.get(lightValues);
			if (!imu.calibrating) {
/*				PRINTF("Acc %5.2f %5.2f %5.2f\n", imu.ax, imu.ay, imu.az);
*/				PRINTF("Mag %5.2f %5.2f %5.2f\n", imu.mx, imu.my, imu.mz);
/*				PRINTF("Gyr %5.2f %5.2f %5.2f\n", radToDeg(imu.wx), radToDeg(imu.wy), radToDeg(imu.wz));
				PRINTF("XM  %5.2f %5.2f %5.2f\n",radToDeg(xm.roll),radToDeg(xm.pitch),radToDeg(xm.heading));
				PRINTF("Gyr %5.2f %5.2f %5.2f\n",radToDeg(gyro.roll),radToDeg(gyro.pitch),radToDeg(gyro.heading));
*/				PRINTF("Com %5.2f %5.2f %5.2f\n", radToDeg(ahrs.roll), radToDeg(ahrs.pitch), radToDeg(ahrs.heading));
			}
			if (lightValues.light_status) {
				PRINTF("Lightsensor: %05d\n",lightValues.light_raw);
			}
		}
	}
}

void Telemetry::setFlag(int *flag) {
	timer = *flag;
}


