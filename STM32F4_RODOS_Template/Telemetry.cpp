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
	ahrsPublish ahrs;
	RPY xm;
	RPY gyro;
	electricalStruct lightValues;
//	suspendCallerUntil();
	while (1) {
		suspendCallerUntil(NOW() + TELEMETRY_SAMPLING_RATE*MILLISECONDS);
		if (timer) {
			imuBuffer.get(imu);
			ahrsBuffer.get(ahrs);
			electricalBuffer.get(lightValues);
			if (!imu.calibrating) {
/*				PRINTF("Acc %5.2f %5.2f %5.2f\n", imu.ax, imu.ay, imu.az);
				PRINTF("Mag %5.2f %5.2f %5.2f\n", imu.mx, imu.my, imu.mz);
/*				PRINTF("Gyr %5.2f %5.2f %5.2f\n", radToDeg(imu.wx), radToDeg(imu.wy), radToDeg(imu.wz));
				PRINTF("XM  %5.2f %5.2f %5.2f\n",radToDeg(xm.roll),radToDeg(xm.pitch),radToDeg(xm.heading));
				PRINTF("Gyr %5.2f %5.2f %5.2f\n",radToDeg(gyro.roll),radToDeg(gyro.pitch),radToDeg(gyro.heading));
				PRINTF("Com %5.2f %5.2f %5.2f\n", radToDeg(ahrs.roll), radToDeg(ahrs.pitch), radToDeg(ahrs.heading));
*/			}
			if (lightValues.light_status) {
				PRINTF("Lightsensor: %05f\n",lightValues.light);
			}
		}
	}
}

void Telemetry::setFlag(int *flag) {
	timer = *flag;
}


