#include "AHRS.h"

AHRS::AHRS(const char* name) : Thread(name) {
	ahrs_counter = 0;
}

AHRS::~AHRS() {
	ahrs_euler.heading = ahrs_euler.roll = ahrs_euler.pitch = 0.0;
	gyro_euler.heading = gyro_euler.roll = gyro_euler.pitch = 0.0;
	xm_euler.heading = xm_euler.roll = xm_euler.pitch = 0.0;
}

void AHRS::init() {

}

void AHRS::run() {
	suspendCallerUntil(NOW()+1*SECONDS);
	while(1) {
		imuBuffer.get(imu_data);
		if (!imu_data.calibrating) {
			calculateAccEuler();
			calculateHeading();
			calculateGyroEuler();
			complementaryFilter();
			if (ahrs_counter < MEAN_FILTER_SAMPLES) {
				ahrs_temp.roll += ahrs_euler.roll;
				ahrs_temp.pitch += ahrs_euler.pitch;
				ahrs_temp.heading += ahrs_euler.heading;
				ahrs_counter++;
			}
			if (ahrs_counter == MEAN_FILTER_SAMPLES) {
				ahrs_temp.roll /= (float)MEAN_FILTER_SAMPLES;
				ahrs_temp.pitch /= (float)MEAN_FILTER_SAMPLES;
				ahrs_temp.heading /= (float)MEAN_FILTER_SAMPLES;
				gyro_topic.publish(gyro_euler);
				xm_topic.publish(xm_euler);
				ahrs_topic.publish(ahrs_temp);
				ahrs_temp.roll = ahrs_temp.pitch = ahrs_temp.heading = 0.0;
				ahrs_counter = 0;

			}
		}
		suspendCallerUntil(NOW()+AHRS_SAMPLING_RATE*MILLISECONDS);
	}
}

void AHRS::calculateAccEuler() {
	xm_euler.roll = imu_data.ax / sqrt(imu_data.ay*imu_data.ay + imu_data.az*imu_data.az);
	xm_euler.pitch = imu_data.ay / sqrt(imu_data.ax*imu_data.ax + imu_data.az*imu_data.az);
}

void AHRS::calculateHeading() {
	float sr = sin(xm_euler.roll);
	float cr = cos(xm_euler.roll);
	float sp = sin(xm_euler.pitch);
	float cp = cos(xm_euler.pitch);
	float mxh, myh = 0.0;
	mxh = imu_data.mx * cp + imu_data.mz * sp;
	myh = imu_data.mx * sr * sp + imu_data.my * cr - imu_data.mz * sr * cp;
	xm_euler.heading = atan2(myh,mxh);
	if (xm_euler.heading < 0) xm_euler.heading +=2*M_PI;
}

void AHRS::calculateGyroEuler() {
	float sr = sin(ahrs_euler.roll);
	float cr = cos(ahrs_euler.roll);
	float sp = sin(ahrs_euler.pitch);
	float cp = cos(ahrs_euler.pitch);

	if (cp != 0) {
		gyro_euler.pitch = ahrs_euler.pitch + 	(((cr*cp*imu_data.wy - sr*cp*imu_data.wz)/cp)*AHRS_SAMPLING_RATE/1000.0);
		gyro_euler.roll = ahrs_euler.roll + 	(((cp*imu_data.wx + sr*sp*imu_data.wy + cr*sp*imu_data.wz)/cp)*AHRS_SAMPLING_RATE/1000.0);
		gyro_euler.heading = ahrs_euler.heading + (((sr*imu_data.wy + cr*imu_data.wz)/cp)*AHRS_SAMPLING_RATE/1000.0);
	}

}

void AHRS::complementaryFilter() {
	if (!isnan(gyro_euler.pitch) | !isnan(gyro_euler.roll) | !isnan(gyro_euler.heading)) {
		ahrs_euler.pitch = ((0.1)*xm_euler.pitch) + (0.9*gyro_euler.pitch);
		ahrs_euler.roll = ((0.1)*xm_euler.roll) + (0.9*gyro_euler.roll);
		ahrs_euler.heading = ((0.1)*xm_euler.heading) + (0.9*gyro_euler.heading);
	}
	//PRINTF("%f %f %f \n%f %f %f\n",xm_euler.pitch,xm_euler.roll,xm_euler.heading,gyro_euler.pitch,gyro_euler.roll,gyro_euler.heading);
}
