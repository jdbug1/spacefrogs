/*
 * imu.h
 *
 *  @author	Sven Jörissen
 *  @date	09.11.2015
 */

#ifndef IMU_H_
#define IMU_H_

#include "rodos.h"
#include "hal.h"
#include "basics.h"

class IMU : public Thread {
public:
	IMU(const char* name);
	virtual ~IMU();

	void setCalibrateMagnetometer();
	void setCalibrateGyroscope();
	void setCalibrateAccelerometer();

private:

	void init();
	void initSensors();
	void run();
	void I2CError();

	void readGyro();
	void calculateGyro();
	void calibrateGyro();

	void readAcc();
	void calculateAcc();
	void calibrateAcc();

	void readMag();
	void calculateMag();
	void calibrateMag();

	void calculateAccEuler();
	void calculateHeading();
	void calculateGyroEuler();
	void complementaryFilter();

private:
	int g_counter, a_counter, m_counter;
	xyz16 a_offset, a_raw, m_raw, g_raw;
	xyz32 a_temp, m_temp, g_temp;
	magMaxMin m_offset;
	imuData imu_data;
	xyzFloat g_offset;
	RPY xm_euler, gyro_euler, ahrs_euler;
	ahrsPublish publish;
	bool calibrate_magnetometer, calibrate_gyroscope, calibrate_accelerometer;
};

#endif
