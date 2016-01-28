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

class IMU : public Thread, public SubscriberReceiver<tcStruct> {
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
	void put(tcStruct &command);
	void handleTelecommand(tcStruct *tc);

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
	void changeGain(float a);

private:
	tcStruct current_tc;
	float alpha;
	xyz16 a_offset, g_offset, a_raw, m_raw, g_raw;
	magMaxMin m_offset;
	imuData imu_data;
	RPY xm_euler, gyro_euler, ahrs_euler;
	tmStructIMU publish;
	bool calibrate_magnetometer, calibrate_gyroscope, calibrate_accelerometer;
};

#endif
