#ifndef IMU_H_
#define IMU_H_

#include "rodos.h"
#include "hal.h"
#include "basics.h"

class IMU : public Thread {
public:
	IMU(const char* name);
	virtual ~IMU();

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



private:
	int g_counter, a_counter, m_counter;
	xyz16 a_offset, a_raw, m_raw, g_raw;
	xyz32 a_temp, m_temp, g_temp;
	magMaxMin m_offset;
	imuData values;
	xyzFloat g_offset;
};

#endif
