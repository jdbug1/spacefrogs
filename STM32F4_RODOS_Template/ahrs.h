#ifndef AHRS_H_
#define AHRS_H_

#include "rodos.h"
#include "hal.h"
#include "basics.h"

class AHRS : public Thread{
public:
	AHRS(const char* name);
	virtual ~AHRS();
	void init();
	void run();

	void calculateAccEuler();
	void calculateHeading();
	void calculateGyroEuler();
	void complementaryFilter();


private:
	imuData imu_data;
	RPY xm_euler, gyro_euler, ahrs_euler, ahrs_temp;
	int ahrs_counter;
};

#endif /* AHRS_H_ */
