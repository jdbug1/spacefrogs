/*
 * Electrical.h
 *
 *  Created on: 10.12.2015
 *      Author: JackVEnterprises
 */

#ifndef ELECTRICAL_H_
#define ELECTRICAL_H_

#include "rodos.h"
#include "hal.h"
#include "basics.h"

class Electrical : public Thread {
public:
	Electrical(const char* name);
	virtual ~Electrical();

	void init();
	void run();

	void setMainMotorSpeed(int *speed);
	void setDeployment1Speed(int *speed);
	void setDeployment2Speed(int *speed);
	void setKnife(int *status);
	void setMagnet(int *status);
	void readLightsensor(int16_t *channel_0, int16_t *channel_1);
	void setLightsensor(int *value);


	bool read_lightsensor;
	bool em, knife;

};

#endif /* ELECTRICAL_H_ */
