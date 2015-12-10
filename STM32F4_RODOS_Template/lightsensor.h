/*
 * lightsensor.h
 *
 *  Created on: 07.12.2015
 *      Author: JackVEnterprises
 */

#ifndef LIGHTSENSOR_H_
#define LIGHTSENSOR_H_

#include "rodos.h"
#include "hal.h"
#include "basics.h"



class lightsensor : public Thread {
public:
	lightsensor(const char* name);
	virtual ~lightsensor();

	void init();
	void run();

	void readRaw(int16_t *channel_1, int16_t *channel_2);
};

#endif /* LIGHTSENSOR_H_ */
