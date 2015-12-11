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

class Electrical : public Thread {
public:
	Electrical(const char* name);
	virtual ~Electrical();

	void init();
	void run();
};

#endif /* ELECTRICAL_H_ */
