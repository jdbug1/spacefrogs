/*
 * Telemetry.h
 *
 *  Created on: 13.12.2015
 *      Author: JackVEnterprises
 */

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include "rodos.h"
#include "basics.h"

class Telemetry : public Thread{
public:
	Telemetry(const char* name);
	virtual ~Telemetry();

	void init();
	void run();

	void setFlag(int *flag);

private:
	int timer;

};

#endif /* TELEMETRY_H_ */
