/*
 * pwm.h
 *
 *  Created on: 03.12.2015
 *      Author: JackVEnterprises
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "rodos.h"
#include <stdio.h>
#include "hal.h"
#include "basics.h"

class MOTOR : public Thread{
public:
	MOTOR(const char* name);
	virtual ~MOTOR();
	void init();
	void run();

};

#endif /* MOTOR_H_ */
