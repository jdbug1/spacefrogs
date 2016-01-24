/*
 * Controller.h
 *
 *  Created on: 23 jan 2016
 *      Author: Kajsa
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define speed_control	false
#define heading_control	true

#include "PID.h"
#include "PI.h"

class Controller : public Thread{
private:
	PID* pid;
	PI*  pi;
	Electrical* El;
	bool control;

public:
	Controller(const char* name, Electrical* El);
	virtual ~Controller();

	void init();
	void run();
	void set_control(bool control);

};

#endif /* CONTROLLER_H_ */
