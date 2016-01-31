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

class Controller : public Thread, public SubscriberReceiver<tcStruct> {
private:
	PID* pid;
	PI*  pi;
	Electrical* El;
	bool control, enable_control;

public:
	Controller(const char* name, Electrical* El);
	virtual ~Controller();

	void init();
	void run();
	void put(tcStruct &command);
	void handleTelecommand(tcStruct *tc);

	void set_control(bool control);
	void set_Velocity(float rev_val);
	void set_Reference_Angle(float ref_angle);
	void enableControl();

};

#endif /* CONTROLLER_H_ */
