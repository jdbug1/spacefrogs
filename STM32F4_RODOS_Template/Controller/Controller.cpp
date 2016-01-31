/*
 * Controller.cpp
 *
 *  Created on: 23 jan 2016
 *      Author: Kajsa
 */

#include "Controller.h"

	Controller::Controller(const char* name, Electrical* El) : SubscriberReceiver<tcStruct>(tm_topic_incoming, "SubRec Controller for Telecommands") {
		this->El = El;
		pi 		 = new PI("PI", El);
		pid 	 = new PID("PID", El);
		control = speed_control;
		enable_control = false;
	}
	Controller::~Controller(){

	}

	void Controller::init(){

	}

void Controller::run(){
	int64_t t1, t2;
	while (true) {
		t1 = NOW();
		if (enable_control) {
			if (control == speed_control) {
				pi->Change_Duty_Cycle();
			}
			else if (control == heading_control) {
				pid->Change_Ref_Vel();
			}
		} else {
			pi->stopMotor();
		}
		t2 = NOW();
		int64_t delay = 10 - ((t2-t1)/1000000.0);
		//			PRINTF("Delay in ms: %lld\n", delay);
		if (delay > 0) {
			suspendCallerUntil(NOW() + (Tms*MILLISECONDS - ((t2-t1)/1000000.0)));
		} else {
			PRINTF("Loop took too long \n");
		}

	}
}

	void Controller::put(tcStruct &command) {
		if (command.id == 2) {
			this->handleTelecommand(&command);
		}
	}

	void Controller::handleTelecommand(tcStruct * tc) {
		int command = tc->command;
		PRINTF("Command was %d\n",command);
		switch (command) {
		case 2001:
			PRINTF("Speed set to %d deg/s\n", tc->value);
			this->control = speed_control;
			this->set_Velocity(tc->value * M_PI / 180.0);
			break;
		case 2002:
			PRINTF("Steer to %d deg\n", tc->value);
			this->control = heading_control;
			this->set_Reference_Angle(tc->value * M_PI / 180.0);
			break;
		case 2003:
			this->enable_control = (bool)tc->value;
			break;
		}
	}


	void Controller::set_control(bool control){
		this->control = control;
	}

	void Controller::set_Velocity(float rev_val) {
		//PRINTF("Rotating at %f\n", rev_val*180.0/M_PI);
		pi->set_Velocity(rev_val);
	}

	void Controller::set_Reference_Angle(float ref_angle) {
		//PRINTF("Stabilizing satellite at %d degrees\n",(int)radToDeg(ref_angle));
		pid->Set_Ref_Angle(ref_angle);
	}

	void Controller::enableControl() {
		this->enable_control = true;
	}

