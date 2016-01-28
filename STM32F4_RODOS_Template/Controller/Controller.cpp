/*
 * Controller.cpp
 *
 *  Created on: 23 jan 2016
 *      Author: Kajsa
 */

#include "Controller.h"

	//Hello Kajsa
	Controller::Controller(const char* name, Electrical* El) : SubscriberReceiver<tcStruct>(tm_topic_incoming, "SubRec Controller for Telecommands") {
		this->El = El;
		pi 		 = new PI("PI", El);
		pid 	 = new PID("PID", pi);
		control = speed_control;
	}
	Controller::~Controller(){

	}

	void Controller::init(){

	}

	void Controller::run(){
		int64_t t1, t2;
		while(true){
			t1 = NOW();

			if (control == speed_control){
				pi->Change_Duty_Cycle();
			}
			else if (control == heading_control){
				pid->Change_Ref_Vel();
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
			this->control = speed_control;
			this->set_Velocity(tc->value * M_PI / 180.0);
			break;
		case 2002:
			this->control = heading_control;
			this->set_Reference_Angle(tc->value * M_PI / 180.0);
			break;
		}
	}


	void Controller::set_control(bool control){
		this->control = control;
	}

	void Controller::set_Velocity(float rev_val) {
		PRINTF("Rotating at %f\n", rev_val*180.0/M_PI);
		pi->set_Velocity(rev_val);
	}

	void Controller::set_Reference_Angle(float ref_angle) {
		pid->Set_Ref_Angle(ref_angle);
	}

