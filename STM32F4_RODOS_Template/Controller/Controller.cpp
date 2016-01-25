/*
 * Controller.cpp
 *
 *  Created on: 23 jan 2016
 *      Author: Kajsa
 */

#include "Controller.h"

void * operator new(size_t size)
{
  return malloc(size);
}

	Controller::Controller(const char* name, Electrical* El){
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
		uint16_t t1, t2;
		while(true){
			t1 = NOW();

			if (control == speed_control){
				pi->Change_Duty_Cycle();
			}
			else if (control == heading_control){
				pid->Change_Ref_Vel();
			}

			t2 = NOW();
			uint16_t delay = Ts1 - (t2-t1);
			PRINTF("Delay: %u/n", delay);
			if (delay > 0) {
				suspendCallerUntil(NOW() + Ts1 - (t2-t1));
			} else {
				PRINTF("Loop took too long \n");
			}
		}
	}

	void Controller::set_control(bool control){
		this->control = control;
	}

	void Controller::set_State(int state){
		this->pi->set_State(state);
	}

