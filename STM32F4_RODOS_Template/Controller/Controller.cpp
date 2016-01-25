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
				suspendCallerUntil(NOW() + (Ts1*MILLISECONDS - ((t2-t1)/1000000.0)));
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

	void Controller::set_Velocity(float rev_val) {
		pi->set_Velocity(rev_val);
	}

