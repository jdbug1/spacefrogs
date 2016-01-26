/*
 * PI.cpp
 *
 *  Created on: 23 jan 2016
 *      Author: Kajsa
 */

#include "PI.h"

	void PI::Change_Duty_Cycle(){
		float e;
		float new_Vel;

		new_Vel = get_Velocity();
		e = ref_Vel - new_Vel;

		e_sum += e;

		if (e_sum > 100){
			e_sum = 100;
		} else if (e_sum < -100){
			e_sum = -100;
		}

		float P_term = Pi*e;
		float I_term = Ii*e_sum;

		DC = DC - P_term - I_term;

		if (DC > 100){
			DC = 100;
		} else if (DC < -100) {
			DC = -100;
		}

		PRINTF("DC is %d\n", DC);

		El->setMainMotorSpeed(&DC);
	}

	float PI::get_Velocity() {
		ahrsBuffer.get(imu);
		float ang = imu.heading;
		float velocity = (ang - ang_temp)/Ts;
		ang_temp = ang;
		return velocity;
	}

	PI::PI(const char* name, Electrical* El){
		P_term = 0;
		I_term = 0;
		e_sum = 0;
		ref_Vel = 0;
		ang_temp = imu.heading;
		DC = 0;
		this->El = El;
	}

	PI::~PI() {

	}

	void PI::set_Velocity(float ref_vel){
		ref_vel = ref_Vel;
	}

