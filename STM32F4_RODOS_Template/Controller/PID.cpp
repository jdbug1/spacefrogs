/*
 * PI.cpp
 *
 *  Created on: 20 jan 2016
 *      Author: Kajsa
 */

#include "PID.h"

	void PID::Change_Ref_Vel(){
		float e;
		float new_angle;
		float ref_speed;

		new_angle = get_Angle();
		e = ref_Ang - new_angle;

		float e_slope = (e - e_old);

		e_sum += e;

		if (e_sum > 100){
			e_sum = 100;
		} else if (e_sum < -100) {
			e_sum = -100;
		}

		float P_term = Po * e;
		float I_term = Io * e_sum;
		float D_term = Do * e_slope;


		ref_speed = P_term + I_term + D_term;

		if (ref_speed < -speed_lim){
			ref_speed = -speed_lim;
		} else if (ref_speed > speed_lim){
			ref_speed = speed_lim;
		}

		e_old = e;

		pi->set_Velocity(ref_speed);
	}

	float PID::get_Angle() {
		ahrsBuffer.get(imu);
		return imu.heading;
	}

	PID::PID(const char* name, PI* pi){
		e_sum = 0;
		e_old = 0;
		ref_Ang = 0;
		this->pi = pi;
	}

	PID::~PID() {

	}
