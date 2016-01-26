/*
 * PI.cpp
 *
 *  Created on: 20 jan 2016
 *      Author: Kajsa
 */

#include "PID.h"

	void PID::Change_Ref_Vel(){
		float iMax =  Ibound1;
		float iMin = -Ibound1;
		float e;
		float P_term;
		float I_term;
		float D_term;
		float speed_lim;
		float new_angle;
		float ref_speed;
		float a, b, c;

		new_angle = get_Angle();
		e = ref_Ang - new_angle;

		if (pi->get_State() == closed){
			a = P1_closed + I1_closed * Ts2 / 2 + D1_closed / Ts2;
			b = -P1_closed + I1_closed * Ts2 / 2 - 2 * D1_closed / Ts2;
			c = D1_closed / Ts2;
			speed_lim = wl_closed;
		} else if (pi->get_State() == deployed){
			a = P1_deployed + I1_deployed * Ts2 / 2 + D1_deployed / Ts2;
			b = -P1_deployed + I1_deployed * Ts2 / 2 - 2 * D1_deployed / Ts2;
			c = D1_deployed / Ts2;
			speed_lim = wl_deployed;
		} else if (pi->get_State() == extended){
			a = P1_extended + I1_extended * Ts2 / 2 + D1_extended / Ts2;
			b = -P1_extended + I1_extended * Ts2 / 2 - 2 * D1_extended / Ts2;
			c = D1_extended / Ts2;
			speed_lim = wl_extended;
		}

		ref_speed = a*e + b*e_1 + c*e_2;

		if (ref_speed < -speed_lim){
			ref_speed = -speed_lim;
		} else if (ref_speed > speed_lim){
			ref_speed = speed_lim;
		}

		e_2 = e_1;
		e_1 = e;

		pi->set_Velocity(ref_speed);
	}

	float PID::get_Angle() {
		ahrsBuffer.get(imu);
		return imu.heading;
	}

	PID::PID(const char* name, PI* pi){
		e_1 = 0;
		e_2 = 0;
		ref_Ang = 0;
		this->pi = pi;
	}

	PID::~PID() {

	}
