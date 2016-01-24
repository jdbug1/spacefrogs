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
		float P, I, D, N;

		new_angle = get_Angle();
		e = ref_Ang - new_angle;

		if (pi->get_State() == closed){
			P = P1_closed;
			I = I1_closed;
			D = D1_closed;
			N = N_closed;
			speed_lim = wl_closed;
		} else if (pi->get_State() == deployed){
			P = P1_deployed;
			I = I1_deployed;
			D = D1_deployed;
			N = N_deployed;
			speed_lim = wl_deployed;
		} else if (pi->get_State() == extended){
			P = P1_extended;
			I = I1_extended;
			D = D1_extended;
			N = N_extended;
			speed_lim = wl_extended;
		}

		P_term = P*e;

		i_temp += e*Ts2;
		if (i_temp < iMin){
			i_temp = iMin;
		} else if (i_temp > iMax){
			i_temp = iMax;
		}
		I_term = I*i_temp;

		D_term = D*(d_temp - e)/Ts2; // I need to add N here in some way?????
		d_temp = e;

		ref_speed = P_term + I_term + D_term;

		if (ref_speed < -speed_lim){
			ref_speed = -speed_lim;
		} else if (ref_speed > speed_lim){
			ref_speed = speed_lim;
		}

		pi->set_Velocity(ref_speed);
	}

	float PID::get_Angle() {
		ahrsBuffer.get(imu);
		float heading = imu.heading;
		// Some filtering code
		return heading;
	}

	PID::PID(const char* name, PI* pi){
		i_temp = 0;
		d_temp = 0;
		PWM_temp = 0;
		ref_Ang = 0;
		this->pi = pi;
	}

	PID::~PID() {

	}
