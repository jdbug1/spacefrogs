/*
 * PI.cpp
 *
 *  Created on: 20 jan 2016
 *      Author: Kajsa
 */

#include "PID.h"
int counter;
	void PID::Change_Ref_Vel(){
		float e;
		float new_angle;
		float ref_speed;
		counter++;
		new_angle = get_Angle();
		e = ref_Ang - new_angle;

		float e_slope = (e - e_old);

		e_sum += e;

		if (e_sum > 100){
			e_sum = 100;
		} else if (e_sum < -100) {
			e_sum = -100;
		} else if ((e < 0.3) && (e > -0.3)) {
			e_sum = 0.0;
			e = 0.0;
		}

		float P_term = Po * e;
		float I_term = Io * e_sum;
		float D_term = Do * e_slope;

		ref_speed = P_term + I_term + D_term;
		if (counter == 15) {
			PRINTF("Speed %3.2f Angle is %3.2f Ref is %3.2f E %3.2f ESum %5.2f P %3.2f I %3.2f d %3.2f\n",ref_speed, new_angle, ref_Ang,e,e_sum,P_term,I_term, D_term);
			counter = 0;
		}
		if (ref_speed < -speed_lim){
			ref_speed = -speed_lim;
		} else if (ref_speed > speed_lim){
			ref_speed = speed_lim;
		}

		e_old = e;

		e_old = e;

		pi->set_Velocity(ref_speed*M_PI/180.0);
		pi->Change_Duty_Cycle();
	}

	float PID::get_Angle() {
		imuBuffer.get(imu);
		return (imu.heading*180.0/M_PI);
	}

	PID::PID(const char* name, PI* pi){
		e_sum = 0;
		e_old = 0;
		ref_Ang = 0;
		this->pi = pi;
	}

	PID::~PID() {

	}

	void PID::Set_Ref_Angle(float ref_angle) {
		this->ref_Ang = ref_angle*180.0/M_PI;
	}
