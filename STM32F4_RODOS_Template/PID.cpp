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
		float new_angle;
		float Duty_Cycle;

		new_angle = get_Angle();
		e = ref_Ang - new_angle;

		P_term = P1*e;

		i_temp += e;
		if (i_temp < iMin){
			i_temp = iMin;
		} else if (i_temp > iMax){
			i_temp = iMax;
		}
		I_term = I1*i_temp;

		D_term = D1*(d_temp - e);
		d_temp = e;

		Duty_Cycle = PWM_temp - (P_term + I_term + D_term);

		if (Duty_Cycle < -100){
			Duty_Cycle = -100;
		} else if (Duty_Cycle > 100){
			Duty_Cycle = 100;
		}
		int DC = int(Duty_Cycle);

		//electrical->setMainMotorSpeed(&DC);
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

	void PID::init() {

	}
	void PID::run() {
		uint16_t t1, t2;
		t1 = NOW();

		t2 = NOW();
	    suspendCallerUntil(NOW() + Ts1 - (t2-t1));
	}

