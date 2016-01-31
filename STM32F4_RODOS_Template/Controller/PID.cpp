/*
 * PI.cpp
 *
 *  Created on: 20 jan 2016
 *      Author: Kajsa
 */

#include "PID.h"
int counter;

int PID::calculateDC (float error){
	int a = Kp + 0.5*Ts*Ki + Kd/Ts;
	int b = -Kp +  0.5*Ki*Ts - 2*Kd/Ts;

	int u;
	u = a*error+b*error_1+u_1;
	error_1 = error;
	u_1 = u;
	return u ;
}

void PID::Change_Ref_Vel(){
		float e;
		float new_angle;
		new_angle = get_Angle();
		e = ref_Ang - new_angle;

		int dc = calculateDC(e);
		if (dc > speed_lim) {
			dc = speed_lim;
		} else if (dc < -speed_lim) {
			dc = -speed_lim;
		}
		el->setMainMotorSpeed(&dc);
	}

	float PID::get_Angle() {
		imuBuffer.get(imu);
		return (radToDeg(imu.heading));
	}

	PID::PID(const char* name, Electrical* El){
		e_sum = 0;
		e_old = 0;
		ref_Ang = 0;
		this->el = El;
	}

	PID::~PID() {

	}

	void PID::Set_Ref_Angle(float ref_angle) {
		this->ref_Ang = ref_angle*180.0/M_PI;
	}
