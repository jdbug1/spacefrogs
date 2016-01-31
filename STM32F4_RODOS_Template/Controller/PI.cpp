/*
 * PI.cpp
 *
 *  Created on: 23 jan 2016
 *      Author: Kajsa
 */


#include "PI.h"
int c;

int PI::calculateDC (float error){
	int a = Kp1 + 0.5*Ts*Ki1 + Kd1/Ts;
	int b = -Kp1 +  0.5*Ki1*Ts - 2*Kd1/Ts;

	int u;
	u = a*error+b*error_1+u_1;
	error_1 = error;
	u_1 = u;
	return u ;
}

void PI::Change_Duty_Cycle(){
		float e;
		float new_Vel;
		new_Vel = get_Velocity();
		e = ref_Vel - new_Vel;

		int dc = calculateDC(e);
		if (dc > MAX_DUTY_CYCLE) {
			dc = MAX_DUTY_CYCLE;
		} else if (dc < -MAX_DUTY_CYCLE) {
			dc = -MAX_DUTY_CYCLE;
		}
		El->setMainMotorSpeed(&dc);
	}

	float PI::get_Velocity() {
		imuBuffer.get(imu);
		float velocity = (radToDeg(imu.wz));
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

		ref_Vel = radToDeg(ref_vel);
	}

	void PI::stopMotor() {
		int speed = 0;
		El->setMainMotorSpeed(&speed);
	}



