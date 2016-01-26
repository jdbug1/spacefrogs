/*
 * PI.cpp
 *
 *  Created on: 23 jan 2016
 *      Author: Kajsa
 */

#include "PI.h"

	void PI::Change_Duty_Cycle(){
		float iMax =  Ibound2;
		float iMin = -Ibound2;
		float e;
		float P_term;
		float I_term;
		float new_Vel;
		float P, I;

		if (state == closed){
			P = P2_closed;
			I = I2_closed;
		} else if (state == deployed){
			P = P2_deployed;
			I = I2_deployed;
		} else if (state == extended){
			P = P2_extended;
			I = I2_extended;
		}

		new_Vel = get_Velocity();
		e = ref_Vel - new_Vel;

		float a = P + I*Ts2/2;
		float b = -P + I*Ts2/2;

		DC = DC + a*e + b*e_1;

		e_1 = e;

		PRINTF("DC is %d\n", DC);

		El->setMainMotorSpeed(&DC);
	}

	float PI::get_Velocity() {
		ahrsBuffer.get(imu);
		float ang = imu.heading;
		float velocity = (ang - ang_temp)/Ts2;
		ang_temp = ang;
		return velocity;
	}

	PI::PI(const char* name, Electrical* El){
		e_1 = 0;
		ref_Vel = 0;
		state = closed;
		ang_temp = imu.heading;
		DC = 0;
		this->El = El;
	}

	PI::~PI() {

	}

	void PI::set_Velocity(float ref_vel){
		ref_vel = ref_Vel;
	}

	float PI::get_State() {
		return this->state;
	}

	void PI::set_State(int state){
		this->state = state;
	}

