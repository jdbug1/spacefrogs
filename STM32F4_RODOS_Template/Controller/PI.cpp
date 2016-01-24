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
		float Duty_Cycle;
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

		P_term = P*e;

		i_temp += e*Ts2;
		if (i_temp < iMin){
			i_temp = iMin;
		} else if (i_temp > iMax){
			i_temp = iMax;
		}
		I_term = I*i_temp;

		Duty_Cycle = PWM_temp - (P_term + I_term);

		if (Duty_Cycle < -100){
			Duty_Cycle = -100;
		} else if (Duty_Cycle > 100){
			Duty_Cycle = 100;
		}
		int DC = int(Duty_Cycle);

		El->setMainMotorSpeed(&DC);
	}

	float PI::get_Velocity() {
		ahrsBuffer.get(imu);
		float velocity = imu.wx;
		// Some filtering code
		return velocity;
	}

	PI::PI(const char* name, Electrical* El){
		i_temp = 0;
		PWM_temp = 0;
		ref_Vel = 0;
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

