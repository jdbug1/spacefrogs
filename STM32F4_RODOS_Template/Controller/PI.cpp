/*
 * PI.cpp
 *
 *  Created on: 23 jan 2016
 *      Author: Kajsa
 */


#include "PI.h"
int c;
void PI::Change_Duty_Cycle(){
		float e;
		float new_Vel;
		c++;
		new_Vel = get_Velocity();
		e = ref_Vel - new_Vel;

		e_sum += e;
//		PRINTF("E: %d ESum: %d\n ",e*180.0/M_PI,e_sum);
		if (e_sum > 100.0){
			e_sum = 100.0;
		} else if (e_sum < -100.0){
			e_sum = -100.0;
		} else if ((e < 0.3) && (e > -0.3)) {
			e_sum = 0.0;
			e = 0.0;
		}

		float P_term = Pi*e;
		float I_term = Ii*e_sum;

		DC = DC - P_term - I_term;

		if (DC > 100){
			DC = 100;
		} else if (DC < -100) {
			DC = -100;
		}
		if (c == 15) {
			PRINTF("Ref Vel %3.2f Vel %3.2f E %3.2f ESum %5.2f",ref_Vel, new_Vel, e, e_sum);
			c = 0;
			PRINTF("P %5.2f I %5.2f\n",P_term,I_term);
		}

		DC = DC - (int)(P_term - I_term);

		if (DC > 100){
			DC = 100;
		} else if (DC < -100) {
			DC = -100;
		}

//		PRINTF("DC is %d\n", DC);

		El->setMainMotorSpeed(&DC);
	}

	float PI::get_Velocity() {
		imuBuffer.get(imu);
		float velocity = (imu.wz*180.0/M_PI);
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

		ref_Vel = ref_vel*180.0/M_PI;
	}



