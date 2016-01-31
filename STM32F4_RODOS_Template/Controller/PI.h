/*
 * PID.h
 *
 *  Created on: 23 jan 2016
 *      Author: Kajsa
 */

#ifndef PI_H_
#define PI_H_

#define Kp1 (-54.0)
#define Ki1 (-125.0)
#define Kd1 0

#define MAX_DUTY_CYCLE	50

#define Tms 		20
#define Ts			Tms/1000

#include "rodos.h"
#include "hal.h"
#include "../basics.h"
#include "../Electrical.h"

class PI {
private:
	float e_sum, ref_Vel, ang_temp, P_term, I_term;
	imuData imu;
	Electrical* El;
	int u_1;
	float error_1;

	float get_Velocity();

public:
	int DC;
	PI(const char* name, Electrical* El);
	virtual ~PI();

	void set_Velocity(float ref_vel);
	/*Sets the value of ref_Vel*/

	void Change_Duty_Cycle();
	/*Sends a calculated value to the electrical element main motor*/

	int calculateDC (float error);

	void stopMotor();

};

#endif /* PI_H_ */
