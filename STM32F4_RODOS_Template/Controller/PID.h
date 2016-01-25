/*
 * PID.h
 *
 *  Created on: 20 jan 2016
 *      Author: Kajsa
 */

#ifndef PID_H_
#define PID_H_

#define P1_closed	0.420934675
#define I1_closed	0.001291925
#define D1_closed	-0.704908233
#define N_closed	0.495087645
#define P1_deployed 0.420934675
#define I1_deployed 0.001291925
#define D1_deployed -0.704908233
#define N_deployed	0.495087645
#define P1_extended 0.420934675
#define I1_extended 0.001291925
#define D1_extended	-0.704908233
#define N_extended	0.495087645

#define Ts1 Ts2
#define Ibound1 100

#include "rodos.h"
#include "hal.h"
#include "../basics.h"
#include "PI.h"

class PID {
private:
	float i_temp, d_temp, PWM_temp, ref_Ang;
	ahrsPublish imu;
	PI* pi;

	float get_Angle();

public:
	PID(const char* name, PI* pi);
	virtual ~PID();

	void Change_Ref_Vel();
	/*Calculates a reference velocity and sets it in the PI controller*/
};

#endif /* PID_H_ */
