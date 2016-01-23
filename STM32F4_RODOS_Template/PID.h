/*
 * PID.h
 *
 *  Created on: 20 jan 2016
 *      Author: Kajsa
 */

#ifndef PID_H_
#define PID_H_

#define P1_closed	0.67843361450436
#define I1_closed	0.00367323802974744
#define D1_closed	-0.518588367213643
#define N_closed	1.29969476217057
#define P1_deployed 20
#define I1_deployed 0.2
#define D1_deployed 5
#define N_deployed	2
#define P1_extended 20
#define I1_extended 0.2
#define D1_extended	5
#define N_extended	2

#define Ts1 Ts2
#define Ibound1 100

#include "rodos.h"
#include "hal.h"
#include "basics.h"
#include "PI.h"

class PID {
private:
	float i_temp, d_temp, PWM_temp, ref_Ang;
	imuPublish imu;
	PI* pi;
	void Change_Ref_Vel();
	/*Calculates a reference velocity and sets it in the PI controller*/

	float get_Angle();

public:
	PID(const char* name, PI* pi);
	virtual ~PID();
};

#endif /* PID_H_ */
