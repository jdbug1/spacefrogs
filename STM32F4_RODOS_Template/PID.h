/*
 * PID.h
 *
 *  Created on: 20 jan 2016
 *      Author: Kajsa
 */

#ifndef PID_H_
#define PID_H_

#define P1 20
#define I1 0.2
#define D1 0.2
#define Ts1 0.01
#define Ibound1 100

#include "rodos.h"
#include "hal.h"
#include "basics.h"
#include "PI.h"

class PID : public Thread {
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

	void init();
	void run();

};

#endif /* PID_H_ */
