/*
 * PID.h
 *
 *  Created on: 20 jan 2016
 *      Author: Kajsa
 */

#ifndef PID_H_
#define PID_H_

#define Kp -600
#define Ki -80
#define Kd 0

#define Po 1.0
#define Io 0.001
#define Do 0.3
#define speed_lim 30

#include "rodos.h"
#include "hal.h"
#include "../basics.h"
#include "PI.h"
#include "../Electrical.h"

class PID {
private:
	float e_sum, e_old, ref_Ang;
	imuData imu;
	Electrical* el;

	int u_1;
	float error_1;

	float get_Angle();

public:
	PID(const char* name, Electrical* El);
	virtual ~PID();

	void Change_Ref_Vel();
	/*Calculates a reference velocity and sets it in the PI controller*/

	void Set_Ref_Angle(float ref_angle);

	int calculateDC(float error);
};

#endif /* PID_H_ */
