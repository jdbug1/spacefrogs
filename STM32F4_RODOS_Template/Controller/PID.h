/*
 * PID.h
 *
 *  Created on: 20 jan 2016
 *      Author: Kajsa
 */

#ifndef PID_H_
#define PID_H_

#define Po 1
#define Io 1
#define Do 1
#define speed_lim 80

#include "rodos.h"
#include "hal.h"
#include "../basics.h"
#include "PI.h"

class PID {
private:
	float e_sum, e_old, ref_Ang;
	tmStructIMU imu;
	PI* pi;

	float get_Angle();

public:
	PID(const char* name, PI* pi);
	virtual ~PID();

	void Change_Ref_Vel();
	/*Calculates a reference velocity and sets it in the PI controller*/

	void Set_Ref_Angle(float ref_angle);
};

#endif /* PID_H_ */
