/*
 * PID.h
 *
 *  Created on: 23 jan 2016
 *      Author: Kajsa
 */

#ifndef PI_H_
#define PI_H_

#define Pi (-1.0)
#define Ii (-0.01)

#define Tms 		10
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

	float get_Velocity();

public:
	int DC;
	PI(const char* name, Electrical* El);
	virtual ~PI();

	void set_Velocity(float ref_vel);
	/*Sets the value of ref_Vel*/

	void Change_Duty_Cycle();
	/*Sends a calculated value to the electrical element main motor*/

};


/*
#define Pi -1.0
#define Ii -0.01

#define Tms 		10
#define Ts			Tms/1000

#include "rodos.h"
#include "hal.h"
#include "../basics.h"
#include "../Electrical.h"

class PI {
private:
	float e_sum, ref_Vel, ang_temp, P_term, I_term;
	tmStructIMU imu;
	Electrical* El;

	float get_Velocity();

public:
	int DC;
	PI(const char* name, Electrical* El);
	virtual ~PI();

	void set_Velocity(float ref_vel);
	/*Sets the value of ref_Vel*/
/*
	void Change_Duty_Cycle();
	/*Sends a calculated value to the electrical element main motor*/
/*
};
*/
#endif /* PI_H_ */
