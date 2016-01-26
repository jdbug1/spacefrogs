/*
 * PID.h
 *
 *  Created on: 23 jan 2016
 *      Author: Kajsa
 */

#ifndef PI_H_
#define PI_H_

#define P2_closed 	-42.38843906
#define I2_closed 	-381.0334781
#define wl_closed	12
#define P2_deployed -42.38843906
#define I2_deployed -381.0334781
#define wl_deployed 11
#define P2_extended -42.38843906
#define I2_extended -381.0334781
#define wl_extended 7

#define closed 		0
#define deployed 	1
#define extended	2

#define Ts2 		10
#define Ibound2 	100

#include "rodos.h"
#include "hal.h"
#include "../basics.h"
#include "../Electrical.h"

class PI {
private:
	float e_1, ref_Vel, ang_temp;
	uint state : 2;
	ahrsPublish imu;
	Electrical* El;

	float get_Velocity();

public:
	int DC;
	PI(const char* name, Electrical* El);
	virtual ~PI();

	void set_Velocity(float ref_vel);
	/*Sets the value of ref_Vel*/

	void set_State(int state);

	float get_State();

	void Change_Duty_Cycle();
	/*Sends a calculated value to the electrical element main motor*/

};

#endif /* PI_H_ */
