/*
 * PID.h
 *
 *  Created on: 23 jan 2016
 *      Author: Kajsa
 */

#ifndef PI_H_
#define PI_H_

#define P2_closed 	-40.2426953452767
#define I2_closed 	-371.710687698176
#define wl_closed	12
#define P2_deployed 20
#define I2_deployed 0.2
#define wl_deployed 7
#define P2_extended 20
#define I2_extended 0.2
#define wl_extended 7

#define closed 		0
#define deployed 	1
#define extended	2

#define Ts2 		0.01
#define Ibound2 	100

#include "rodos.h"
#include "hal.h"
#include "../basics.h"
#include "../Electrical.h"

class PI {
private:
	float i_temp, PWM_temp, ref_Vel, state;
	ahrsPublish imu;
	Electrical* El;

	float get_Velocity();

public:
	PI(const char* name, Electrical* El);
	virtual ~PI();

	void set_Velocity(float ref_vel);
	/*Sets the value of ref_Vel*/

	void set_State(int i);

	float get_State();

	void Change_Duty_Cycle();
	/*Sends a calculated value to the electrical element main motor*/

};

#endif /* PI_H_ */
