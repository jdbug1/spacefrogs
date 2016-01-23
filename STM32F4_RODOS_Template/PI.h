/*
 * PID.h
 *
 *  Created on: 23 jan 2016
 *      Author: Kajsa
 */

#ifndef PI_H_
#define PI_H_

#define P2_closed 	20
#define I2_closed 	0.2
#define P2_deployed 20
#define I2_deployed 0.2
#define P2_extended 20
#define I2_extended 0.2

#define closed 		0
#define deployed 	1
#define extended	2

#define Ts2 		0.01
#define Ibound2 	100

#include "rodos.h"
#include "hal.h"
#include "basics.h"
#include "Electrical.h"

class PI : public Thread {
private:
	float i_temp, PWM_temp, ref_Vel, state;
	imuPublish imu;
	Electrical* electrical;
	void Change_Duty_Cycle();
	/*Sends a calculated value to the electrical element main motor*/

	float get_Velocity();

public:
	PI(const char* name, Electrical* El);
	virtual ~PI();

	void init();
	/*Sets the variables to initial values*/

	void run();
	/*Checks the reference velocity and tries to achieve it*/

	void set_Velocity(float ref_vel);
	/*Sets the value of ref_Vel*/

	void set_State(int i);

};

#endif /* PI_H_ */
