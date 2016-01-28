/*
 * Electrical.h
 *
 *  Created on: 10.12.2015
 *      Author: JackVEnterprises
 */

#ifndef ELECTRICAL_H_
#define ELECTRICAL_H_

#include "rodos.h"
#include "hal.h"
#include "basics.h"
#include "Currentsensor.h"

/* lightsensor */
#define CONTROL_REGISTER	0x00
#define TIMING_REGISTER		0x01
#define DATA_0_LOW			0x0C
#define DATA_1_LOW			0x0E

/* Currentsensor */
#define BATTERY_CURRENT		0b1000000


class Electrical : public Thread, public SubscriberReceiver<tcStruct> {
public:
	Electrical(const char* name);

	void init();
	void run();
	void put(tcStruct &command);
	void handleTelecommand(tcStruct *tc);


	void setMainMotorSpeed(int *speed);
	void setDeployment1Speed(int *speed);
	void setDeployment2Speed(int *speed);
	void setKnife(int *status);
	void setMagnet(int *status);
	void readLightsensor(int16_t *channel_0, int16_t *channel_1);
	void setLightsensor(int *value);
	void deployRacks(int *status);

private:
	tcStruct current_tc;
	tmStructElectrical values;
	bool deploy_racks;
	bool racks, electromagnet, lightsensor, thermal_knife, solar_panels;
	int32_t lightsensor_value;
	float battery_voltage, battery_current, solar_panel_voltage, solar_panel_current;

};

#endif /* ELECTRICAL_H_ */
