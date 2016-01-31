/*
 * Electrical.cpp
 *
 *  Combines all electrical parts provides functions for motors, thermal knife, electromagnet and lightsensor
 *  @author	Sven Jörissen
 *  @date	10.12.2015
 *
 */

#include "Electrical.h"


#define SolarVoltageADC_1		ADC_CH_001		//PA1
#define SolarCurrentADC_2		ADC_CH_002		//PA2
#define ADC_VOLTAGE_SCALE_FACTOR		(4.9/3735.0)	//raw to V
#define ADC_CURRENT_SCALE_FACTOR		(29.0/411.0)	//raw to mA

/* ADCs for solar panel voltage and current */
HAL_ADC Solar_Voltage(ADC_IDX1);
HAL_ADC Solar_Current(ADC_IDX1);

/* Power enable for Hbridges */
HAL_GPIO HBRIDGE_EN(GPIO_066);

/* H-Bridge A - main engine */
HAL_GPIO HBRIDGE_A_INA(GPIO_036);
HAL_GPIO HBRIDGE_A_INB(GPIO_017);
HAL_PWM MAIN_ENGINE_A(PWM_IDX12);

/* H-Bridge B - camera/magnet deployment */
HAL_GPIO HBRIDGE_B_INA(GPIO_016);
HAL_GPIO HBRIDGE_B_INB(GPIO_071);
HAL_PWM DEPLOYMENT_1_B(PWM_IDX13);

/* H-Bridge C - counterweight deployment */
HAL_GPIO HBRIDGE_C_INA(GPIO_072);
HAL_GPIO HBRIDGE_C_INB(GPIO_074);
HAL_PWM DEPLOYMENT_2_C(PWM_IDX14);

/* H-Bridge D - thermal knife & electromagnet */
HAL_GPIO THERMAL_KNIFE(GPIO_076);	//D-A
HAL_GPIO ELECTROMAGNET(GPIO_079);	//D-B
HAL_GPIO HBDRIGE_D_PWM(GPIO_063);

/* Lightsensor registers and adresses */
uint8_t LIGHT_CONTROL_REGISTER[2] = {0x00,0x03};
uint8_t CHANNEL_0_LOW[1] = {DATA_0_LOW | 0xA0};
uint8_t CHANNEL_1_LOW[1] = {DATA_1_LOW | 0xA0};

/*
 * Constructor for a new electrical thread
 *
 * @param	const char* name	name of the object
 */
Electrical::Electrical(const char* name) : Thread(name), SubscriberReceiver<tcStruct>(tm_topic_incoming, "SubRec Electrical for Telecommands") {
	deploy_racks = false;

	thermal_knife = false;
	lightsensor = true;
	racks = false;
	solar_panels = false;
	electromagnet = false;
}

/*
 * Rodos init-function
 */
void Electrical::init() {
}

/*
 * Run function with main while loop
 * Default sampling rate is 50ms --> change in basics.h
 *
 * @publishes	status values of devices and data from lightsensor
 *
 */
void Electrical::run() {
	//Initialize HAL_ADCs
	Solar_Voltage.init(SolarVoltageADC_1);
	Solar_Current.init(SolarCurrentADC_2);

	//Generate new currentsensor object
	Currentsensor battery_current(BATTERY_CURRENT);
	battery_current.begin(BATTERY_CURRENT);

	/* init stuff */
	HBRIDGE_EN.init(true, 1, 1);

	THERMAL_KNIFE.init(true, 1, 0);
	ELECTROMAGNET.init(true, 1, 0);
	HBDRIGE_D_PWM.init(true, 1, 1);

	MAIN_ENGINE_A.init(10000,1000);
	HBRIDGE_A_INA.init(true, 1, 1);
	HBRIDGE_A_INB.init(true, 1, 0);

	DEPLOYMENT_1_B.init(1000,1000);
	HBRIDGE_B_INA.init(true, 1, 1);
	HBRIDGE_B_INB.init(true, 1, 0);

	DEPLOYMENT_2_C.init(1000,1000);
	HBRIDGE_C_INA.init(true, 1, 1);
	HBRIDGE_C_INB.init(true, 1, 0);

	int retVal = HAL_I2C_2.write(LIGHT_SLAVE,LIGHT_CONTROL_REGISTER,2);

	uint16_t channel_0, channel_1;
	int64_t t1 = 0;
	int64_t t2 = 0;
	float diff = 0;
	tmStructLight light;
	while (1) {
		if (deploy_racks) {
			if (current_tc.value == 0) {
				deployRacks(&current_tc.value);
			} else {
				if (t1 == 0) {
					t1 = NOW();
					deployRacks(&current_tc.value);
				}
				t2 = NOW();
				diff = (t2-t1)/1000000000.0;
				if (diff >= 10) {
					if (current_tc.value == 1) 	racks = true;
					else racks = false;
					current_tc.value = 0;
					deployRacks(&current_tc.value);
					deploy_racks = false;
					t1 = t2 = 0;
					diff = 0;
				}
			}

		}
		if (lightsensor) {
			readLightsensor(&channel_0,&channel_1);
			light.light_value = channel_0;
		}
		light_topic.publish(light);
		readADCCurrent();
		readADCVoltage();

		values.light_status = lightsensor;
		values.electromagnet = electromagnet;
		values.thermal_knife = thermal_knife;
		values.racks = racks;
		values.solar_panels = solar_panels;
		values.battery_current = battery_current.getCurrent_mA();
		values.battery_voltage = battery_current.getBusVoltage_V();
		values.solar_panel_current = solar_panel_current;
		values.solar_panel_voltage = solar_panel_voltage;
		electrical_topic.publish(values);
		suspendCallerUntil(NOW()+ELECTRICAL_SAMPLING_RATE*MILLISECONDS);
	}
}

void Electrical::put(tcStruct &command) {
	if (command.id == 3) {
		PRINTF("Received message\n");
		current_tc = command;
		this->handleTelecommand(&current_tc);

	}
}

void Electrical::handleTelecommand(tcStruct * tc) {
	int command = tc->command;
	PRINTF("Command was %d\n",command);
	switch(command) {
	case 3001:
		deploy_racks = true;
		break;
	case 3002:
		setMagnet(&current_tc.value);
		break;
	case 3003:
		setKnife(&current_tc.value);
		break;
	case 3004:
		setMainMotorSpeed(&current_tc.value);
		break;
	}
}

/*
 * Sets duty cycle for the big actuator
 *
 * @param int *speed	duty cycle from -100 to +100
 */
void Electrical::setMainMotorSpeed(int *speed) {
	if (*speed > 0) {
		HBRIDGE_A_INA.setPins(1);
		HBRIDGE_A_INB.setPins(0);
	} else {
		HBRIDGE_A_INA.setPins(0);
		HBRIDGE_A_INB.setPins(1);
	}
	MAIN_ENGINE_A.write(abs(*speed *10));
//	PRINTF("Main engine duty cycle set to %d%%\n",*speed);
}

/*
 * Sets duty cycle for camera/magnet rack actuator
 *
 * @param int *speed	duty cycle from -100 to +100
 */
void Electrical::setDeployment1Speed(int *speed) {
	if (*speed > 0) {
		HBRIDGE_B_INA.setPins(1);
		HBRIDGE_B_INB.setPins(0);
	} else {
		HBRIDGE_B_INA.setPins(0);
		HBRIDGE_B_INB.setPins(1);
	}
	DEPLOYMENT_1_B.write(abs(*speed * 10));
//	PRINTF("Deployment 1 duty cycle set to %d%%\n",*speed);
}

/*
 * Sets duty cycle for counterweight rack actuator
 *
 * @param int *speed	duty cycle from -100 to +100
 */
void Electrical::setDeployment2Speed(int *speed) {
	if (*speed > 0) {
		HBRIDGE_C_INA.setPins(1);
		HBRIDGE_C_INB.setPins(0);
	} else {
		HBRIDGE_C_INA.setPins(0);
		HBRIDGE_C_INB.setPins(1);
	}
	DEPLOYMENT_2_C.write(abs(*speed * 10));
//	PRINTF("Deployment 2 duty cycle set to %d%%\n",*speed);
}

/*
 * Turns thermal knife on and off
 *
 * @param TODO bool *status 	Desired status of thermal knife
 */
void Electrical::setKnife(int *status) {
	if (*status == 1) {
		PRINTF("ON\n");
		THERMAL_KNIFE.setPins(1);
		thermal_knife = true;
	} else {
		PRINTF("OFF\n");
		THERMAL_KNIFE.setPins(0);
		thermal_knife = false;
	}
//	PRINTF("Thermal Knife %s %d\n",(*status == 1)?"activated":"deactivated",*status);
}

/*
 * Turns electromagnet on and off
 *
 * @param TODO bool *status 	Desired status of electromagnet
 */
void Electrical::setMagnet(int *status) {
	if (*status == 1) {
		PRINTF("ON\n");
		ELECTROMAGNET.setPins(1);
		electromagnet = true;
	} else {
		PRINTF("OFF\n");
		ELECTROMAGNET.setPins(0);
		electromagnet = false;
	}
	/**
	 *
	 * pick up washer, remove it and start again...still lot of stuff to do, but you know, time, resources, ...
	 *
	 */
//	PRINTF("Electromagnet %s %d\n",(*status == 1)?"activated":"deactivated",*status);
}

/*
 * Reads data from lightsensor
 *
 * TODO lux implementation!
 */
void Electrical::readLightsensor(uint16_t *channel_0, uint16_t *channel_1) {
	uint8_t data[2];
	int retVal = HAL_I2C_2.writeRead(LIGHT_SLAVE,CHANNEL_0_LOW,1,data,2);
	*channel_0 = 256*data[1] | data[0];

	retVal = HAL_I2C_2.writeRead(LIGHT_SLAVE,CHANNEL_1_LOW,1,data,2);
	*channel_1 = 256*data[1] | data[0];
}

/*
 * Activates and deactivates reading from lightsensor
 *
 * @param int *status 	Desired status of lightsensor
 */
void Electrical::setLightsensor(int *value) {
	if (*value) {
		lightsensor = true;
		PRINTF("Lightsensor enabled\n");
	} else {
		lightsensor = false;
		PRINTF("Lightsensor disabled\n");
	}
}

/*
 * Handles rack deployment
 *
 * @param int *status 	1 --> deploy, -1 --> pull in, 0 --> stop!
 */
void Electrical::deployRacks(int *status) {
	int speed1 = 18;
	int speed2 = 20;
	if (*status == 1) {
		setDeployment1Speed(&speed1);
		setDeployment2Speed(&speed2);
	} else if (*status == 0) {
		speed1 = speed2 = *status;
		setDeployment1Speed(&speed1);
		setDeployment2Speed(&speed2);
		deploy_racks = false;
	} else if (*status == -1) {
		speed1 = -18;
		speed2 = -20;
		setDeployment1Speed(&speed1);
		setDeployment2Speed(&speed2);
	}
}

/*
 * Reads solar panel voltage from ADC
 */
void Electrical::readADCVoltage() {
	solar_panel_voltage = ((float)Solar_Voltage.read(SolarVoltageADC_1)*ADC_VOLTAGE_SCALE_FACTOR);
}

/*
 * Reads solar panel current from ADC
 */
void Electrical::readADCCurrent() {
	solar_panel_current = ((float)Solar_Voltage.read(SolarCurrentADC_2)*ADC_CURRENT_SCALE_FACTOR);
}

/*
 * Setter to control racks from outside
 */
void Electrical::setRacks(int *status) {
	deploy_racks = (bool)*status;
	current_tc.value = 1;
}
