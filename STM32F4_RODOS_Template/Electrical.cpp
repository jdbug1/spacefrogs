/*
 * Electrical.cpp
 *
 *  Combines all electrical parts provides functions for motors, thermal knife, electromagnet and lightsensor
 *  @author	Sven J�rissen
 *  @date	10.12.2015
 *
 */

#include "Electrical.h"


#define SolarVoltageADC_1		ADC_CH_001	//PA1
#define SolarCurrentADC_2		ADC_CH_002	//PA2
#define ADC_1_SCALE_FACTOR	1458.0

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
Electrical::Electrical(const char* name) : Thread(name) {
	read_lightsensor = false;
	knife = false;
	em = false;
}

/*
 * Rodos init-function
 */
void Electrical::init() {
}

/*
 * Run function with main while loop
 * Default sampling rate is 100ms --> change in basics.h
 *
 * @publishes	status values of devices and data from lightsensor
 *
 */
void Electrical::run() {
	//Initialize HAL_ADCs
	Solar_Voltage.init(SolarVoltageADC_1);
	Solar_Current.init(SolarCurrentADC_2);


//	HAL_I2C_2.init(400000);
	HAL_I2C_1.init(400000);

	//Generate new currentsensor object
	Currentsensor battery_current(BATTERY_CURRENT);
	battery_current.begin(BATTERY_CURRENT);

	/* init stuff */
	HBRIDGE_EN.init(true, 1, 1);

	THERMAL_KNIFE.init(true, 1, 0);
	ELECTROMAGNET.init(true, 1, 0);
	HBDRIGE_D_PWM.init(true, 1, 1);

	MAIN_ENGINE_A.init(1000,1000);
	HBRIDGE_A_INA.init(true, 1, 1);
	HBRIDGE_A_INB.init(true, 1, 0);

	DEPLOYMENT_1_B.init(1000,1000);
	HBRIDGE_B_INA.init(true, 1, 1);
	HBRIDGE_B_INB.init(true, 1, 0);

	DEPLOYMENT_2_C.init(1000,1000);
	HBRIDGE_C_INA.init(true, 1, 1);
	HBRIDGE_C_INB.init(true, 1, 0);

	int retVal = HAL_I2C_2.write(LIGHT_SLAVE,LIGHT_CONTROL_REGISTER,2);

	int16_t channel_0, channel_1;
	electricalStruct values;
	read_lightsensor = 0;
	while (1) {
		if (read_lightsensor) {
			readLightsensor(&channel_0,&channel_1);
		}
		values.light = (float)channel_0;
		values.light_status = read_lightsensor;
		values.knife_status = knife;
		values.em_status = em;
		values.battery_voltage = battery_current.getBusVoltage_V();
		values.bus_current = battery_current.getCurrent_mA();
		electrical_topic.publish(values);
//		PRINTF("Battery Current is %f Voltage is %f\n",battery_current.getCurrent_mA(), battery_current.getBusVoltage_V());
		suspendCallerUntil(NOW()+ELECTRICAL_SAMPLING_RATE*MILLISECONDS);
//		PRINTF("Solar_Voltage Voltage ADC =  %02.2f V\n",Solar_Voltage.read(SolarVoltageADC)/ADC_1_SCALE_FACTOR);
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
	MAIN_ENGINE_A.write(abs(*speed * 10));
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
		knife = 1;
	} else {
		PRINTF("OFF\n");
		THERMAL_KNIFE.setPins(0);
		knife = 0;
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
		em = 1;
	} else {
		PRINTF("OFF\n");
		ELECTROMAGNET.setPins(0);
		em = 0;
	}
//	PRINTF("Electromagnet %s %d\n",(*status == 1)?"activated":"deactivated",*status);
}

/*
 * Reads data from lightsensor
 *
 * TODO lux implementation!
 */
void Electrical::readLightsensor(int16_t *channel_0, int16_t *channel_1) {
	uint8_t data[2];
	int retVal = HAL_I2C_2.writeRead(LIGHT_SLAVE,CHANNEL_0_LOW,1,data,2);
//	PRINTF("Low: %u High: %u\n", data[0],data[1]);
	*channel_0 = data[1] << 8 | data[0];

	retVal = HAL_I2C_2.writeRead(LIGHT_SLAVE,CHANNEL_1_LOW,1,data,2);
	*channel_1 = data[1] << 8 | data[0];
}

/*
 * Activates and deactivates reading from lightsensor
 *
 * @param TODO bool *status 	Desired status of lightsensor
 */
void Electrical::setLightsensor(int *value) {
	if (*value) {
		read_lightsensor = true;
		PRINTF("Lightsensor enabled\n");
	} else {
		read_lightsensor = false;
		PRINTF("Lightsensor disabled\n");
	}
}

