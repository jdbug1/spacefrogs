/*
 * pwm.cpp
 *
 *  Created on: 03.12.2015
 *      Author: JackVEnterprises
 */

#include "motor.h"

HAL_GPIO HBRIDGE_A_INA(GPIO_036);
HAL_GPIO HBRIDGE_A_INB(GPIO_017);
HAL_GPIO HBRIDGE_EN(GPIO_066);
HAL_PWM Motor(PWM_IDX12);
HAL_GPIO RedLED(LED_RED);

/* Activate for external power --> TODO into main */
HAL_GPIO pe2(GPIO_066);


MOTOR::MOTOR(const char* name) : Thread(name) {

}

MOTOR::~MOTOR() {
	// TODO Auto-generated destructor stub
}

void MOTOR::init() {
	Motor.init(1000,1000);
	HBRIDGE_EN.init(true, 1, 1);
	HBRIDGE_A_INA.init(true, 1, 1);
	HBRIDGE_A_INB.init(true, 1, 0);
	RedLED.init(true, 1, 0);
	pe2.init(true,1,1);
}

void MOTOR::run() {
	PRINTF("Motor test programm\nStarting at 0%%, going to 100%% in 1%%-steps every second\n");
	for (int i = 0; i < 101; i++) {
		Motor.write((unsigned int)(i*10));
		PRINTF("Motor running at %d\n",i);
		suspendCallerUntil(NOW()+1*SECONDS);
	}
	Motor.write(750);
	suspendCallerUntil(NOW()+1*SECONDS);
	Motor.write(300);
	suspendCallerUntil(NOW()+1*SECONDS);
	Motor.write(0);
	suspendCallerUntil(NOW()+1*SECONDS);
	PRINTF("Other way round!\n");
	HBRIDGE_A_INA.setPins(~HBRIDGE_A_INA.readPins());
	HBRIDGE_A_INB.setPins(~HBRIDGE_A_INB.readPins());
	for (int i = 0; i < 101; i++) {
		Motor.write((unsigned int)(i*10));
		PRINTF("Motor running at -%d%%\n",i);
		suspendCallerUntil(NOW()+1*SECONDS);
	}

}

