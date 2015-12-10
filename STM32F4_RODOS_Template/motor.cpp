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

//	while(1) {
//		suspendCallerUntil();
		int duty_cycle = 200;		//duty_cycle in %*10, e.g. 200 means 20% duty cycle
		Motor.write(duty_cycle);
		PRINTF("Motor running at %f\n",duty_cycle*0.1);
//	}
}

