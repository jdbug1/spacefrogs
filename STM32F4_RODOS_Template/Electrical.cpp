/*
 * Electrical.cpp
 *
 *  Created on: 10.12.2015
 *      Author: JackVEnterprises
 */

#include "Electrical.h"

HAL_GPIO HBRIDGE_EN(GPIO_066);

/* H-Bridge A - main engine */
HAL_GPIO HBRIDGE_A_INA(GPIO_036);
HAL_GPIO HBRIDGE_A_INB(GPIO_017);
HAL_PWM MAIN_ENGINE_A(PWM_IDX12);

/* H-Bridge B - deployment A */
HAL_GPIO HBRIDGE_B_INA(GPIO_016);
HAL_GPIO HBRIDGE_B_INB(GPIO_071);
HAL_PWM DEPLOYMENT_1_B(PWM_IDX13);

/* H-Bridge C - deployment B */
HAL_GPIO HBRIDGE_C_INA(GPIO_072);
HAL_GPIO HBRIDGE_C_INB(GPIO_074);
HAL_PWM DEPLOYMENT_2_C(PWM_IDX14);

/* H-Bridge D - thermal knife & electromagnet */
HAL_GPIO THERMAL_KNIFE(GPIO_076);
HAL_GPIO ELECTROMAGNET(GPIO_079);
HAL_GPIO HBDRIGE_D_PWM(GPIO_063);

/* Activate for external power --> TODO into main */
HAL_GPIO PE2(GPIO_066);

Electrical::Electrical(const char* name) : Thread(name) {
	magnet_active = knife_active = false;
}

Electrical::~Electrical() {

}

void Electrical::init() {

	PE2.init(true,1,1);
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
}

void Electrical::run() {

/*
	PRINTF("Testing Hbridge A\n");
	suspendCallerUntil(NOW()+1*SECONDS);
	MAIN_ENGINE_A.write(500);
	suspendCallerUntil(NOW()+5*SECONDS);
	MAIN_ENGINE_A.write(0);
	suspendCallerUntil(NOW()+1*SECONDS);
	PRINTF("Testing Deployment A\n");
	suspendCallerUntil(NOW()+1*SECONDS);
	DEPLOYMENT_1_B.write(500);
	suspendCallerUntil(NOW()+5*SECONDS);
	DEPLOYMENT_1_B.write(0);
	suspendCallerUntil(NOW()+1*SECONDS);
	PRINTF("Testing Deployment B\n");
	suspendCallerUntil(NOW()+1*SECONDS);
	DEPLOYMENT_2_C.write(500);
	suspendCallerUntil(NOW()+5*SECONDS);
	DEPLOYMENT_2_C.write(0);
	suspendCallerUntil(NOW()+1*SECONDS);

	PRINTF("Testing electromagnet\n");
	ELECTROMAGNET.setPins(1);
	suspendCallerUntil(NOW()+5*SECONDS);
	ELECTROMAGNET.setPins(0);
	suspendCallerUntil(NOW()+1*SECONDS);

	PRINTF("Testing thermal knife\n");
	THERMAL_KNIFE.setPins(1);
	suspendCallerUntil(NOW()+5*SECONDS);
	THERMAL_KNIFE.setPins(0);
	suspendCallerUntil(NOW()+1*SECONDS);
*/

}

void Electrical::setMainMotorSpeed(int *speed) {
	if (*speed > 0) {
		HBRIDGE_A_INA.setPins(1);
		HBRIDGE_A_INB.setPins(0);
	} else {
		HBRIDGE_A_INA.setPins(0);
		HBRIDGE_A_INB.setPins(1);
	}
	MAIN_ENGINE_A.write(abs(*speed * 10));
	PRINTF("Main engine duty cycle set to %d%%\n",*speed);}

void Electrical::setDeployment1Speed(int *speed) {
	if (*speed > 0) {
		HBRIDGE_B_INA.setPins(1);
		HBRIDGE_B_INB.setPins(0);
	} else {
		HBRIDGE_B_INA.setPins(0);
		HBRIDGE_B_INB.setPins(1);
	}
	DEPLOYMENT_1_B.write(abs(*speed * 10));
	PRINTF("Deployment 1 duty cycle set to %d%%\n",*speed);
}

void Electrical::setDeployment2Speed(int *speed) {
	if (*speed > 0) {
		HBRIDGE_C_INA.setPins(1);
		HBRIDGE_C_INB.setPins(0);
	} else {
		HBRIDGE_C_INA.setPins(0);
		HBRIDGE_C_INB.setPins(1);
	}
	DEPLOYMENT_2_C.write(abs(*speed * 10));
	PRINTF("Deployment 2 duty cycle set to %d%%\n",*speed);
}

void Electrical::setKnife(int *status) {
	if (*status == 1) {
		PRINTF("ON\n");
		THERMAL_KNIFE.setPins(1);
	} else {
		PRINTF("OFF\n");
		THERMAL_KNIFE.setPins(0);
	}
	PRINTF("Thermal Knife %s %d\n",(*status == 1)?"activated":"deactivated",*status);
}

void Electrical::setMagnet(int *status) {
	if (*status == 1) {
		PRINTF("ON\n");
		ELECTROMAGNET.setPins(1);
	} else {
		PRINTF("OFF\n");
		ELECTROMAGNET.setPins(0);
	}
	PRINTF("Electromagnet %s %d\n",(*status == 1)?"activated":"deactivated",*status);
}

