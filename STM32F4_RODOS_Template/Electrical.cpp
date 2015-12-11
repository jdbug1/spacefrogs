/*
 * Electrical.cpp
 *
 *  Created on: 10.12.2015
 *      Author: JackVEnterprises
 */

#include "Electrical.h"

HAL_GPIO HBRIDGE_D_IN_A(GPIO_076);
HAL_GPIO HBRIDGE_D_IN_B(GPIO_079);
HAL_GPIO HBDRIGE_D_PWM(GPIO_063);

Electrical::Electrical(const char* name) : Thread(name) {
	// TODO Auto-generated constructor stub

}

Electrical::~Electrical() {
	// TODO Auto-generated destructor stub
}

void Electrical::init() {
	HBRIDGE_D_IN_A.init(true,1,1);
	HBDRIGE_D_PWM.init(true,1,1);
}

void Electrical::run() {
	PRINTF("Hbridge-D high\n");
	HBRIDGE_D_IN_A.init(true,1,1);
	HBDRIGE_D_PWM.init(true,1,1);
	suspendCallerUntil(NOW()+5*SECONDS);
	PRINTF("Hbridge-D low\n");
	HBRIDGE_D_IN_A.init(true,1,0);
	HBDRIGE_D_PWM.init(true,1,0);


}

