/*
 * lightsensor.cpp
 *
 *  Created on: 07.12.2015
 *      Author: JackVEnterprises
 */

#include "lightsensor.h"

#define CONTROL_REGISTER	0x00
#define TIMING_REGISTER		0x01
#define DATA_0_LOW			0x0C
#define DATA_0_HIGH			0x0D
#define DATA_1_LOW			0x0E
#define DATA_1_HIGH			0x0F


uint8_t LIGHT_CONTROL_REGISTER[2] = {0x00,0x03};
uint8_t CHANNEL_0_LOW[1] = {DATA_0_LOW | 0xA0};
uint8_t CHANNEL_1_LOW[1] = {DATA_1_LOW | 0xA0};



lightsensor::lightsensor(const char* name) : Thread(name) {
	send_data = false;
}

lightsensor::~lightsensor() {
	// TODO Auto-generated destructor stub
}

void lightsensor::init() {
	HAL_I2C_2.init(400000);
	HAL_I2C_2.write(LIGHT_SLAVE,LIGHT_CONTROL_REGISTER,2);
}

void lightsensor::run() {
	int16_t channel_0, channel_1;
	suspendCallerUntil();
	while(1) {
		readRaw(&channel_0,&channel_1);
		light_topic.publish(channel_0);
		//PRINTF("Channel_0 %d Channel_1 %d\n",channel_0,channel_1);
		suspendCallerUntil(NOW()+2*MILLISECONDS);
	}
}

void lightsensor::readRaw(int16_t *channel_0, int16_t *channel_1) {
	uint8_t data[2];
	int retVal = HAL_I2C_2.writeRead(LIGHT_SLAVE,CHANNEL_0_LOW,1,data,2);
	*channel_0 = data[1] << 8 | data[0];
	retVal = HAL_I2C_2.writeRead(LIGHT_SLAVE,CHANNEL_1_LOW,1,data,2);
	*channel_1 = data[1] << 8 | data[0];
}
