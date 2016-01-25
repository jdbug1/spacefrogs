/*
 * basics.h
 *
 *  @author	Sven Jörissen
 *  @date	21.11.2015
 *
 */

#ifndef BASICS_H_
#define BASICS_H_

#include "rodos.h"
#include "math.h"
#include <stdlib.h>


/* adresses */
#define LSM9DS0_G		0x6B
#define LSM9DS0_XM		0x1D
#define LIGHT_SLAVE		0x39
#define LSM303DLM_XM	0x1E

/* GPIOs */
#define LED_GREEN GPIO_060
#define LED_ORANGE GPIO_061
#define LED_RED GPIO_062
#define LED_BLUE GPIO_063

/* important IMU values */
#define ACC_SENSITIVITY_2		0.061	// [mg/LSB]
#define GYRO_SENSITIVITY_2000	0.07	// [dps/digit]
#define GYRO_SENSITIVITY_500	0.0175	// [dps/digit]
#define GYRO_SENSITIVITY_245	0.00875	// [dps/digit]
#define CALIBRATION_VALUES		1024	// for sensor calibration
#define MEAN_FILTER_SAMPLES		4		// simple low pass filter

/* important AHRS values */
#define ALPHA 0.9					// gain of complementary filter

/* I2Cs */
extern HAL_I2C HAL_I2C_1;
extern HAL_I2C HAL_I2C_2;

/* Sampling rates for all threads in [ms] */
#define IMU_SAMPLING_RATE			2
#define ELECTRICAL_SAMPLING_RATE	100
#define TELEMETRY_SAMPLING_RATE		250

/* important structs */

struct xyz32 {
	int32_t x;
	int32_t y;
	int32_t z;
};

struct xyz16 {
	int16_t x;
	int16_t y;
	int16_t z;
};

struct xyzFloat {
	float x;
	float y;
	float z;
};

struct RPY {
	float roll;
	float pitch;
	float heading;
};

struct imuData {
		float ax;
		float ay;
		float az;
		float mx;
		float my;
		float mz;
		float wx;
		float wy;
		float wz;
		bool calibrating;
};

struct ahrsPublish {
	float wx;
	float roll;
	float pitch;
	float heading;
	float xm_heading;
	float gyro_heading;
};

struct magMaxMin {
	int16_t xMax;
	int16_t xMin;
	int16_t yMax;
	int16_t yMin;
	int16_t zMax;
	int16_t zMin;
};

struct Quat {
	float q0;
	float q1;
	float q2;
	float q3;
};

struct electricalStruct {
	float light;
	bool light_status;
	bool em_status;
	bool knife_status;
	float battery_voltage;
	float bus_current;
};

struct tmStructIMU {
	float ax,ay,az;
	float mx,my,mz;
	float wx,wy,wz;
	float roll, pitch, heading;
};

struct tmStructElectrical {
	bool light, em, knife;
	int16_t light_value;
};

/* useful functions */

static inline double degToRad(double deg) {
	return deg*M_PI/180.0;
}

static inline double radToDeg(double rad) {
	return rad*180.0/M_PI;
}

/*
 * Compares two char pointers
 * @param	*cx pointer to char x
 * @return	0 if not equal, 1 if equal
 */
static inline char compare(char *c1, char *c2) {
	int i;
	for (i = 0; i < strlen(c2); i++ ) {
		if (c1[i] != c2[i]) {
			return 0;
		}
	}
	return 1;
}

/*
 * Looks for maximum in struct xyz32
 * @return	max element of given struct
 */
static inline double myMax(struct xyz32 temp) {
	int t;

	if (temp.x >= temp.y) {
		t = temp.x;
	} else {
		t = temp.y;
	}

	if (t >= temp.z) {
		return t;
	} else {
		return temp.z;
	}

	return 0;
}

/*
 * Used for calibration of accelerometer
 * @param	xyz32 *axis	pointer to xyz32 struct
 * @return	char for axis with highes values, 0 if error
 */
static inline char findAxis(struct xyz32 *axis) {
	xyz32 temp;
	double max;

	temp.x = abs(axis->x);
	temp.y = abs(axis->y);
	temp.z = abs(axis->z);
	max = myMax(temp);

	if (max == temp.x) return 'x';
	if (max == temp.y) return 'y';
	if (max == temp.z) return 'z';
	return '0';
}

/* Topics with buffers and subscribers */

/*! internal - communication between threads */
extern Topic<imuData> imu_topic;
static CommBuffer<imuData> imuBuffer;
static Subscriber imuSubscriber(imu_topic, imuBuffer, "IMU Subscriber");

extern Topic<ahrsPublish> ahrs_topic;
static CommBuffer<ahrsPublish> ahrsBuffer;
static Subscriber ahrsSubscriber(ahrs_topic, ahrsBuffer, "AHRS Subscriber");

extern Topic<electricalStruct> electrical_topic;
static CommBuffer<electricalStruct> electricalBuffer;
static Subscriber lightSubscriber(electrical_topic,electricalBuffer, "Electrical Subscriber");

/*! external - telemetry*/
extern Topic<tmStructIMU> tm_topic_imu;
static CommBuffer<tmStructIMU> tmIMUBuffer;
static Subscriber tmIMUSubscriber(tm_topic_imu, tmIMUBuffer, "Telemetry IMU Subscriber");

extern Topic<tmStructElectrical> tm_topic_electrical;
static CommBuffer<tmStructElectrical> tmElectricalBuffer;
static Subscriber tmElectricalSubscriber(tm_topic_electrical, tmElectricalBuffer, "Telemetry IMU Subscriber");

extern Topic<char*> tm_topic_incoming;


#endif /* BASICS_H_ */
