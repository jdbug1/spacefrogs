/*
 * imu.cpp
 *
 *  Includes implementation of the IMU and the AHRS
 *  @author	Sven Jörissen
 *  @date	09.11.2015
 *
 */

#include "IMU.h"

/** IMU specific data LSM9DS0 */
/* Gyro control registers */
uint8_t CTRL_REG1_G[2] = {0x20,0b11111111}; // ODR=760Hz (11) + Cutoff=100Hz (11) + Normal Mode (1) + Gyro Axis Enabled (111)
uint8_t CTRL_REG4_G[2] = {0x23,0b10110000}; // BDU(1) + Data LSb @ lower address (0) + Full-scale = 2000 dps (11) + (0) + Self-test disabled (00) + 4-wire interface SPI (0)
/* XM control registers */
uint8_t CTRL_REG1_XM[2] = {0x20, 0b01111111}; //200Hz(0111) and BDU(1) and Axis Enabled (111)
uint8_t CTRL_REG2_XM[2] = {0x21, 0b01000000}; //773Hz Filter (10) and 2g Scale (000) and normal mode (00) and 4-wire SPI (0)
uint8_t CTRL_REG5_XM[2] = {0x24, 0b01110000}; //Temperatur enabled (1) and Low Magnetic Resolution (11) and 50Hz (100) and (00)
uint8_t CTRL_REG6_XM[2] = {0x25, 0b00000000}; //x00xxxxx 2gauss
uint8_t CTRL_REG7_XM[2] = {0x26, 0b10000000}; //Normal mode (00) + internal filter bypassed (0) + (00) + low-power (0) + power down (10)
/* Starting adresses for sensors */
uint8_t LSM9DS0_OUT_X_L_G[1] = {0x28 | 0x80}; // Gyroscope OUT_X_L address with multiple bytes indicator (0x80)
uint8_t LSM9DS0_OUT_X_L_A[1] = {0x28 | 0x80}; // Accelerometer OUT_X_L address with multiple bytes indicator (0x80)

/** IMU specific data LSM303DLH */
/* Magnetometer control registers */
uint8_t CRA_REG_M[2] = {0x00, 0b00011000};	//75Hz ODR
uint8_t CRB_REG_M[2] = {0x01, 0b01000000};	//1.9 Gauss
uint8_t MR_REG_M[2]	= {0x02,0b00000000};	//Continuous-conversion mode
/* Starting adresses for axes */
uint8_t LSM303DLM_OUT_X_H_M[1] = {0x03 | 0x80}; // Magnetometer out x high

HAL_GPIO CS_G(GPIO_018); 		/* declare HAL_GPIO for GPIO_018 = PB2 (IMU Chip Select pin for the Gyro) */
HAL_GPIO IMU_EN(GPIO_055); 		/* declare HAL_GPIO for GPIO_055 = PD7 (IMU Power Enable pin) */
HAL_GPIO USERBUTTON(GPIO_000); 	//UserButton

/*
 * The rotation matrix from the matlab script
 */
float el_ma[3][3] = {{	    -1.2143,    6.3892,    0.6054},
	    				{6.3892,    6.1326,   -1.8173},
	    				{0.6054,   -1.8173,   51.9001}};
//						3.9419,   -0.0282,   -0.8055},
//                     {	 -0.0282,    3.5682,    0.1643},
//                     {	 -0.8055,    0.1643,    5.2197}};

/*
 * The scale factors from the matlab script
 */
float el_sc[3] = {-120.5047,    9.4105,   37.7048};
//float el_sc[3] = {-157.3306,   -0.3943,   72.4050};


/*
 * Contructor for imu class
 *
 * @param	const char* name	name for the thread
 */
IMU::IMU(const char* name) : Thread (name) {

	/* default gyroscope offsets */
	g_offset.x = 60.4;
	g_offset.y = 28.9;
	g_offset.z = 14.6;
	/* default magnetometer offsets for hard iron calibration */
	m_offset.xMax = 189;
	m_offset.xMin = -533;
	m_offset.yMax = 389;
	m_offset.yMin = -278;
	m_offset.zMin = 101;
	m_offset.zMax = -559;
	/* default accelerometer offsets */
	a_offset.x = 1496;
	a_offset.y = -243;
	a_offset.z = 1047;

}

IMU::~IMU() {
}

/*
 * Rodos init function for imu class
 */
void IMU::init() {
	HAL_I2C_2.init(400000);
	IMU_EN.init(true,1,1);

	/* Userbutton for calibration */
	USERBUTTON.init(false,1,0);						//init Button as input
	USERBUTTON.config(GPIO_CFG_IRQ_SENSITIVITY,2);	//set sensitivity to falling edge (1)
	USERBUTTON.interruptEnable(true);

	initSensors();
}

/*
 * Initializes internal imu and external magnetometer
 */
void IMU::initSensors() {
	CS_G.init(true,1,1);
	/* initialize Gyro */
	HAL_I2C_2.write(LSM9DS0_G, CTRL_REG1_G, 2);
	HAL_I2C_2.write(LSM9DS0_G, CTRL_REG4_G, 2);

	/* initialize Acc and Mag */
	HAL_I2C_2.write(LSM9DS0_XM, CTRL_REG1_XM, 2);
	HAL_I2C_2.write(LSM9DS0_XM, CTRL_REG2_XM, 2);
	HAL_I2C_2.write(LSM9DS0_XM, CTRL_REG5_XM, 2);
	HAL_I2C_2.write(LSM9DS0_XM, CTRL_REG6_XM, 2);
	HAL_I2C_2.write(LSM9DS0_XM, CTRL_REG7_XM, 2);

	HAL_I2C_2.write(LSM303DLM_XM, CRA_REG_M, 2);
	HAL_I2C_2.write(LSM303DLM_XM, CRB_REG_M, 2);
	HAL_I2C_2.write(LSM303DLM_XM, MR_REG_M, 2);
}

/*
 * Run function with main while loop
 * Default sampling rate is 250ms --> change in basics.h
 * Reads data from all sensors and calculates the AHRS
 * If sensor needs to be calibrated, all other functions will be paused
 *
 * @publishes	imu and ahrs data
 */
void IMU::run() {
	int64_t t1,t2;
	int counter = 0;
	float temp_heading;
	TIME_LOOP(0,IMU_SAMPLING_RATE*MILLISECONDS) {
//		t1 = NOW();
		if (calibrate_magnetometer) {
			calibrateMag();
		} else if (calibrate_gyroscope) {
			calibrateGyro();
		} else if (calibrate_accelerometer) {
			calibrateAcc();
		} else {
			calculateGyro();
			calculateMag();
			calculateAcc();
			imu_topic.publish(imu_data);
			complementaryFilter();
/*			publish.heading = ahrs_euler.heading;
			publish.roll = ahrs_euler.roll;
			publish.pitch = ahrs_euler.pitch;
			publish.wx = imu_data.wx;
			publish.gyro_heading = gyro_euler.heading;
			publish.xm_heading = xm_euler.heading;
			ahrs_topic.publish(publish);
*/			if (counter <= MEAN_FILTER_SAMPLES - 1) {
				counter++;
				temp_heading += ahrs_euler.heading;
			} else if (counter == MEAN_FILTER_SAMPLES) {
				publish.heading = temp_heading/(float)MEAN_FILTER_SAMPLES;
				publish.roll = ahrs_euler.roll;
				publish.pitch = ahrs_euler.pitch;
				publish.wx = imu_data.wx;
				ahrs_topic.publish(publish);
				temp_heading = 0;
				counter = 0;
			}
		}

//		t2 = NOW();
//		if (counter >= 50) {
//			PRINTF("Time was %1.2f ms\n",(float)(t2-t1)/1000000.0);
//			counter = 0;
//		}
	}
}

/*
 * Resets the I2C2, if error is detected
 * Reinitialization of IMU is necessary
 */
void IMU::I2CError() {
	HAL_I2C_2.reset();
	HAL_I2C_2.init(400000);
	IMU_EN.setPins(0);
	suspendCallerUntil(NOW()+5*MILLISECONDS);
	IMU_EN.setPins(1);
	IMU_EN.init(true,1,1);
	initSensors();
}

/** IMU - functions to read and process data from imu */

/*
 * Reads raw data from gyroscope
 */
void IMU::readGyro() {
	uint8_t data[6] = {};
	int retVal = HAL_I2C_2.writeRead(LSM9DS0_G, LSM9DS0_OUT_X_L_G, 1, data, 6);
	HAL_I2C_2.suspendUntilReadFinished();
	if (retVal <= 0) {
		I2CError();
	} else {
		g_raw.x = (int16_t)((data[1] << 8) | data[0]);
		g_raw.y = (int16_t)((data[3] << 8) | data[2]);
		g_raw.z = (int16_t)((data[5] << 8) | data[4]);
//		PRINTF("Gyro %d %d %d\n",g_raw.x,g_raw.y,g_raw.z);
	}
}

/*
 * calculates angular rate [rad/s]
 */
void IMU::calculateGyro() {
	readGyro();
	imu_data.wx = degToRad((((float)g_raw.x)-g_offset.x)*GYRO_SENSITIVITY_2000);
	imu_data.wy = degToRad((((float)g_raw.y)-g_offset.y)*GYRO_SENSITIVITY_2000);
	imu_data.wz = degToRad((((float)g_raw.z)-g_offset.z)*GYRO_SENSITIVITY_2000);
}

/*
 * Calibration function for gyroscope
 * Takes X samples in standstill mode for each axis and averages them to get the zero offset
 */
void IMU::calibrateGyro() {
	imu_data.calibrating = true;
	imu_topic.publish(imu_data);
	PRINTF("Calibrating gyro...\n");
	int16_t tempX =0, tempY = 0, tempZ = 0;
	g_temp.x = g_temp.y = g_temp.z = 0;
	uint8_t data[6] = {};
	int failCounter = 0;
	int j = 0;
	for (int i = 0; i < CALIBRATION_VALUES; i++) {
		j = i;
		int retVal = HAL_I2C_2.writeRead(LSM9DS0_G, LSM9DS0_OUT_X_L_G, 1, data, 6);
		HAL_I2C_2.suspendUntilReadFinished();
		if (retVal <= 0) {
			I2CError();
			failCounter++;
		}
		else {
			tempX = (data[1] << 8) | data[0];
			tempY = (data[3] << 8) | data[2];
			tempZ = (data[5] << 8) | data[4];
			g_temp.x += tempX;
			g_temp.y += tempY;
			g_temp.z += tempZ;
			suspendCallerUntil(NOW()+5*MILLISECONDS);
		}
	}
	g_offset.x = g_temp.x/(float)(CALIBRATION_VALUES-failCounter);
	g_offset.y = g_temp.y/(float)(CALIBRATION_VALUES-failCounter);
	g_offset.z = g_temp.z/(float)(CALIBRATION_VALUES-failCounter);
	g_temp.x = g_temp.y = g_temp.z = 0;
	imu_data.calibrating = false;
	PRINTF("Gyro Offsets [raw] Found: X %f Y %f Z %f\n",j, g_offset.x,g_offset.y,g_offset.z);
	suspendCallerUntil(NOW()+2*SECONDS);
	calibrate_gyroscope = false;
}

/*
 * Reads raw data from accelerometer
 */
void IMU::readAcc() {
	uint8_t data[6] = { };
	int retVal = HAL_I2C_2.writeRead(LSM9DS0_XM, LSM9DS0_OUT_X_L_A, 1, data, 6);
	if (retVal <= 0)
		I2CError();
	else {
		a_raw.x = -(data[1] << 8) | data[0];
		a_raw.y = -(data[3] << 8) | data[2];
		a_raw.z = -(data[5] << 8) | data[4];
	}
}

/*
 * calculate linear velocity [mg]
 */
void IMU::calculateAcc() {
	readAcc();
	imu_data.ax = (((float)a_raw.x)-a_offset.x)*ACC_SENSITIVITY_2;
	imu_data.ay = (((float)a_raw.y)-a_offset.y)*ACC_SENSITIVITY_2;
	imu_data.az = (((float)a_raw.z)-a_offset.z)*ACC_SENSITIVITY_2;
}

/*
 * Calibration function for accelerometer
 * Takes X samples in 3 different positions (one for each axis) to the the offset from 1g
 */
void IMU::calibrateAcc() {
	imu_data.calibrating = true;
	imu_topic.publish(imu_data);
	bool buttonPressed = false;
	USERBUTTON.resetInterruptEventStatus();
	bool xAxis = false, yAxis = false, zAxis = false;
	char axis;
	a_offset.x = a_offset.y = a_offset.z = 0;
	PRINTF("Calibrating accelerometer!\nPress UserButton, if device is in first position!\n");
	while (!(xAxis && yAxis && zAxis)) {
		buttonPressed = USERBUTTON.isDataReady();
		if (buttonPressed) {
			a_temp.x = a_temp.y = a_temp.z = 0;
			PRINTF("Getting samples, do not move device!\n");
			suspendCallerUntil(NOW()+1*SECONDS);
			buttonPressed = false;
			USERBUTTON.resetInterruptEventStatus();
			for (int i = 0; i < CALIBRATION_VALUES; i++) {
				readAcc();
				a_temp.x += a_raw.x;
				a_temp.y += a_raw.y;
				a_temp.z += a_raw.z;
				suspendCallerUntil(NOW() + 5*MILLISECONDS);
			}
			PRINTF("%d %d %d\n",a_temp.x,a_temp.y,a_temp.z);
			axis = findAxis(&a_temp);
			PRINTF("%c-Axis found!\n",axis);
			switch(axis) {
			case 'x':
				if (xAxis == false) {
					PRINTF("X-Axis done!\nPress UserButton, if device is in next position!\n");
					a_offset.y += (a_temp.y >> 10);
					a_offset.z += (a_temp.z >> 10);
					xAxis = true;
				} else {
					PRINTF("Axis already done!\n");
				}
				break;
			case 'y':
				if (yAxis == false) {
					PRINTF("Y-Axis done!\nPress UserButton, if device is in next position!\n");
					a_offset.x += (a_temp.x >> 10);
					a_offset.z += (a_temp.z >> 10);
					yAxis = true;
				} else {
					PRINTF("Axis already done!\n");
				}
				break;
			case 'z':
				if (zAxis == false) {
					PRINTF("Z-Axis done!\nPress UserButton, if device is in next position!\n");
					a_offset.x += (a_temp.x >> 10);
					a_offset.y += (a_temp.y >> 10);
					zAxis = true;
				} else {
					PRINTF("Axis already done!\n");
				}
				break;
			}
		}
	}
	a_offset.x /= 2.0;
	a_offset.y /=2.0;
	a_offset.z /= 2.0;
	imu_data.calibrating = false;
	PRINTF("Offsets [mg] found!\nX: %d Y: %d Z: %d\n",a_offset.x,a_offset.y,a_offset.z);
	suspendCallerUntil(NOW()+2*SECONDS);
	calibrate_accelerometer = false;
}


/*
 * Read raw data from magnetometer
 */
void IMU::readMag() {
	uint8_t data[6] = { };
	int retVal = HAL_I2C_2.writeRead(LSM303DLM_XM, LSM303DLM_OUT_X_H_M, 1, data, 6);
	if (retVal <= 0)
		I2CError();
	else {
		m_raw.x = (data[0] << 8) | data[1];
		m_raw.y = (data[2] << 8) | data[3];
		m_raw.z = (data[4] << 8) | data[5];
	}
}

/*
 * calculates magnetometer data to scale from -1000 to 1000
 */
void IMU::calculateMag() {
	readMag();
	float mv[3];
	/* soft iron calibration */
/*	mv[0] = m_raw.x - el_sc[0];
	mv[1] = m_raw.y - el_sc[1];
	mv[2] = m_raw.z - el_sc[2];
	imu_data.mx = el_ma[0][0] * mv[0] + el_ma[0][1] * mv[1] + el_ma[0][2] * mv[2];
	imu_data.my = el_ma[1][0] * mv[0] + el_ma[1][1] * mv[1] + el_ma[1][2] * mv[2];
	imu_data.mz = el_ma[2][0] * mv[0] + el_ma[2][1] * mv[1] + el_ma[2][2] * mv[2];
	*/
	/* hard iron calibration */
	imu_data.mx = ((((float)m_raw.x)-m_offset.xMin)/(m_offset.xMax-m_offset.xMin)) * 2 - 1;
	imu_data.my = ((((float)m_raw.y)-m_offset.yMin)/(m_offset.yMax-m_offset.yMin)) * 2 - 1;
	imu_data.mz = ((((float)m_raw.z)-m_offset.zMin)/(m_offset.zMax-m_offset.zMin)) * 2 - 1;
}

/*
 * Hard iron calibration for magnetometer, not used anymore!
 * Finds min and max value for each axis
 */
void IMU::calibrateMag() {
	imu_data.calibrating = true;
	imu_topic.publish(imu_data);
	bool buttonPressed = false;
	PRINTF("Calibrating Magnetometer! Move around every axis!\nPress UserButton, if finished!\n");
	m_offset.xMin = m_offset.xMax = 0;
	m_offset.yMin = m_offset.yMax = 0;
	m_offset.zMin = m_offset.zMax = 0;
	while (!buttonPressed) {
		buttonPressed = USERBUTTON.isDataReady();
		readMag();
		if (m_raw.x < m_offset.xMin) m_offset.xMin = m_raw.x;
		if (m_raw.x > m_offset.xMax) m_offset.xMax = m_raw.x;
		if (m_raw.y < m_offset.yMin) m_offset.yMin = m_raw.y;
		if (m_raw.y > m_offset.yMax) m_offset.yMax = m_raw.y;
		if (m_raw.z < m_offset.zMin) m_offset.zMin = m_raw.z;
		if (m_raw.z > m_offset.zMax) m_offset.zMax = m_raw.z;
		suspendCallerUntil(NOW()+2*MILLISECONDS);
	}
	PRINTF("Values [raw] found!\nX: %d %d\nY: %d %d\nZ: %d %d\nCalibration of magnetometer done!\n",
			m_offset.xMax, m_offset.xMin,
			m_offset.yMax, m_offset.yMin,
			m_offset.zMax, m_offset.zMin);
	imu_data.calibrating = false;
	suspendCallerUntil(NOW()+2*SECONDS);
	USERBUTTON.resetInterruptEventStatus();
	calibrate_magnetometer = false;
}

/*! public functions to initiate sensor calibrations */

void IMU::setCalibrateMagnetometer() {
	calibrate_magnetometer = true;
}

void IMU::setCalibrateGyroscope() {
	calibrate_gyroscope = true;
}

void IMU::setCalibrateAccelerometer() {
	calibrate_accelerometer = true;
}

/** AHRS - simple complementary filter for IMU data fusion */

/*
 * Calculates roll and pitch from accelerometer data
 */
void IMU::calculateAccEuler() {
	xm_euler.roll = imu_data.ax / sqrt(imu_data.ay*imu_data.ay + imu_data.az*imu_data.az);
	xm_euler.pitch = imu_data.ay / sqrt(imu_data.ax*imu_data.ax + imu_data.az*imu_data.az);
}

/*
 * Calculates current heading from magnetometer data
 * Roll and Pitch from accelerometer are used to compensate the tilt
 */
void IMU::calculateHeading() {
	float sr = sin(xm_euler.roll);
	float cr = cos(xm_euler.roll);
	float sp = sin(xm_euler.pitch);
	float cp = cos(xm_euler.pitch);
	float mxh, myh = 0.0;
	mxh = imu_data.mx * cp + imu_data.mz * sp;
	myh = imu_data.mx * sr * sp + imu_data.my * cr - imu_data.mz * sr * cp;
	xm_euler.heading = atan2(myh,mxh);
	if (xm_euler.heading < 0) xm_euler.heading +=2*M_PI;
}

/*
 * Calculates euler angles from gyroscope data via integration and feedback of the filtered angles
 */
void IMU::calculateGyroEuler() {
	float sr = sin(ahrs_euler.roll);
	float cr = cos(ahrs_euler.roll);
	float sp = sin(ahrs_euler.pitch);
	float cp = cos(ahrs_euler.pitch);

	if (cp != 0) {
		gyro_euler.pitch	= ahrs_euler.pitch 	 + (((					cr*cp*imu_data.wy 	- sr*cp*imu_data.wz)/cp)*IMU_SAMPLING_RATE/1000.0);
		gyro_euler.roll		= ahrs_euler.roll 	 + (((cp*imu_data.wx + 	sr*sp*imu_data.wy 	+ cr*sp*imu_data.wz)/cp)*IMU_SAMPLING_RATE/1000.0);
		gyro_euler.heading	= ahrs_euler.heading + (((					sr*imu_data.wy 		+ cr*imu_data.wz)	/cp)*IMU_SAMPLING_RATE/1000.0);
	}
}

/*
 * performes fusion of calculated angles and deals with singularity at transition between 0 and 360 degrees
 */
void IMU::complementaryFilter() {
	calculateAccEuler();
	calculateHeading();
	calculateGyroEuler();
	if (!isnan(gyro_euler.pitch) | !isnan(gyro_euler.roll) | !isnan(gyro_euler.heading)) {

		//deal with singularity
		if (gyro_euler.heading - xm_euler.heading > M_PI) xm_euler.heading += 2*M_PI;
		if (xm_euler.heading - gyro_euler.heading > M_PI) gyro_euler.heading -= 2*M_PI;

		//fusion
		ahrs_euler.pitch = ((1-ALPHA)*xm_euler.pitch) + (ALPHA*gyro_euler.pitch);
		ahrs_euler.roll = ((1-ALPHA)*xm_euler.roll) + (ALPHA*gyro_euler.roll);
		ahrs_euler.heading = ((1-ALPHA)*xm_euler.heading) + (ALPHA*gyro_euler.heading);

		//make sure angle stays between 0 and 360 degrees
		if (ahrs_euler.heading > 2*M_PI) ahrs_euler.heading -=2*M_PI;
		if (ahrs_euler.heading < 0) ahrs_euler.heading += 2*M_PI;
	}
}

