#include "IMU.h"


/** IMU specific data */
/* Gyro control registers */
uint8_t CTRL_REG1_G[2] = {0x20,0b11111111}; // ODR=760Hz (11) + Cutoff=100Hz (11) + Normal Mode (1) + Gyro Axis Enabled (111)
uint8_t CTRL_REG4_G[2] = {0x23,0b10110000}; // BDU(1) + Data LSb @ lower address (0) + Full-scale = 2000 dps (11) + (0) + Self-test disabled (00) + 4-wire interface SPI (0)
//uint8_t CTRL_REG5_G[2] = {0x24,0b00001010}; // normal mode (0) + FIFO disable (0) + x + low pass filter LPF2 and low pass interrupt data(01010)
/* XM control registers */
uint8_t CTRL_REG1_XM[2] = {0x20, 0b01111111}; //200Hz(0111) and BDU(1) and Axis Enabled (111)
uint8_t CTRL_REG2_XM[2] = {0x21, 0b01000000}; //773Hz Filter (10) and 2g Scale (000) and normal mode (00) and 4-wire SPI (0)
uint8_t CTRL_REG5_XM[2] = {0x24, 0b01110000}; //Temperatur enabled (1) and Low Magnetic Resolution (11) and 50Hz (100) and (00)
uint8_t CTRL_REG6_XM[2] = {0x25, 0b00000000}; //x00xxxxx 2gauss
uint8_t CTRL_REG7_XM[2] = {0x26, 0b10000000}; //Normal mode (00) + internal filter bypassed (0) + (00) + low-power (0) + power down (10)
/* Starting adresses for sensors */
uint8_t LSM9DS0_OUT_X_L_G[1] = {0x28 | 0x80}; // Gyroscope OUT_X_L address with multiple bytes indicator (0x80)
uint8_t LSM9DS0_OUT_X_L_A[1] = {0x28 | 0x80}; // Accelerometer OUT_X_L address with multiple bytes indicator (0x80)
uint8_t LSM9DS0_OUT_X_L_M[1] = {0x08 | 0x80}; // Magnetometer OUT_X_L address with multiple bytes indicator (0x80)

HAL_I2C HAL_I2C2(I2C_IDX2);

HAL_GPIO CS_G(GPIO_018); /* declare HAL_GPIO for GPIO_018 = PB2 (IMU Chip Select pin for the Gyro) */
HAL_GPIO IMU_EN(GPIO_055); /* declare HAL_GPIO for GPIO_055 = PD7 (IMU Power Enable pin) */
HAL_GPIO USERBUTTON(GPIO_000); //UserButton


IMU::IMU(const char* name) : Thread (name) {
	g_counter = a_counter = m_counter =  0;
	g_temp.x = g_temp.y = g_temp.z = 0;
	g_offset.x = g_offset.y = g_offset.z = 0;
	g_offset.x = -3356.0;
	g_offset.y = 189.0;
	g_offset.z = 64.0;
	m_offset.xMax = 3306;
	m_offset.xMin = -9577;
	m_offset.yMax = 7789;
	m_offset.yMin = -4338;
	m_offset.zMin = 6998;
	m_offset.zMax = -4255;
}

IMU::~IMU() {
	// TODO Auto-generated destructor stub
}

void IMU::init() {
	HAL_I2C2.init(400000);
	IMU_EN.init(true,1,1);

	/* Userbutton for calibration */
	USERBUTTON.init(false,1,0);						//init Button as input
	USERBUTTON.config(GPIO_CFG_IRQ_SENSITIVITY,2);	//set sensitivity to falling edge (1)
	USERBUTTON.interruptEnable(true);

	initSensors();
}

void IMU::initSensors() {
	CS_G.init(true,1,1);
	/* initialize Gyro */
	HAL_I2C2.write(LSM9DS0_G, CTRL_REG1_G, 2);
	HAL_I2C2.write(LSM9DS0_G, CTRL_REG4_G, 2);

	/* initialize Acc and Mag */
	HAL_I2C2.write(LSM9DS0_XM, CTRL_REG1_XM, 2);
	HAL_I2C2.write(LSM9DS0_XM, CTRL_REG2_XM, 2);
	HAL_I2C2.write(LSM9DS0_XM, CTRL_REG5_XM, 2);
	HAL_I2C2.write(LSM9DS0_XM, CTRL_REG6_XM, 2);
	HAL_I2C2.write(LSM9DS0_XM, CTRL_REG7_XM, 2);
}

void IMU::run() {
//	calibrateGyro();
//	calibrateMag();
//	calibrateAcc();
//	suspendCallerUntil();
	TIME_LOOP(0,IMU_SAMPLING_RATE*MILLISECONDS) {
		calculateGyro();
		calculateMag();
		calculateAcc();
		imu_topic.publish(values);
	}
}

void IMU::I2CError() {
	HAL_I2C2.reset();
	HAL_I2C2.init(400000);
	IMU_EN.setPins(0);
	suspendCallerUntil(NOW()+5*MILLISECONDS);
	IMU_EN.setPins(1);
	IMU_EN.init(true,1,1);
	initSensors();
}

/**
 * Gyroscope functions
 */
void IMU::readGyro() {
	uint8_t data[6] = {};
	int retVal = HAL_I2C2.writeRead(LSM9DS0_G, LSM9DS0_OUT_X_L_G, 1, data, 6);
	HAL_I2C2.suspendUntilReadFinished();
	if (retVal <= 0) {
		I2CError();
	} else {
		g_raw.x = (int16_t)((data[1] << 8) | data[0]);
		g_raw.y = (int16_t)((data[3] << 8) | data[2]);
		g_raw.z = (int16_t)((data[5] << 8) | data[4]);
	}
}

void IMU::calculateGyro() {
	readGyro();
	if (g_counter < MEAN_FILTER_SAMPLES) {
		g_counter++;
		g_temp.x += g_raw.x;
		g_temp.y += g_raw.y;
		g_temp.z += g_raw.z;
	}
	if (g_counter == MEAN_FILTER_SAMPLES) {
		values.wx = degToRad(((g_temp.x/(float)MEAN_FILTER_SAMPLES)-g_offset.x)*GYRO_SENSITIVITY);
		values.wy = degToRad(((g_temp.y/(float)MEAN_FILTER_SAMPLES)-g_offset.y)*GYRO_SENSITIVITY);
		values.wz = degToRad(((g_temp.z/(float)MEAN_FILTER_SAMPLES)-g_offset.z)*GYRO_SENSITIVITY);
		g_counter = 0;
		g_temp.x = g_temp.y = g_temp.z = 0;

	}
}

void IMU::calibrateGyro() {
	values.calibrating = true;
	imu_topic.publish(values);
	PRINTF("Calibrating gyro...\n");
	int16_t tempX =0, tempY = 0, tempZ = 0;
	g_temp.x = g_temp.y = g_temp.z = 0;
	uint8_t data[6] = {};
	int failCounter = 0;
	int j = 0;
	for (int i = 0; i < CALIBRATION_VALUES; i++) {
		j = i;
		int retVal = HAL_I2C2.writeRead(LSM9DS0_G, LSM9DS0_OUT_X_L_G, 1, data, 6);
		HAL_I2C2.suspendUntilReadFinished();
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
	values.calibrating = false;
	PRINTF("Gyro Offsets [raw] Found: X %f Y %f Z %f\n",j, g_offset.x,g_offset.y,g_offset.z);
	suspendCallerUntil(NOW()+2*SECONDS);
}

/**
 * Accelerometer functions
 */
void IMU::readAcc() {
	uint8_t data[6] = { };
	int retVal = HAL_I2C2.writeRead(LSM9DS0_XM, LSM9DS0_OUT_X_L_A, 1, data, 6);
	if (retVal <= 0)
		I2CError();
	else {
		a_raw.x = -(data[1] << 8) | data[0];
		a_raw.y = -(data[3] << 8) | data[2];
		a_raw.z = -(data[5] << 8) | data[4];
	}
}

void IMU::calculateAcc() {
	readAcc();
	if (a_counter < MEAN_FILTER_SAMPLES) {
		a_counter++;
		a_temp.x += a_raw.x;
		a_temp.y += a_raw.y;
		a_temp.z += a_raw.z;
	}
	if (a_counter == MEAN_FILTER_SAMPLES) {
		values.ax = (((float)a_temp.x/MEAN_FILTER_SAMPLES)-a_offset.x)*ACC_SENSITIVITY;
		values.ay = (((float)a_temp.y/MEAN_FILTER_SAMPLES)-a_offset.y)*ACC_SENSITIVITY;
		values.az = (((float)a_temp.z/MEAN_FILTER_SAMPLES)-a_offset.z)*ACC_SENSITIVITY;
		a_counter = 0;
		a_temp.x = a_temp.y = a_temp.z = 0;

	}
}

void IMU::calibrateAcc() {
	values.calibrating = true;
	imu_topic.publish(values);
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
	values.calibrating = false;
	PRINTF("Offsets [mg] found!\nX: %5.2f Y: %5.2f Z: %5.2f\n",a_offset.x*ACC_SENSITIVITY,a_offset.y*ACC_SENSITIVITY,a_offset.z*ACC_SENSITIVITY);
	suspendCallerUntil(NOW()+2*SECONDS);
}


/**
 * Magnetometer functions
 */
void IMU::readMag() {
	uint8_t data[6] = { };
	int retVal = HAL_I2C2.writeRead(LSM9DS0_XM, LSM9DS0_OUT_X_L_M, 1, data, 6);
	if (retVal <= 0)
		I2CError();
	else {
		m_raw.x = (data[1] << 8) | data[0];
		m_raw.y = (data[3] << 8) | data[2];
		m_raw.z = (data[5] << 8) | data[4];
	}
}

void IMU::calculateMag() {
	if (m_counter < MEAN_FILTER_SAMPLES) {
		m_counter++;
		readMag();
		m_temp.x += m_raw.x;
		m_temp.y += m_raw.y;
		m_temp.z += m_raw.z;
	}
	if (m_counter == MEAN_FILTER_SAMPLES) {
		values.mx = ((((float)m_temp.x/MEAN_FILTER_SAMPLES)-m_offset.xMin)/(m_offset.xMax-m_offset.xMin)) * 2 - 1;
		values.my = ((((float)m_temp.y/MEAN_FILTER_SAMPLES)-m_offset.yMin)/(m_offset.yMax-m_offset.yMin)) * 2 - 1;
		values.mz = ((((float)m_temp.z/MEAN_FILTER_SAMPLES)-m_offset.zMin)/(m_offset.zMax-m_offset.zMin)) * 2 - 1;
		m_counter = 0;
		m_temp.x = m_temp.y = m_temp.z = 0;
	}
}

void IMU::calibrateMag() {
	values.calibrating = true;
	imu_topic.publish(values);
	bool buttonPressed = false;
	PRINTF("Calibrating Magnetometer! Move around every axis!\nPress UserButton, if finished!\n");
	while (!buttonPressed) {
		buttonPressed = USERBUTTON.isDataReady();
		readMag();
		if (m_raw.x < m_offset.xMin) m_offset.xMin = m_raw.x;
		if (m_raw.x > m_offset.xMax) m_offset.xMax = m_raw.x;
		if (m_raw.y < m_offset.yMin) m_offset.yMin = m_raw.y;
		if (m_raw.y > m_offset.yMax) m_offset.yMax = m_raw.y;
		if (m_raw.z < m_offset.zMin) m_offset.zMin = m_raw.z;
		if (m_raw.z > m_offset.zMax) m_offset.zMax = m_raw.z;
		suspendCallerUntil(NOW()+5*MILLISECONDS);
	}
	PRINTF("Values [raw] found!\nX: %d %d\nY: %d %d\nZ: %d %d\nCalibration of magnetometer done!\n",
			m_offset.xMax, m_offset.xMin,
			m_offset.yMax, m_offset.yMin,
			m_offset.zMax, m_offset.zMin);
	values.calibrating = false;
}


