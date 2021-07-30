/*
 * BMI088.c
 *
 *  Created on: Jul 29, 2021
 *      Author: enesayanoglu
 */

#include "BMI088.h"

uint8_t BMI088_Init(BMI088IMU *imu, I2C_HandleTypeDef *I2Chandle, GPIO_TypeDef *intAccPinBank, uint16_t intAccPin, GPIO_TypeDef *intGyrPinBank, uint16_t intGyrPin) {
	imu->I2Chandle     = I2Chandle;
	imu->intAccPinBank = intAccPinBank;
	imu->intAccPin     = intAccPin;
	imu->intGyrPinBank = intGyrPinBank;
	imu->intGyrPin     = intGyrPin;
	imu->acc[0] = 0.0f;
	imu->acc[1] = 0.0f;
	imu->acc[2] = 0.0f;
	imu->gyr[0] = 0.0f;
	imu->gyr[1] = 0.0f;
	imu->gyr[2] = 0.0f;

	uint8_t txBuf[2];

	/*
	 * ACCELEROMETER
	 */

	/*Chip ID check*/
	uint8_t chipID;
	HAL_I2C_Mem_Read(imu->I2Chandle, BMI088_ACC_I2C_ADDR, BMI088_ACC_CHIP_ID, I2C_MEMADD_SIZE_8BIT, &chipID, 1, BMI088_I2C_TIMEOUT);

	if (chipID != 0x1E) {
		return 0;
	}

	/*Configure the accelerometer register
	 * [7:4] bits for bandwith of the accelerometer low pass filter
	 * 0x08 for OSR4, 0x09 for OSR2, 0x0A reset value
	 * [3:0] bits for set output data rate ODR
	 * 0x05 for 12.5Hz
	 * 0x06 for 25Hz
	 * 0x07 for 50Hz
	 * 0x08 for 100Hz
	 * 0x09 for 200Hz
	 * 0x0A for 400Hz
	 * 0x0B for 800Hz
	 * 0x0C for 1600Hz
	*/
	uint8_t accConf = 0x89;
	txBuf[0] = BMI088_ACC_CONF; txBuf[1] = accConf;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_ACC_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	/*Configure the accelerometer's range
	 * 0x00 for +-3g
	 * 0x01 for +-6g
	 * 0x02 for +-12g
	 * 0x03 for +-24g
	*/
	uint8_t accRange= 0x01;
	txBuf[0] = BMI088_ACC_RANGE; txBuf[1] = accRange;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_ACC_I2C_ADDR, tcBuf, 2, BMI088_I2C_TIMEOUT);

	/*Configure the input/output pin INT1 and INT2
	 * in register 0x53(INT1_IO_CONF
	 * 5. bit for Enable INT1 as an input pin.
	 * 4. bit for Enable INT1 as an output pin.
	 * 3. bit for Pin behavior 0 for Push-Pull, 1 for Open-Drain.
	 * 2. bit for Active state 0 for Active-low, 1 for Active-high.
	 * it is same for INT2
	*/
	uint8_t intConf = 0x0A;
	txBuf[0] = BMI088_INT1_IO_CONF; txBuf[1] = intConf;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_ACC_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	txBuf[0] = BMI088_INT2_IO_CONF;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_ACC_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	/*Map data Interrup register
	 * 7. bit for Map data ready interrupt to pin INT2
	 * 3. bit for Map data ready interrupt to pin INT1
	*/
	txBuf[0] = BMI088_INT1_INT2_MAP_DATA; txBuf[1] = 0x44;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_ACC_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);
	/*Setting accelerometer to active mode
	 * 0x03 for suspend mode
	 * 0x00 for active mode
	*/
	txBuf[0] = BMI088_ACC_PWR_CONF; txBuf[1] = 0x00;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_ACC_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);
	/*Switch the accelerometer on
	 * 0x00 Accelerometer off
	 * 0x03 Accelerometer on
	 */
	txBuf[0] = BMI088_ACC_PWR_CTRL; txBuf[1] = 0x04;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_ACC_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	/*
	 * GYROSCOPE
	 */

	/*Chip ID check*/
	HAL_I2C_Mem_Read(imu->I2Chandle, BMI088_GYR_I2C_ADDR, BMI088_GYR_CHIP_ID, I2C_MEMADD_SIZE_8BIT, &chipID, 1, BMI088_I2C_TIMEOUT);

	if (chipID != 0x0F) {
			return 0;
	}
	/* Setting the Gyroscope range
	 * 0x00 for +-2000 deg/s
	 * 0x01 for +-1000 deg/s
	 * 0x02 for +-500 deg/s
	 * 0x03 for +-250 deg/s
	 * 0x04 for +-125 deg/s
	*/
	uint8_t gyrRange = 0x01;
	txBuf[0] = BMI088_GYR_RANGE; txBuf[1] = gyrRange;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_GYR_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	/*Setting the Gyroscope Bandwidth
	 * 0x00 ODR(2000Hz) Filter Bandwidth(532 Hz)
	 * 0x01 ODR(2000Hz) Filter Bandwidth(230 Hz)
	 * 0x02 ODR(1000Hz) Filter Bandwidth(116 Hz)
	 * 0x03 ODR(400Hz) Filter Bandwidth(47 Hz)
	 * 0x04 ODR(200Hz) Filter Bandwidth(23 Hz)
	 * 0x05 ODR(100Hz) Filter Bandwidth(12 Hz)
	 * 0x06 ODR(200Hz) Filter Bandwidth(64 Hz)
	 * 0x07 ODR(100Hz) Filter Bandwidth(32 Hz)
	 */
	uint8_t gyrBandwidth = 0x02;
	txBuf[0] = BMI088_GYR_BANDWIDTH; txBuf[1] = gyrBandwidth;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_GYR_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	/*Gyroscope power mode
	 * 0x00 for normal mode
	 * 0x01 for suspend mode
	 * 0x02 for deep suspend mode
	 */
	txBuf[0] = BMI088_GYR_LPM1; txBuf[1] = 0x00;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_GYR_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	/*Gyroscope Interrupt Control
	 * 0x00 for No data ready interrupt is triggered
	 * 0x80 for Enables the new data interrupt to be trigerred on new data
	 */
	txBuf[0] = BMI088_GYR_INT_CTRL; txBuf[1] = 0x80;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_GYR_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	/*Set the electrical and logical properties of the interrupt pins
	 * 4. bit configure pin4 as Push-pull if it is 0, and Open-drain if it is 1
	 * 3. bit configure pin4's state if it is 0 Active low,if it is 1 Active High
	 * 2. bit configure pin3 as Push-pull if it is 0, and Open-drain if it is 1
	 * 1. bit configure pin3's state if it is 0 Active low,if it is 1 Active High
	 */
	txBuf[0] = BMI088_GYR_INT3_INT4_IO_CONF; txBuf[1] = 0x05;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_GYR_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	/*Map the data ready interrupt pin to one of the interrupt pins INT3 and INT4
	 * 0x00 Data ready interrupt is not mapped to any INT pin
	 * 0x01 Data ready interrupt is mapped to INT3 pin
	 * 0x80 Data ready interrupt is mapped to INT4 pin
	 * 0x81 Data ready interrupt is mapped to INT3 and INT4 pins
	 */
	txBuf[0] = BMI088_GYR_INT3_INT4_IO_MAP; txBuf[1] = 0x81;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_GYR_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	return 1;
}

void BMI088_ReadAcc(BMI088IMU *imu) {
	uint8_t rxBuf[6];
	HAL_I2C_Mem_Read(imu->I2Chandle, BMI088_ACC_I2C_ADDR, BMI088_ACC_DATA, I2C_MEMADD_SIZE_8BIT, rxBuf, 6, BMI088_I2C_TIMEOUT);

	int16_t accX = rxBuf[1];
			accX <<= 8;
			accX |= rxBuf[0];

	int16_t accY = rxBuf[3];
			accY <<= 8;
			accY |= rxBuf[2];

	int16_t accZ = rxBuf[5];
			accZ <<= 8;
			accZ |= rxBuf[4];

	/* Scale (to m/s^2) and re-map axes */
	imu->acc[0] = -accY * 0.00179626456f;
	imu->acc[1] = -accX * 0.00179626456f;
	imu->acc[2] = -accZ * 0.00179626456f;
}
void BMI088_ReadGyr(BMI088IMU *imu) {
	uint8_t rxBuf[6];
	HAL_I2C_Mem_Read(imu->I2Chandle, BMI088_GYR_I2C_ADDR, BMI088_GYR_DATA, I2C_MEMADD_SIZE_8BIT, rxBuf, 6, BMI088_I2C_TIMEOUT);

	int16_t gyrX = rxBuf[1];
			gyrX <<= 8;
			gyrX |= rxBuf[0];

	int16_t gyrY = rxBuf[3];
			gyrY <<= 8;
			gyrY |= rxBuf[2];

	int16_t gyrZ = rxBuf[5];
			gyrZ <<= 8;
			gyrZ |= rxBuf[4];

	/* Scale (to rad/s) and re-map axes */
	imu->gyr[0] = -gyrY * 0.00026632423f;
	imu->gyr[1] = -gyrX * 0.00026632423f;
	imu->gyr[2] = -gyrZ * 0.00026632423f;
}
