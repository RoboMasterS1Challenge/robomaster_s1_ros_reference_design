#ifndef __MPU9250_H__
#define __MPU9250_H__
#include "SPI.h"

#define MPU9250_SPI	        hspi1
#define MPU9250_CS_GPIO    0
#define MPU9250_CS_PIN     1


extern SPI_HandleTypeDef hspi1;

typedef enum accel_range {
	ACCEL_RANGE_2G = 0,
	ACCEL_RANGE_4G,
	ACCEL_RANGE_8G,
	ACCEL_RANGE_16G
} accel_range;

typedef enum gyro_range {
	GYRO_RANGE_250DPS = 0,
	GYRO_RANGE_500DPS,
	GYRO_RANGE_1000DPS,
	GYRO_RANGE_2000DPS
} gyro_range;

typedef enum dlpf_bandwidth {
	DLPF_BANDWIDTH_184HZ = 0,
	DLPF_BANDWIDTH_92HZ,
	DLPF_BANDWIDTH_41HZ,
	DLPF_BANDWIDTH_20HZ,
	DLPF_BANDWIDTH_10HZ,
	DLPF_BANDWIDTH_5HZ
} dlpf_bandwidth;

typedef enum sample_rate_divider {
	LP_ACCEL_ODR_0_24HZ = 0,
	LP_ACCEL_ODR_0_49HZ,
	LP_ACCEL_ODR_0_98HZ,
	LP_ACCEL_ODR_1_95HZ,
	LP_ACCEL_ODR_3_91HZ,
	LP_ACCEL_ODR_7_81HZ,
	LP_ACCEL_ODR_15_63HZ,
	LP_ACCEL_ODR_31_25HZ,
	LP_ACCEL_ODR_62_50HZ,
	LP_ACCEL_ODR_125HZ,
	LP_ACCEL_ODR_250HZ,
	LP_ACCEL_ODR_500HZ
} sample_rate_divider;

uint8_t MPU9250_init();
void MPU9250_getData(int16_t* acc_data, int16_t* gyro_data, int16_t* mag_data);
void MPU9250_setSampleRateDivider(sample_rate_divider value);
void MPU9250_setDLPFBandwidth(dlpf_bandwidth value);
void MPU9250_setAccelRange(accel_range range);
void MPU9250_setGyroRange(gyro_range range);

#endif /* __MPU9250_H__ */
