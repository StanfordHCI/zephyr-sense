//
// Created by Jackie Yang on 4/19/23.
//

#ifndef APP_IMU_INTERFACE_H
#define APP_IMU_INTERFACE_H
#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <sensor/lsm6dsl/lsm6dsl.h>

extern const struct device *const accel_dev;

int imu_init();

int imu_start();

struct imu_data_pkt {
	struct sensor_value accel[8][3];
	struct sensor_value gyro[8][3];
	uint32_t timestamp[8];
	struct sensor_value mag[3];
};

#ifdef __cplusplus
}
#endif

#endif // APP_IMU_INTERFACE_H