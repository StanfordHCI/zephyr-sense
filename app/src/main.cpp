#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#include "pmic/pmic_interface.h"
#include "imu/imu_interface.h"
#include "logger/logger_interface.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "EndlessLoop"


/**
 * @brief Initialize all the chips
 * @return
 */
int init()
{
	/* Init Power Management Subsystem */
	power_init();
	imu_init();
	logger_init();

	printk("Init successful.\n");
	return 0;
}

/**
 * @brief Waiting for data collection to start
 * @return
 */
int wait_for_start()
{
	logger_wait_for_start();
	return 0;
}

int main()
{
	init();
	wait_for_start();

	imu_start();
	while (true) {
		k_sleep(K_MSEC(1000));
	}
}

#pragma clang diagnostic pop
