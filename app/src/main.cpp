#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#include "pmic/pmic_interface.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "EndlessLoop"

static inline float out_ev(struct sensor_value *val)
{
	return ((float)val->val1 + (float)val->val2 / 1000000);
}

int init()
{
	/* Init Power Management Subsystem */
	power_init();
	printk("Power Init successful.\r\n");
	return 0;
}

int main()
{
	init();
	const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	if (usb_enable(nullptr)) {
		return 0;
	}

	/* Poll if the DTR flag was set */
	while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		/* Give CPU resources to low priority threads. */
		k_sleep(K_MSEC(100));
	}

	const struct device *const accel_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

	if (!device_is_ready(accel_dev)) {
		printk("%s: device not ready.\n", accel_dev->name);
		return 1;
	}

	struct sensor_value a_odr_attr {
	};

	/* set sampling frequency to 104Hz for accel */
	a_odr_attr.val1 = 104;
	a_odr_attr.val2 = 0;

	if (sensor_attr_set(accel_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY,
			    &a_odr_attr) < 0) {
		printk("Cannot set sampling frequency for accelerometer.\n");
		return 0;
	}

	struct sensor_value g_odr_attr {
	};

	/* set sampling frequency to 104Hz for accel */
	g_odr_attr.val1 = 104;
	g_odr_attr.val2 = 0;

	if (sensor_attr_set(accel_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY,
			    &g_odr_attr) < 0) {
		printk("Cannot set sampling frequency for gyro.\n");
		return 0;
	}

	struct sensor_value accel_x {
	};
	struct sensor_value accel_y {
	};
	struct sensor_value accel_z {
	};
	struct sensor_value gyro_x {
	};
	struct sensor_value gyro_y {
	};
	struct sensor_value gyro_z {
	};
	struct sensor_value magn_x {
	};
	struct sensor_value magn_y {
	};
	struct sensor_value magn_z {
	};
	char out_str[128];

	while (true) {
		/* lsm6dsl accel */
		sensor_sample_fetch_chan(accel_dev, SENSOR_CHAN_ACCEL_XYZ);
		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_X, &accel_x);
		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Z, &accel_z);
		sprintf(out_str, "accel (%f %f %f) m/s2", (double)out_ev(&accel_x),
			(double)out_ev(&accel_y), (double)out_ev(&accel_z));
		printk("%s\n", out_str);

		/* lsm6dsl gyro */
		sensor_sample_fetch_chan(accel_dev, SENSOR_CHAN_GYRO_XYZ);
		sensor_channel_get(accel_dev, SENSOR_CHAN_GYRO_X, &gyro_x);
		sensor_channel_get(accel_dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
		sensor_channel_get(accel_dev, SENSOR_CHAN_GYRO_Z, &gyro_z);
		sprintf(out_str, "gyro (%f %f %f) dps", (double)out_ev(&gyro_x),
			(double)out_ev(&gyro_y), (double)out_ev(&gyro_z));
		printk("%s\n", out_str);

		sensor_sample_fetch_chan(accel_dev, SENSOR_CHAN_MAGN_XYZ);
		sensor_channel_get(accel_dev, SENSOR_CHAN_MAGN_X, &magn_x);
		sensor_channel_get(accel_dev, SENSOR_CHAN_MAGN_Y, &magn_y);
		sensor_channel_get(accel_dev, SENSOR_CHAN_MAGN_Z, &magn_z);
		sprintf(out_str, "magn x:%f gauss y:%f gauss z:%f gauss", out_ev(&magn_x),
			out_ev(&magn_y), out_ev(&magn_z));
		printk("%s\n", out_str);

		k_sleep(K_MSEC(100));
	}
}

#pragma clang diagnostic pop
