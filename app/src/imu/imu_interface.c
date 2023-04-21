//
// Created by Jackie Yang on 4/19/23.
//

#define DT_DRV_COMPAT st_lsm6dsl
#include <stdio.h>
#include "imu_interface.h"
#include "logger/logger_interface.h"

const struct device *const accel_dev = DEVICE_DT_GET_ONE(st_lsm6dsl); // NOLINT(cppcoreguidelines-interfaces-global-init)

static inline float out_ev(struct sensor_value *val)
{
	return ((float)val->val1 + (float)val->val2 / 1000000);
}

static inline uint32_t out_ev_x(struct sensor_value *val)
{
	union {
		uint32_t u32;
		float f32;
	} u;
	u.f32 = out_ev(val);
	return u.u32;
}

struct imu_data_pkt imu_data;

void lsm6dsl_trigger_handler(__attribute__((unused)) const struct device *dev,
			     __attribute__((unused)) struct gpio_callback *trig,
			     __attribute__((unused)) gpio_port_pins_t i)
{
	// generate random value
	uint64_t random_id = k_cycle_get_64();
	printk("Triggered! %llu\n", random_id);
	struct lsm6dsl_data *data = (struct lsm6dsl_data *)accel_dev->data;
	static uint8_t fifo_status1 = 0;
	static uint8_t fifo_status2 = 0;
	static uint8_t fifo_status3 = 0;
	static uint8_t fifo_status4 = 0;

	static uint16_t fifo_len = 0;
	static bool fifo_empty = false;
	static bool fifo_full_smart = false;
	static bool fifo_over_run = false;
	static bool fifo_full_watermark = false;
	static uint16_t fifo_pattern = 0;

	static uint8_t fifo_data_out_l = 0;
	static uint8_t fifo_data_out_h = 0;

	bool first = true;

	static unsigned char print_buffer[500] = {0};

	while (true) {
		//		data->hw_tf->read_reg(accel_dev, LSM6DSL_REG_FIFO_STATUS1,
		//&fifo_status1);
		data->hw_tf->read_reg(accel_dev, LSM6DSL_REG_FIFO_STATUS2, &fifo_status2);
		data->hw_tf->read_reg(accel_dev, LSM6DSL_REG_FIFO_STATUS3, &fifo_status3);
		data->hw_tf->read_reg(accel_dev, LSM6DSL_REG_FIFO_STATUS4, &fifo_status4);

		fifo_len = (((uint16_t)fifo_status2 & LSM6DSL_MASK_FIFO_STATUS2_DIFF_FIFO) << 8) |
			   fifo_status1;
		fifo_empty = fifo_status2 & LSM6DSL_MASK_FIFO_STATUS2_FIFO_EMPTY;
		fifo_full_smart = fifo_status2 & LSM6DSL_MASK_FIFO_STATUS2_FIFO_FULL_SMART;
		fifo_over_run = fifo_status2 & LSM6DSL_MASK_FIFO_STATUS2_OVER_RUN;
		fifo_full_watermark = fifo_status2 & LSM6DSL_MASK_FIFO_STATUS2_WATERM;
		fifo_pattern =
			(((uint16_t)fifo_status4 & LSM6DSL_MASK_FIFO_STATUS4_FIFO_PATTERN) << 8) |
			fifo_status3;

		if (fifo_empty) {
			break;
		}

		if (first) {
			first = false;
			// print fifo_len, fifo_empty, fifo_full_smart, fifo_over_run,
			// fifo_full_watermark
			printk("fifo_len: %d\n", fifo_len);
			printk("fifo_empty: %d\n", fifo_empty);
			printk("fifo_full_smart: %d\n", fifo_full_smart);
			printk("fifo_over_run: %d\n", fifo_over_run);
			printk("fifo_full_watermark: %d\n", fifo_full_watermark);
		}

		data->hw_tf->read_reg(accel_dev, LSM6DSL_REG_FIFO_DATA_OUT_L, &fifo_data_out_l);
		data->hw_tf->read_reg(accel_dev, LSM6DSL_REG_FIFO_DATA_OUT_H, &fifo_data_out_h);

		uint16_t fifo_tuple = fifo_pattern / 3;
		uint8_t fifo_index = fifo_pattern % 3;
		if (fifo_tuple == 2) { // mag
			int *driver_data_target = NULL;
			enum sensor_channel channel = SENSOR_CHAN_MAX;

			switch (fifo_index) {
			case 0:
				driver_data_target = &data->magn_sample_x;
				channel = SENSOR_CHAN_MAGN_X;
				break;
			case 1:
				driver_data_target = &data->magn_sample_y;
				channel = SENSOR_CHAN_MAGN_Y;
				break;
			case 2:
				driver_data_target = &data->magn_sample_z;
				channel = SENSOR_CHAN_MAGN_Z;
				break;
			default:
				printk("Invalid index\n");
				k_panic();
			}

			// save the data back to driver and read it out to do the conversion
			*driver_data_target = fifo_data_out_l + ((uint16_t)fifo_data_out_h << 8);
			if (sensor_channel_get(accel_dev, channel, &imu_data.mag[fifo_index]) < 0) {
				printk("Failed to get mag\n");
			}
		} else {
			// data is gyro, accel, or timestamp
			int data_group = fifo_tuple;
			if (data_group > 2) {
				data_group -= 1;
			}
			int data_group_type = data_group % 3;
			int data_group_index = data_group / 3;
			if (data_group_type == 2) { // timestamp
				if (fifo_index == 0) {
					imu_data.timestamp[data_group_index] =
						(uint32_t)fifo_data_out_l << 8 |
						(uint32_t)fifo_data_out_h << 16;
				} else if (fifo_index == 1) {
					imu_data.timestamp[data_group_index] |=
						(uint32_t)fifo_data_out_l;
				} else {
					// last data in this group
					snprintk(print_buffer, sizeof(print_buffer), "t: %u\n",
						 imu_data.timestamp[data_group_index]);
					print_to_uart(print_buffer);
					snprintf(print_buffer, sizeof(print_buffer),
						 "a: %x %x %x\n",
						 out_ev_x(&imu_data.accel[data_group_index][0]),
						 out_ev_x(&imu_data.accel[data_group_index][1]),
						 out_ev_x(&imu_data.accel[data_group_index][2]));
					print_to_uart(print_buffer);
					snprintf(print_buffer, sizeof(print_buffer),
						 "g: %x %x %x\n",
						 out_ev_x(&imu_data.gyro[data_group_index][0]),
						 out_ev_x(&imu_data.gyro[data_group_index][1]),
						 out_ev_x(&imu_data.gyro[data_group_index][2]));
					print_to_uart(print_buffer);
					if (data_group == 0) {
						snprintf(print_buffer, sizeof(print_buffer),
							 "m: %x %x %x\n",
							 out_ev_x(&imu_data.mag[0]),
							 out_ev_x(&imu_data.mag[1]),
							 out_ev_x(&imu_data.mag[2]));
						print_to_uart(print_buffer);
					}
				}
			} else {
				int *driver_data_target = NULL;
				enum sensor_channel channel = SENSOR_CHAN_MAX;
				if (data_group_type == 0) { // accel
					switch (fifo_index) {
					case 0:
						driver_data_target = &data->accel_sample_x;
						channel = SENSOR_CHAN_ACCEL_X;
						break;
					case 1:
						driver_data_target = &data->accel_sample_y;
						channel = SENSOR_CHAN_ACCEL_Y;
						break;
					case 2:
						driver_data_target = &data->accel_sample_z;
						channel = SENSOR_CHAN_ACCEL_Z;
						break;
					default:
						printk("Invalid index\n");
						k_panic();
					}
				} else { // gyro
					switch (fifo_index) {
					case 0:
						driver_data_target = &data->gyro_sample_x;
						channel = SENSOR_CHAN_GYRO_X;
						break;
					case 1:
						driver_data_target = &data->gyro_sample_y;
						channel = SENSOR_CHAN_GYRO_Y;
						break;
					case 2:
						driver_data_target = &data->gyro_sample_z;
						channel = SENSOR_CHAN_GYRO_Z;
						break;
					default:
						printk("Invalid index\n");
						k_panic();
					}
				}
				*driver_data_target =
					fifo_data_out_l + ((uint16_t)fifo_data_out_h << 8);
				if (data_group_type == 0) {
					if (sensor_channel_get(
						    accel_dev, channel,
						    &imu_data.accel[data_group_index][fifo_index]) <
					    0) {
						printk("Failed to get accel\n");
					}
				} else {
					if (sensor_channel_get(
						    accel_dev, channel,
						    &imu_data.gyro[data_group_index][fifo_index]) <
					    0) {
						printk("Failed to get gyro\n");
					}
				}
			}
		}
	}
	printk("FIFO data read complete. %llu\n", random_id);
}

int imu_init()
{
	if (!device_is_ready(accel_dev)) {
		printk("%s: device not ready.\n", accel_dev->name);
		return 1;
	}

	struct sensor_value a_odr_attr;

	/* set sampling frequency to 104Hz for accel */
	a_odr_attr.val1 = 6660;
	a_odr_attr.val2 = 0;

	if (sensor_attr_set(accel_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY,
			    &a_odr_attr) < 0) {
		printk("Cannot set sampling frequency for accelerometer.\n");
		return 0;
	}

	struct sensor_value g_odr_attr;

	/* set sampling frequency to 104Hz for accel */
	//	g_odr_attr.val1 = 6660;
	g_odr_attr.val1 = 6660;
	g_odr_attr.val2 = 0;

	if (sensor_attr_set(accel_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY,
			    &g_odr_attr) < 0) {
		printk("Cannot set sampling frequency for gyro.\n");
		return 0;
	}

	struct lsm6dsl_data *data = (struct lsm6dsl_data *)accel_dev->data;

	const struct lsm6dsl_config *config = accel_dev->config;

	// disable interrupt
	if (gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_DISABLE)) {
		printk("Could not disable interrupt\n");
		return 0;
	}
	gpio_remove_callback(config->int_gpio.port, &data->gpio_cb);

	gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);

	gpio_init_callback(&data->gpio_cb, lsm6dsl_trigger_handler, BIT(config->int_gpio.pin));

	if (gpio_add_callback(config->int_gpio.port, &data->gpio_cb) < 0) {
		printk("Could not set gpio callback.");
		return -EIO;
	}

	uint8_t ctrl10_c =
		(1 << LSM6DSL_SHIFT_CTRL10_C_TIMER_EN) | (1 << LSM6DSL_SHIFT_CTRL10_C_FUNC_EN);

	uint8_t wake_up_dur = (1 << LSM6DSL_SHIFT_WAKE_UP_DUR_TIMER_HR); // 2.5us timestamps

	//	constexpr uint16_t fifo_watermark = 1000u; // words or 2000 bytes

	if (data->hw_tf->write_data(accel_dev, LSM6DSL_REG_CTRL10_C, &ctrl10_c, 1) < 0) {
		printk("Failed to write ctrl10_c\n");
		return 0;
	}

	if (data->hw_tf->write_data(accel_dev, LSM6DSL_REG_WAKE_UP_DUR, &wake_up_dur, 1) < 0) {
		printk("Failed to write wake_up_dur\n");
		return 0;
	}

	const uint16_t fifo_watermark = 500u; // words or 2000 bytes

	uint8_t fifo_ctrl1 = (fifo_watermark & LSM6DSL_MASK_FIFO_CTRL1_FTH);
	uint8_t fifo_ctrl2 =
		(1 << LSM6DSL_SHIFT_FIFO_CTRL2_TIMER_PEDO_FIFO_EN |   // enable timestamp
		 0 << LSM6DSL_SHIFT_FIFO_CTRL2_TIMER_PEDO_FIFO_DRDY | // write when data is ready
		 0 << LSM6DSL_SHIFT_FIFO_CTRL2_FIFO_TEMP_EN |	      // disable temperature
		 fifo_watermark >> 8);
	uint8_t fifo_ctrl3 =
		(1 << LSM6DSL_SHIFT_FIFO_CTRL3_DEC_FIFO_GYRO) | // no decimation for gyro
		(1 << LSM6DSL_SHIFT_FIFO_CTRL3_DEC_FIFO_XL);	// no decimation for accel
	uint8_t fifo_ctrl4 =
		(0 << LSM6DSL_SHIFT_FIFO_CTRL4_STOP_ON_FTH) |	  // do not stop on watermark
		(0 << LSM6DSL_SHIFT_FIFO_CTRL4_ONLY_HIGH_DATA) |  // store all data
		(1 << LSM6DSL_SHIFT_FIFO_CTRL4_DEC_DS4_FIFO) |	  // no decimation for timestamp
		(0b101 << LSM6DSL_SHIFT_FIFO_CTRL4_DEC_DS3_FIFO); // 8x decimation for sensor data
								  // magnetometer
	uint8_t fifo_ctrl5 = (0b0110 << LSM6DSL_SHIFT_FIFO_CTRL5_ODR_FIFO) | // 416Hz
			     (0b000 << LSM6DSL_SHIFT_FIFO_CTRL5_FIFO_MODE);  // bypass mode

	if (data->hw_tf->write_data(accel_dev, LSM6DSL_REG_FIFO_CTRL1, &fifo_ctrl1, 1) != 0) {
		printk("Cannot set fifo_ctrl1.\n");
		return 0;
	}
	if (data->hw_tf->write_data(accel_dev, LSM6DSL_REG_FIFO_CTRL2, &fifo_ctrl2, 1) != 0) {
		printk("Cannot set fifo_ctrl2.\n");
		return 0;
	}
	if (data->hw_tf->write_data(accel_dev, LSM6DSL_REG_FIFO_CTRL3, &fifo_ctrl3, 1) != 0) {
		printk("Cannot set fifo_ctrl3.\n");
		return 0;
	}
	if (data->hw_tf->write_data(accel_dev, LSM6DSL_REG_FIFO_CTRL4, &fifo_ctrl4, 1) != 0) {
		printk("Cannot set fifo_ctrl4.\n");
		return 0;
	}
	if (data->hw_tf->write_data(accel_dev, LSM6DSL_REG_FIFO_CTRL5, &fifo_ctrl5, 1) != 0) {
		printk("Cannot set fifo_ctrl5.\n");
		return 0;
	}

	uint8_t int1_ctrl =
		(0 << LSM6DSL_SHIFT_INT1_CTRL_STEP_DETECTOR) | // enable step detector
		(0 << LSM6DSL_SHIFT_INT1_CTRL_SIGN_MOT) |      // enable sign motion
		(0 << LSM6DSL_SHIFT_INT1_CTRL_FULL_FLAG) |     // enable fifo full interrupt
		(0 << LSM6DSL_SHIFT_INT1_CTRL_FIFO_OVR) |      // enable fifo overrun interrupt
		(1 << LSM6DSL_SHIFT_INT1_FTH) |		       // enable fifo threshold interrupt
		(0 << LSM6DSL_SHIFT_INT1_CTRL_BOOT) |	       // enable boot interrupt
		(0 << LSM6DSL_SHIFT_INT1_CTRL_DRDY_G) |	       // enable gyro data ready interrupt
		(0 << LSM6DSL_SHIFT_INT1_CTRL_DRDY_XL);	       // enable accel data ready interrupt

	if (data->hw_tf->write_data(accel_dev, LSM6DSL_REG_INT1_CTRL, &int1_ctrl, 1) != 0) {
		printk("Cannot set int1_ctrl.\n");
		return 0;
	}

	printk("imu_init\n");
	return 0;
}

int imu_start()
{
	struct lsm6dsl_data *data = (struct lsm6dsl_data *)accel_dev->data;

	const struct lsm6dsl_config *config = accel_dev->config;

	if (data->hw_tf->update_reg(accel_dev, LSM6DSL_REG_FIFO_CTRL5,
				    LSM6DSL_MASK_FIFO_CTRL5_FIFO_MODE,
				    0b110 << LSM6DSL_SHIFT_FIFO_CTRL5_FIFO_MODE) != 0) {
		printk("Cannot set fifo mode.\n");
		return -1;
	}

	if (gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_TO_ACTIVE)) {
		printk("Could not configure interrupt\n");
		return -1;
	}

	if (gpio_pin_get(config->int_gpio.port, config->int_gpio.pin)) {
		lsm6dsl_trigger_handler(NULL, NULL, 0);
	}
	return 0;
}