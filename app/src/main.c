#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <zephyr/drivers/i2c.h>

#include "app_version.h"

//BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
//             "Console device is not ACM CDC UART device");

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

void main(void)
{
//    const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
//    uint32_t dtr = 0;
//
//    if (usb_enable(NULL)) {
//        return;
//    }
//
//    /* Poll if the DTR flag was set */
//    while (!dtr) {
//        uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
//        /* Give CPU resources to low priority threads. */
//        k_sleep(K_MSEC(100));
//    }

    printk("Zephyr Example Application %s\n", APP_VERSION_STR);

    const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
    int led_on = 0;

    while (1) {
        printk("Hello World! %s\n", CONFIG_ARCH);

        gpio_pin_set_dt(&led, led_on);
        led_on = !led_on;

        k_sleep(K_SECONDS(1));
    }
//    const struct device *const dev_imu = DEVICE_DT_GET_ONE(bosch_bmi270);
//    struct sensor_value acc[3], gyr[3];
//    struct sensor_value full_scale, sampling_freq, oversampling;
//
////    if (!device_is_ready(dev_imu)) {
////        printf("Device %s is not ready\n", dev_imu->name);
////        return;
////    }
//
//    printf("Device %p name is %s\n", dev_imu, dev_imu->name);
//
//    /* Setting scale in G, due to loss of precision if the SI unit m/s^2
//     * is used
//     */
//    full_scale.val1 = 2;            /* G */
//    full_scale.val2 = 0;
//    sampling_freq.val1 = 100;       /* Hz. Performance mode */
//    sampling_freq.val2 = 0;
//    oversampling.val1 = 1;          /* Normal mode */
//    oversampling.val2 = 0;
//
//    sensor_attr_set(dev_imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE,
//                    &full_scale);
//    sensor_attr_set(dev_imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING,
//                    &oversampling);
//    /* Set sampling frequency last as this also sets the appropriate
//     * power mode. If already sampling, change to 0.0Hz before changing
//     * other attributes
//     */
//    sensor_attr_set(dev_imu, SENSOR_CHAN_ACCEL_XYZ,
//                    SENSOR_ATTR_SAMPLING_FREQUENCY,
//                    &sampling_freq);
//
//
//    /* Setting scale in degrees/s to match the sensor scale */
//    full_scale.val1 = 500;          /* dps */
//    full_scale.val2 = 0;
//    sampling_freq.val1 = 100;       /* Hz. Performance mode */
//    sampling_freq.val2 = 0;
//    oversampling.val1 = 1;          /* Normal mode */
//    oversampling.val2 = 0;
//
//    sensor_attr_set(dev_imu, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE,
//                    &full_scale);
//    sensor_attr_set(dev_imu, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING,
//                    &oversampling);
//    /* Set sampling frequency last as this also sets the appropriate
//     * power mode. If already sampling, change sampling frequency to
//     * 0.0Hz before changing other attributes
//     */
//    sensor_attr_set(dev_imu, SENSOR_CHAN_GYRO_XYZ,
//                    SENSOR_ATTR_SAMPLING_FREQUENCY,
//                    &sampling_freq);
//
//    while (1) {
//        /* 10ms period, 100Hz Sampling frequency */
//        k_sleep(K_MSEC(10));
//
//        sensor_sample_fetch(dev_imu);
//
//        sensor_channel_get(dev_imu, SENSOR_CHAN_ACCEL_XYZ, acc);
//        sensor_channel_get(dev_imu, SENSOR_CHAN_GYRO_XYZ, gyr);
//
//        printf("AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; "
//               "GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d;\n",
//               acc[0].val1, acc[0].val2,
//               acc[1].val1, acc[1].val2,
//               acc[2].val1, acc[2].val2,
//               gyr[0].val1, gyr[0].val2,
//               gyr[1].val1, gyr[1].val2,
//               gyr[2].val1, gyr[2].val2);
//    }
}
