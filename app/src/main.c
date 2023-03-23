/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>

#include "app_version.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL
);

void main(void) {
    int ret;
    const struct device *sensor;

    printk("Zephyr Example Application %s\n", APP_VERSION_STR);

    const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);

    while (1) {
        // blink
        gpio_pin_set_dt(&led, 1);

        k_sleep(K_MSEC(1000));

        gpio_pin_set_dt(&led, 0);
        k_sleep(K_MSEC(1000));
    }
}

