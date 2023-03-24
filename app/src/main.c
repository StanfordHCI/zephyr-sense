#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "app_version.h"

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
             "Console device is not ACM CDC UART device");

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

void main(void)
{
    const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    uint32_t dtr = 0;

    if (usb_enable(NULL)) {
        return;
    }

    /* Poll if the DTR flag was set */
    while (!dtr) {
        uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
        /* Give CPU resources to low priority threads. */
        k_sleep(K_MSEC(100));
    }

    printk("Zephyr Example Application %s\n", APP_VERSION_STR);

    const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
    int led_on = 0;

    while (1) {
        printk("Hello World! %s\n", CONFIG_ARCH);

        gpio_pin_set_dt(&led, led_on);
        led_on = !led_on;

        k_sleep(K_SECONDS(1));
    }
}
