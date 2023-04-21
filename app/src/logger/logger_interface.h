//
// Created by Jackie Yang on 4/19/23.
//

#ifndef APP_LOGGER_INTERFACE_H
#define APP_LOGGER_INTERFACE_H
#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

extern const struct device *const uart_dev;

int logger_init();

int logger_wait_for_start();

void print_to_uart(unsigned char *print_buffer);

#ifdef __cplusplus
}
#endif
#endif // APP_LOGGER_INTERFACE_H
