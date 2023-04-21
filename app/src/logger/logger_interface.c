//
// Created by Jackie Yang on 4/19/23.
//

#include "logger_interface.h"

const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));

int logger_init()
{
	if (usb_enable(NULL)) {
		return 1;
	}
	return 0;
}

int logger_wait_for_start()
{
	uint32_t dtr = 0;
	/* Poll if the DTR flag was set */
	while (!dtr) {
		uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
		/* Give CPU resources to low priority threads. */
		k_sleep(K_MSEC(100));
	}
	return 0;
}


void print_to_uart(unsigned char *print_buffer)
{
	for (int j = 0; j < strlen(print_buffer); j++) {
		uart_poll_out(uart_dev, print_buffer[j]);
	}
}
