/**
***************************************************************************
* @file         main.cpp
* @author       Tyler Chen
* @version      V3.0.01
* @date         17-Mar-2023
* @brief        Basic AFE read function, PMIC
******************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include <zephyr/drivers/spi.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include "pmic_interface.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "EndlessLoop"

/*----------------------------- Module Defines ----------------------------*/
/* Debug Printfs Setting (on/off) */
// #define DEBUGGING

#define NUM_CHANNELS 20
#define NUM_TOTAL_CHANNELS 21
#define BUFFER_LENGTH 100
#define BASELINE_BUFFER_LENGTH 50
#define BASELINE_SUBSAMPLING 2

/* Printing defines */
#define U8BYTES 2
#define U16BYTES 4
#define U32BYTES 8
#define U64BYTES 16
#define GMABYTES (U32BYTES*3 + 2 + 4)
#define QBYTES (U32BYTES*4 + 3 + 4)
#define OBYTES (U32BYTES*20 + (20-1) + 4)
#define OPBYTES (U16BYTES*40 + (40-1) + 4)

/* Intensity Guardrails */
#define TARGET_MAX 400000
#define TARGET_MIN 250000

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.
*/


/*---------------------------- Public Functions ---------------------------*/

static inline float out_ev(struct sensor_value *val)
{
    return (val->val1 + (float)val->val2 / 1000000);
}

/**
*  @brief    Main + Primary Loop.
*/
int main() {

    /* Init Power Management Subsystem */
    power_init();
    printk("Power Init successful.\r\n");

    const struct device *const accel_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

    if (!device_is_ready(accel_dev)) {
        printk("%s: device not ready.\n", accel_dev->name);
        return 1;
    }

    struct sensor_value a_odr_attr;

    /* set sampling frequency to 104Hz for accel */
    a_odr_attr.val1 = 104;
    a_odr_attr.val2 = 0;

    if (sensor_attr_set(accel_dev, SENSOR_CHAN_ACCEL_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &a_odr_attr) < 0) {
        printk("Cannot set sampling frequency for accelerometer.\n");
        return 0;
    }


    struct sensor_value g_odr_attr;

    /* set sampling frequency to 104Hz for accel */
    g_odr_attr.val1 = 104;
    g_odr_attr.val2 = 0;

    if (sensor_attr_set(accel_dev, SENSOR_CHAN_GYRO_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &g_odr_attr) < 0) {
        printk("Cannot set sampling frequency for gyro.\n");
        return 0;
    }

    struct sensor_value accel_x, accel_y, accel_z;
    struct sensor_value gyro_x, gyro_y, gyro_z;
    char out_str[128];

    while (true) {
        /* lsm6dsl accel */
        sensor_sample_fetch_chan(accel_dev, SENSOR_CHAN_ACCEL_XYZ);
        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_X, &accel_x);
        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Z, &accel_z);
        sprintf(out_str, "accel (%f %f %f) m/s2", (double)out_ev(&accel_x),
                (double)out_ev(&accel_y),
                (double)out_ev(&accel_z));
        printk("%s\n", out_str);

        /* lsm6dsl gyro */
        sensor_sample_fetch_chan(accel_dev, SENSOR_CHAN_GYRO_XYZ);
        sensor_channel_get(accel_dev, SENSOR_CHAN_GYRO_X, &gyro_x);
        sensor_channel_get(accel_dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
        sensor_channel_get(accel_dev, SENSOR_CHAN_GYRO_Z, &gyro_z);
        sprintf(out_str, "gyro (%f %f %f) dps", (double)out_ev(&gyro_x),
                (double)out_ev(&gyro_y),
                (double)out_ev(&gyro_z));
        printk("%s\n", out_str);
        k_sleep(K_MSEC(10));
    }

}

/*---------------------------- Private Functions ---------------------------*/



#pragma clang diagnostic pop
