#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>
#include <zephyr/bluetooth/services/ias.h>
#include <sys/types.h>

#include "app_version.h"

//BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
//             "Console device is not ACM CDC UART device");

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#include "cts.h"

/* Custom Service Variables */
#define BT_UUID_CUSTOM_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

static struct bt_uuid_128 vnd_uuid = BT_UUID_INIT_128(
        BT_UUID_CUSTOM_SERVICE_VAL);

static struct bt_uuid_128 vnd_enc_uuid = BT_UUID_INIT_128(
        BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));

static struct bt_uuid_128 vnd_auth_uuid = BT_UUID_INIT_128(
        BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2));

#define VND_MAX_LEN 20

static uint8_t vnd_value[VND_MAX_LEN + 1] = { 'V', 'e', 'n', 'd', 'o', 'r'};
static uint8_t vnd_auth_value[VND_MAX_LEN + 1] = { 'V', 'e', 'n', 'd', 'o', 'r'};
static uint8_t vnd_wwr_value[VND_MAX_LEN + 1] = { 'V', 'e', 'n', 'd', 'o', 'r' };

static ssize_t read_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset)
{
    const char *value = attr->user_data;

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
                             strlen(value));
}

static ssize_t write_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len, uint16_t offset,
                         uint8_t flags)
{
    uint8_t *value = attr->user_data;

    if (offset + len > VND_MAX_LEN) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(value + offset, buf, len);
    value[offset + len] = 0;

    return len;
}

static uint8_t simulate_vnd;
static uint8_t indicating;
static struct bt_gatt_indicate_params ind_params;

static void vnd_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    simulate_vnd = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

static void indicate_cb(struct bt_conn *conn,
                        struct bt_gatt_indicate_params *params, uint8_t err)
{
    printk("Indication %s\n", err != 0U ? "fail" : "success");
}

static void indicate_destroy(struct bt_gatt_indicate_params *params)
{
    printk("Indication complete\n");
    indicating = 0U;
}

#define VND_LONG_MAX_LEN 74
static uint8_t vnd_long_value[VND_LONG_MAX_LEN + 1] = {
        'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '1',
        'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '2',
        'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '3',
        'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '4',
        'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '5',
        'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '6',
        '.', ' ' };

static ssize_t write_long_vnd(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr, const void *buf,
                              uint16_t len, uint16_t offset, uint8_t flags)
{
    uint8_t *value = attr->user_data;

    if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
        return 0;
    }

    if (offset + len > VND_LONG_MAX_LEN) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(value + offset, buf, len);
    value[offset + len] = 0;

    return len;
}

static const struct bt_uuid_128 vnd_long_uuid = BT_UUID_INIT_128(
        BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3));

static struct bt_gatt_cep vnd_long_cep = {
        .properties = BT_GATT_CEP_RELIABLE_WRITE,
};

static int signed_value;

static ssize_t read_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                           void *buf, uint16_t len, uint16_t offset)
{
    const char *value = attr->user_data;

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
                             sizeof(signed_value));
}

static ssize_t write_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            const void *buf, uint16_t len, uint16_t offset,
                            uint8_t flags)
{
    uint8_t *value = attr->user_data;

    if (offset + len > sizeof(signed_value)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(value + offset, buf, len);

    return len;
}

static const struct bt_uuid_128 vnd_signed_uuid = BT_UUID_INIT_128(
        BT_UUID_128_ENCODE(0x13345678, 0x1234, 0x5678, 0x1334, 0x56789abcdef3));

static const struct bt_uuid_128 vnd_write_cmd_uuid = BT_UUID_INIT_128(
        BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef4));

static ssize_t write_without_rsp_vnd(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len, uint16_t offset,
                                     uint8_t flags)
{
    uint8_t *value = attr->user_data;

    if (!(flags & BT_GATT_WRITE_FLAG_CMD)) {
        /* Write Request received. Reject it since this Characteristic
         * only accepts Write Without Response.
         */
        return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
    }

    if (offset + len > VND_MAX_LEN) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(value + offset, buf, len);
    value[offset + len] = 0;

    return len;
}

/* Vendor Primary Service Declaration */
BT_GATT_SERVICE_DEFINE(vnd_svc,
        BT_GATT_PRIMARY_SERVICE(&vnd_uuid),
BT_GATT_CHARACTERISTIC(&vnd_enc_uuid.uuid,
BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE |
BT_GATT_CHRC_INDICATE,
BT_GATT_PERM_READ_ENCRYPT |
BT_GATT_PERM_WRITE_ENCRYPT,
read_vnd, write_vnd, vnd_value),
BT_GATT_CCC(vnd_ccc_cfg_changed,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT),
BT_GATT_CHARACTERISTIC(&vnd_auth_uuid.uuid,
BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
BT_GATT_PERM_READ_AUTHEN |
BT_GATT_PERM_WRITE_AUTHEN,
read_vnd, write_vnd, vnd_auth_value),
BT_GATT_CHARACTERISTIC(&vnd_long_uuid.uuid, BT_GATT_CHRC_READ |
BT_GATT_CHRC_WRITE | BT_GATT_CHRC_EXT_PROP,
BT_GATT_PERM_READ | BT_GATT_PERM_WRITE |
BT_GATT_PERM_PREPARE_WRITE,
read_vnd, write_long_vnd, &vnd_long_value),
BT_GATT_CEP(&vnd_long_cep),
BT_GATT_CHARACTERISTIC(&vnd_signed_uuid.uuid, BT_GATT_CHRC_READ |
BT_GATT_CHRC_WRITE | BT_GATT_CHRC_AUTH,
BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
read_signed, write_signed, &signed_value),
BT_GATT_CHARACTERISTIC(&vnd_write_cmd_uuid.uuid,
BT_GATT_CHRC_WRITE_WITHOUT_RESP,
BT_GATT_PERM_WRITE, NULL,
write_without_rsp_vnd, &vnd_wwr_value),
);

static const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
                      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
                      BT_UUID_16_ENCODE(BT_UUID_CTS_VAL)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_VAL),
};

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
    printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
        .att_mtu_updated = mtu_updated
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err 0x%02x)\n", err);
    } else {
        printk("Connected\n");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason 0x%02x)\n", reason);
}

static void alert_stop(void)
{
    printk("Alert stopped\n");
}

static void alert_start(void)
{
    printk("Mild alert started\n");
}

static void alert_high_start(void)
{
    printk("High alert started\n");
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
        .connected = connected,
        .disconnected = disconnected,
};

BT_IAS_CB_DEFINE(ias_callbacks) = {
        .no_alert = alert_stop,
        .mild_alert = alert_start,
        .high_alert = alert_high_start,
};

static void bt_ready(void)
{
    int err;

    printk("Bluetooth initialized\n");

    cts_init();

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }

    printk("Advertising successfully started\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
        .passkey_display = auth_passkey_display,
        .passkey_entry = NULL,
        .cancel = auth_cancel,
};

static void bas_notify(void)
{
    uint8_t battery_level = bt_bas_get_battery_level();

    battery_level--;

    if (!battery_level) {
        battery_level = 100U;
    }

    bt_bas_set_battery_level(battery_level);
}

static void hrs_notify(void)
{
    static uint8_t heartrate = 90U;

    /* Heartrate measurements simulation */
    heartrate++;
    if (heartrate == 160U) {
        heartrate = 90U;
    }

    bt_hrs_notify(heartrate);
}

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

    struct bt_gatt_attr *vnd_ind_attr;
    char str[BT_UUID_STR_LEN];
    int err;

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    bt_ready();

    bt_gatt_cb_register(&gatt_callbacks);
    bt_conn_auth_cb_register(&auth_cb_display);

    vnd_ind_attr = bt_gatt_find_by_uuid(vnd_svc.attrs, vnd_svc.attr_count,
                                        &vnd_enc_uuid.uuid);
    bt_uuid_to_str(&vnd_enc_uuid.uuid, str, sizeof(str));
    printk("Indicate VND attr %p (UUID %s)\n", vnd_ind_attr, str);

    while (1) {
        printk("Hello World! %s\n", CONFIG_ARCH);

        gpio_pin_set_dt(&led, led_on);
        led_on = !led_on;

        /* Current Time Service updates only when time is changed */
        cts_notify();

        /* Heartrate measurements simulation */
        hrs_notify();

        /* Battery level simulation */
        bas_notify();

        /* Vendor indication simulation */
        if (simulate_vnd && vnd_ind_attr) {
            if (indicating) {
                continue;
            }

            ind_params.attr = vnd_ind_attr;
            ind_params.func = indicate_cb;
            ind_params.destroy = indicate_destroy;
            ind_params.data = &indicating;
            ind_params.len = sizeof(indicating);

            if (bt_gatt_indicate(NULL, &ind_params) == 0) {
                indicating = 1U;
            }
        }

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
