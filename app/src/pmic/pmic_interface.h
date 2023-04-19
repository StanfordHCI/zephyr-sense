/**
******************************************************************************
* @file     pmic_interface.h
* @author   Olaf Hichwa
* @version  V0.1
* @date     16-Mar-2023
* @brief    Power Management IC Interface header
******************************************************************************
* @attention
******************************************************************************
**/

#ifndef __PMIC_INTERFACE_H
#define __PMIC_INTERFACE_H

#include <string>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include "MAX20360_REGISTERS.h"

//q: is this a bad idea?
using namespace std;

/**--------------------------- External Variables --------------------------*/
// TODO: contain these within the module
struct i2c_register_s
{
    uint8_t slave_address;
    uint8_t register_address;
    uint8_t register_data;
};
struct pmic_register_s
{
    uint8_t slave_address;
    uint8_t register_address;
    string register_name;
};
#define I2C_DEV_NODE    DT_NODELABEL(max20360regulator)
extern struct i2c_dt_spec max20360regulator_dev;

typedef struct i2c_register_s i2c_register_t;
typedef struct pmic_register_s pmic_register_t;


/** setup PMIC GPIOs */
#define USER_NODE DT_PATH(zephyr_user)
constexpr struct gpio_dt_spec max20360regulator_gpio_0 = GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, regulator_gpios, 0);
constexpr struct gpio_dt_spec max20360regulator_gpio_1 = GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, regulator_gpios, 1);
constexpr struct gpio_dt_spec max20360regulator_gpio_2 = GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, regulator_gpios, 2);
constexpr struct gpio_dt_spec max20360regulator_gpio_3 = GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, regulator_gpios, 3);
constexpr struct gpio_dt_spec max20360regulator_gpio_4 = GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, regulator_gpios, 4);
constexpr struct gpio_dt_spec max20360regulator_gpio_5 = GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, regulator_gpios, 5);
constexpr struct gpio_dt_spec max20360regulator_gpio_6 = GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, regulator_gpios, 6);
constexpr struct gpio_dt_spec max20360regulator_gpio_7 = GPIO_DT_SPEC_GET_BY_IDX(USER_NODE, regulator_gpios, 7);


/**--------------------------- External Functions --------------------------*/
uint8_t max20360_read_script();
uint8_t config_pmic(i2c_register_t *register_array);
uint8_t config_pmic(i2c_register_t *register_array, uint8_t array_size);
uint8_t calculate_fletcher_checksum_bit_1(uint8_t slave_address, uint8_t register_address, uint8_t register_data);
uint8_t calculate_fletcher_checksum_bit_2(uint8_t slave_address, uint8_t register_address, uint8_t register_data);
uint8_t i2c_write_to_dt(const i2c_dt_spec *device, uint8_t register_address, uint8_t register_data);
uint8_t i2c_write_to_pmic_regulators(uint8_t register_address, uint8_t register_data);
uint8_t  i2c_read_from_pmic_regulators(uint8_t register_address, uint8_t *register_data);
uint8_t i2c_write_to_pmic_haptics(uint8_t register_address, uint8_t register_data);
uint8_t i2c_write_to_pmic_charger(uint8_t register_address, uint8_t register_data);

/**
 * @brief  Initializes the power system setting all regulators to the defined default values
 * @param  None
 * @retval 0 if successful, 1 if error
*/
int power_init();

uint8_t max20360_set_load_switch_1();
uint8_t max20360_set_load_switch_2();
uint8_t max20360_clear_load_switch_1();
uint8_t max20360_clear_load_switch_2();
uint8_t max20360_enable_ldo1();
uint8_t max20360_disable_ldo1();

#endif //__PMIC_INTERFACE_H
