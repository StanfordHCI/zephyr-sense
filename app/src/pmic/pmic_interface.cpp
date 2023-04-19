//
// Created by Olaf Hichwa on 3/1/23.
//

#include "pmic_interface.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
struct i2c_dt_spec max20360regulator_dev = I2C_DT_SPEC_GET(I2C_DEV_NODE);
//
//i2c_register_t pmic_eval_to_C_reg_values[] = {
//    {0x50, 0x0F, 0x86}, 	//	ILimCntl
//    {0x50, 0x10, 0x27}, 	//	ChgCntl0
//    {0x50, 0x11, 0x27}, 	//	ChgCntl1
//    {0x50, 0x12, 0x65}, 	//	ChgTmr
//    {0x50, 0x13, 0xE4}, 	//	StepChgCfg0
//    {0x50, 0x1F, 0x3F}, 	//	Buck1VSet
//    {0x50, 0x27, 0x0F}, 	//	Buck2Ena
//    {0x50, 0x28, 0x81}, 	//	Buck2Cfg
//    {0x50, 0x34, 0x50}, 	//	Buck3Ena
//    {0x50, 0x36, 0x61}, 	//	Buck3Cfg1
//    {0x50, 0x53, 0x40}, 	//	LDO1VSet
//    {0x50, 0x55, 0x36}, 	//	LDO2Ena
//    {0x50, 0x56, 0x41}, 	//	LDO2Cfg
//    {0x50, 0x5A, 0x09}, 	//	LSWCfg
//    {0x50, 0x5D, 0x01}, 	//	LSW2Cfg
//    {0x50, 0x65, 0x01}, 	//	BoostVSet
//    {0x50, 0x81, 0x3C}, 	//	BuckCfg
//    {0x50, 0x14, 0x00},		// 	StepChgCfg1
//    {0x50, 0x38, 0x17},		// 	Buck3VSet
//    {0x50, 0x52, 0x1A},		// 	LDO1Cfg
//    {0x50, 0x57, 0x03}		// 	LDO2VSet
//};


i2c_register_t pmic_c_to_proto3_reg_values[] = {
    {0xA0, 0x03, 0x00}, // HptInt0
    {0xA0, 0x0C, 0x04}, // HptProt
    {0x50, 0x0C, 0x00}, // IntMask2
    {0x50, 0x0F, 0x06}, // ILimCntl
    {0x50, 0x10, 0x0D}, // ChgCntl0
    {0xA0, 0x11, 0x0E}, // HPTCfg0
    {0x50, 0x11, 0x73}, // ChgCntl1
    {0xA0, 0x12, 0x8B}, // HPTCfg1
    {0x50, 0x12, 0xFD}, // ChgTmr
    {0xA0, 0x13, 0x3E}, // HPTCfg2
    {0x50, 0x13, 0x30}, // StepChgCfg0
    {0xA0, 0x14, 0x19}, // HPTCfg3
    {0x50, 0x14, 0x17}, // StepChgCfg1
    {0xA0, 0x15, 0x03}, // HPTCfg4
    {0x50, 0x15, 0x3F}, // ThmCfg0
    {0xA0, 0x16, 0x05}, // HPTCfg5
    {0x50, 0x16, 0x1F}, // ThmCfg1
    {0xA0, 0x17, 0x11}, // HPTCfg6
    {0x50, 0x17, 0x1F}, // ThmCfg2
    {0xA0, 0x18, 0x08}, // HPTCfg7
    {0x50, 0x18, 0x00}, // HrvCfg0
    {0xA0, 0x19, 0x1F}, // HPTCfg8
    {0x50, 0x19, 0x3F}, // HrvCfg1
    {0xA0, 0x1A, 0x84}, // HPTCfg9
    {0x50, 0x1A, 0x10}, // IVMONCfg
    {0xA0, 0x1B, 0x07}, // HPTCfgA
    {0xA0, 0x1C, 0x40}, // HPTCfgB
    {0x50, 0x1C, 0x50}, // Buck1Cfg0
    {0xA0, 0x1D, 0xD0}, // HPTCfgC
    {0x50, 0x1D, 0x00}, // Buck1Cfg1
    {0xA0, 0x1E, 0x07}, // HPTCfgD
    {0x50, 0x1E, 0x00}, // Buck1Iset
    {0xA0, 0x1F, 0x06}, // HPTCfgE
    {0x50, 0x1F, 0x37}, // Buck1VSet
    {0xA0, 0x20, 0x24}, // HPTCfgF
    {0x50, 0x20, 0x01}, // Buck1Ctr
    {0x50, 0x21, 0x00}, // Buck1DvsCfg0
    {0xA0, 0x23, 0xD0}, // BEMFPeriod0
    {0x50, 0x23, 0x00}, // Buck1DvsCfg2
    {0xA0, 0x24, 0x07}, // BEMFPeriod1
    {0x50, 0x24, 0x00}, // Buck1DvsCfg3
    {0x50, 0x28, 0x51}, // Buck2Cfg
    {0x50, 0x2B, 0x32}, // Buck2VSet
    {0x50, 0x2C, 0x02}, // Buck2Ctr
    {0x50, 0x2D, 0x00}, // Buck2DvsCfg0
    {0xA0, 0x30, 0x7F}, // HptETRGOdAmp
    {0x50, 0x30, 0x00}, // Buck2DvsCfg3
    {0xA0, 0x31, 0x04}, // HptETRGOdDur
    {0x50, 0x31, 0x00}, // Buck2DvsCfg4
    {0xA0, 0x32, 0x3F}, // HptETRGActAmp
    {0x50, 0x32, 0x00}, // Buck2DvsSpi
    {0xA0, 0x33, 0x32}, // HptETRGActDur
    {0xA0, 0x34, 0xFF}, // HptETRGBrkAmp
    {0xA0, 0x35, 0x20}, // HptETRGBrkDur
    {0x50, 0x35, 0x51}, // Buck3Cfg
    {0x50, 0x36, 0x08}, // Buck3Cfg1
    {0x50, 0x37, 0x8F}, // Buck3Iset
    {0x50, 0x38, 0x1A}, // Buck3VSet
    {0x50, 0x39, 0x04}, // Buck3Ctr
    {0x50, 0x3A, 0x00}, // Buck3DvsCfg0
    {0xA0, 0x41, 0xAE}, // HptRAMDataH
    {0x50, 0x41, 0x85}, // BBstCfg
    {0xA0, 0x42, 0x84}, // HptRAMDataM
    {0x50, 0x42, 0x11}, // BBstVSet
    {0xA0, 0x43, 0x8D}, // HptRAMDataL
    {0x50, 0x43, 0xBF}, // BBstISet
    {0x50, 0x44, 0x73}, // BBstCfg1
    {0x50, 0x45, 0x08}, // BBstCtr0
    {0x50, 0x46, 0x00}, // BBstCtr1
    {0x50, 0x52, 0x03}, // LDO1Cfg
    {0x50, 0x53, 0x34}, // LDO1VSet
    {0x50, 0x54, 0x08}, // LDO1Ctr
    {0x50, 0x56, 0x01}, // LDO2Cfg
    {0x50, 0x57, 0x09}, // LDO2VSet
    {0x50, 0x5A, 0x03}, // LSWCfg
    {0x50, 0x5B, 0x02}, // LSW1Ctr
    {0x50, 0x5D, 0x03}, // LSW2Cfg
    {0x50, 0x5E, 0x04}, // LSW2Ctr
    {0x50, 0x60, 0x03}, // ChgPmpCfg
    {0x50, 0x61, 0x00}, // ChgPmpCtr
    {0x50, 0x63, 0x0A}, // BoostCfg
    {0x50, 0x64, 0x8F}, // BoostISet
    {0x50, 0x65, 0x00}, // BoostVSet
    {0x50, 0x67, 0x02}, // MPC0Cfg
    {0x50, 0x68, 0x02}, // MPC1Cfg
    {0x50, 0x69, 0x02}, // MPC2Cfg
    {0x50, 0x6A, 0x02}, // MPC3Cfg
    {0x50, 0x6B, 0x02}, // MPC4Cfg
    {0x50, 0x6C, 0x02}, // MPC5Cfg
    {0x50, 0x6D, 0x02}, // MPC6Cfg
    {0x50, 0x6E, 0x02}, // MPC7Cfg
    {0x50, 0x7D, 0x01}, // PFN
    {0x50, 0x7E, 0xB9}, // BootCfg
    {0x50, 0x7F, 0x01}, // PwrCfg
    {0x50, 0x80, 0x00}, // PwrCmd
    {0x50, 0x81, 0x38}, // BuckCfg
    {0x50, 0x86, 0x81}, // SFOUTCtr
    {0x50, 0x87, 0x00}, // SFOUTMPC
    {0x50, 0x1B, 0xE0}, // Buck1Ena
    {0x50, 0x27, 0xE0}, // Buck2Ena
    {0x50, 0x34, 0xE1}, // Buck3Ena
    {0x50, 0x40, 0xE1}, // BBstEna
    {0x50, 0x51, 0xE2}, // LDO1Ena
    {0x50, 0x55, 0xE1}, // LDO2Ena
    {0x50, 0x59, 0xE2}, // LSW1Ena
    {0x50, 0x5C, 0xE2}, // LSW2Ena
    {0x50, 0x5F, 0xE0}, // ChgPmpEna
    {0x50, 0x62, 0xE1} // BoostEna



};

pmic_register_t pmic_registers[] = {
        {0xA0, 0x01, "HptStatus1"},
        {0xA0, 0x01, "HptStatus1"},
        {0xA0, 0x02, "HptStatus2"},
        {0xA0, 0x03, "HptInt0"},
        {0xA0, 0x04, "HptInt1"},
        {0xA0, 0x05, "HptInt2"},
        {0xA0, 0x06, "HptIntMask0"},
        {0xA0, 0x07, "HptIntMask1"},
        {0xA0, 0x08, "HptIntMask2"},
        {0xA0, 0x09, "HptControl"},
        {0xA0, 0x0A, "HptRTI2CPat"},
        {0xA0, 0x0B, "HptRAMPatAdd"},
        {0xA0, 0x0C, "HptProt"},
        {0xA0, 0x0D, "HptUnlock"},
        {0xA0, 0x11, "HPTCfg0"},
        {0xA0, 0x12, "HPTCfg1"},
        {0xA0, 0x13, "HPTCfg2"},
        {0xA0, 0x14, "HPTCfg3"},
        {0xA0, 0x15, "HPTCfg4"},
        {0xA0, 0x16, "HPTCfg5"},
        {0xA0, 0x17, "HPTCfg6"},
        {0xA0, 0x18, "HPTCfg7"},
        {0xA0, 0x19, "HPTCfg8"},
        {0xA0, 0x1A, "HPTCfg9"},
        {0xA0, 0x1B, "HPTCfgA"},
        {0xA0, 0x1C, "HPTCfgB"},
        {0xA0, 0x1D, "HPTCfgC"},
        {0xA0, 0x1E, "HPTCfgD"},
        {0xA0, 0x1F, "HPTCfgE"},
        {0xA0, 0x20, "HPTCfgF"},
        {0xA0, 0x22, "HptAutoTune"},
        {0xA0, 0x23, "BEMFPeriod0"},
        {0xA0, 0x24, "BEMFPeriod1"},
        {0xA0, 0x30, "HptETRGOdAmp"},
        {0xA0, 0x31, "HptETRGOdDur"},
        {0xA0, 0x32, "HptETRGActAmp"},
        {0xA0, 0x33, "HptETRGActDur"},
        {0xA0, 0x34, "HptETRGBrkAmp"},
        {0xA0, 0x35, "HptETRGBrkDur"},
        {0xA0, 0x40, "HptRAMAdd"},
        {0xA0, 0x41, "HptRAMDataH"},
        {0xA0, 0x42, "HptRAMDataM"},
        {0xA0, 0x43, "HptRAMDataL"},
        {0xA0, 0x50, "ADCEn"},
        {0xA0, 0x51, "ADCCfg"},
        {0xA0, 0x53, "ADCDatAvg"},
        {0xA0, 0x54, "ADCDatMin"},
        {0xA0, 0x55, "ADCDatMax"},
        {0x50, 0x00, "ChipID"},
        {0x50, 0x01, "Status0"},
        {0x50, 0x02, "Status1"},
        {0x50, 0x03, "Status2"},
        {0x50, 0x04, "Status3"},
        {0x50, 0x05, "Status4"},
        {0x50, 0x06, "Int0"},
        {0x50, 0x07, "Int1"},
        {0x50, 0x08, "Int2"},
        {0x50, 0x09, "Int3"},
        {0x50, 0x0A, "IntMask0"},
        {0x50, 0x0B, "IntMask1"},
        {0x50, 0x0C, "IntMask2"},
        {0x50, 0x0D, "IntMask3"},
        {0x50, 0x0F, "ILimCntl"},
        {0x50, 0x10, "ChgCntl0"},
        {0x50, 0x11, "ChgCntl1"},
        {0x50, 0x12, "ChgTmr"},
        {0x50, 0x13, "StepChgCfg0"},
        {0x50, 0x14, "StepChgCfg1"},
        {0x50, 0x15, "ThmCfg0"},
        {0x50, 0x16, "ThmCfg1"},
        {0x50, 0x17, "ThmCfg2"},
        {0x50, 0x18, "HrvCfg0"},
        {0x50, 0x19, "HrvCfg1"},
        {0x50, 0x1A, "IVMONCfg"},
        {0x50, 0x1B, "Buck1Ena"},
        {0x50, 0x1C, "Buck1Cfg0"},
        {0x50, 0x1D, "Buck1Cfg1"},
        {0x50, 0x1E, "Buck1Iset"},
        {0x50, 0x1F, "Buck1VSet"},
        {0x50, 0x20, "Buck1Ctr"},
        {0x50, 0x21, "Buck1DvsCfg0"},
        {0x50, 0x22, "Buck1DvsCfg1"},
        {0x50, 0x23, "Buck1DvsCfg2"},
        {0x50, 0x24, "Buck1DvsCfg3"},
        {0x50, 0x25, "Buck1DvsCfg4"},
        {0x50, 0x26, "Buck1DvsSpi"},
        {0x50, 0x27, "Buck2Ena"},
        {0x50, 0x28, "Buck2Cfg"},
        {0x50, 0x29, "Buck2Cfg1"},
        {0x50, 0x2A, "Buck2Iset"},
        {0x50, 0x2B, "Buck2VSet"},
        {0x50, 0x2C, "Buck2Ctr"},
        {0x50, 0x2D, "Buck2DvsCfg0"},
        {0x50, 0x2E, "Buck2DvsCfg1"},
        {0x50, 0x2F, "Buck2DvsCfg2"},
        {0x50, 0x30, "Buck2DvsCfg3"},
        {0x50, 0x31, "Buck2DvsCfg4"},
        {0x50, 0x32, "Buck2DvsSpi"},
        {0x50, 0x34, "Buck3Ena"},
        {0x50, 0x35, "Buck3Cfg"},
        {0x50, 0x36, "Buck3Cfg1"},
        {0x50, 0x37, "Buck3Iset"},
        {0x50, 0x38, "Buck3VSet"},
        {0x50, 0x39, "Buck3Ctr"},
        {0x50, 0x3A, "Buck3DvsCfg0"},
        {0x50, 0x3B, "Buck3DvsCfg1"},
        {0x50, 0x3C, "Buck3DvsCfg2"},
        {0x50, 0x3D, "Buck3DvsCfg3"},
        {0x50, 0x3E, "Buck3DvsCfg4"},
        {0x50, 0x3F, "Buck3DvsSpi"},
        {0x50, 0x40, "BBstEna"},
        {0x50, 0x41, "BBstCfg"},
        {0x50, 0x42, "BBstVSet"},
        {0x50, 0x43, "BBstISet"},
        {0x50, 0x44, "BBstCfg1"},
        {0x50, 0x45, "BBstCtr0"},
        {0x50, 0x46, "BBstCtr1"},
        {0x50, 0x47, "BBstDvsCfg0"},
        {0x50, 0x48, "BBstDvsCfg1"},
        {0x50, 0x49, "BBstDvsCfg2"},
        {0x50, 0x4A, "BBstDvsCfg3"},
        {0x50, 0x4B, "BBstDvsSpi"},
        {0x50, 0x51, "LDO1Ena"},
        {0x50, 0x52, "LDO1Cfg"},
        {0x50, 0x53, "LDO1VSet"},
        {0x50, 0x54, "LDO1Ctr"},
        {0x50, 0x55, "LDO2Ena"},
        {0x50, 0x56, "LDO2Cfg"},
        {0x50, 0x57, "LDO2VSet"},
        {0x50, 0x58, "LDO2Ctr"},
        {0x50, 0x59, "LSW1Ena"},
        {0x50, 0x5A, "LSWCfg"},
        {0x50, 0x5B, "LSW1Ctr"},
        {0x50, 0x5C, "LSW2Ena"},
        {0x50, 0x5D, "LSW2Cfg"},
        {0x50, 0x5E, "LSW2Ctr"},
        {0x50, 0x5F, "ChgPmpEna"},
        {0x50, 0x60, "ChgPmpCfg"},
        {0x50, 0x61, "ChgPmpCtr"},
        {0x50, 0x62, "BoostEna"},
        {0x50, 0x63, "BoostCfg"},
        {0x50, 0x64, "BoostISet"},
        {0x50, 0x65, "BoostVSet"},
        {0x50, 0x66, "BoostCtr"},
        {0x50, 0x67, "MPC0Cfg"},
        {0x50, 0x68, "MPC1Cfg"},
        {0x50, 0x69, "MPC2Cfg"},
        {0x50, 0x6A, "MPC3Cfg"},
        {0x50, 0x6B, "MPC4Cfg"},
        {0x50, 0x6C, "MPC5Cfg"},
        {0x50, 0x6D, "MPC6Cfg"},
        {0x50, 0x6E, "MPC7Cfg"},
        {0x50, 0x6F, "MPCItrSts"},
        {0x50, 0x70, "BK1DedIntCfg"},
        {0x50, 0x71, "BK2DedIntCfg"},
        {0x50, 0x72, "BK3DedIntCfg"},
        {0x50, 0x73, "HptDedIntCfg"},
        {0x50, 0x74, "ADCDedIntCfg"},
        {0x50, 0x75, "USBOkDedIntCfg"},
        {0x50, 0x78, "LEDCommon"},
        {0x50, 0x79, "LED0Ref"},
        {0x50, 0x7A, "LED0Ctr"},
        {0x50, 0x7B, "LED1Ctr"},
        {0x50, 0x7C, "LED2Ctr"},
        {0x50, 0x7D, "PFN"},
        {0x50, 0x7E, "BootCfg"},
        {0x50, 0x7F, "PwrCfg"},
        {0x50, 0x80, "PwrCmd"},
        {0x50, 0x81, "BuckCfg"},
        {0x50, 0x83, "LockMsk"},
        {0x50, 0x84, "LockUnlock"},
        {0x50, 0x86, "SFOUTCtr"},
        {0x50, 0x87, "SFOUTMPC"},
        {0x50, 0x88, "I2C_OTP_ADD"},
        {0x50, 0x89, "I2C_OTP_DAT"},
};



uint8_t max20360_unlock(){
    //{0x50, 0x83, "LockMsk"},
    //{0x50, 0x84, "LockUnlock"},

    // unlock all registers by writing 0x00 to 0x83
    i2c_write_to_pmic_regulators(0x83, 0x00);
    // apply unlock by writing password (0x55) to 0x84 register
    return i2c_write_to_pmic_regulators(0x84, 0x55);

}





uint8_t max20360_read_script(){
    // read all pmic registers
    printk("max20360_read_script()\n");
    printk("slave_address, register_address, register_name, register_data\n");
    uint8_t reg_read_result;
    for (auto & pmic_register : pmic_registers) {
        i2c_reg_read_byte_dt(&max20360regulator_dev, pmic_register.register_address, &reg_read_result);
        printk("0x%02X, 0x%02X, \"%s\", 0x%02X\n",
               pmic_register.slave_address, pmic_register.register_address,
               pmic_register.register_name.c_str(), reg_read_result);
    }
    printk("max20360_read_script() done\n");
    return 0;
}


/**
 * @brief  Program PMIC device with register array
 * @param  register_array: pointer to array of I2C_register_T structs
 * @param  array_size: size of array
 * @retval 0 if successful, 1 if error
 * //todo: implement without array size as an input parameter
*/
//uint8_t config_pmic(i2c_register_t registers[]) {
//    // loop through array and write to device
//    uint8_t error = 0;
//    // todo: use auto keyword.
//    //  q: why does this cause an error
//    for(auto & temp_register : registers){
uint8_t config_pmic(i2c_register_t *register_array, uint8_t array_size){
    uint8_t error = 0;
    for(int i=0; i<array_size; i++){
        error = i2c_write_to_pmic_regulators(register_array[i].register_address, register_array[i].register_data);
        if (error){
            printk("error writing to device programming stopped\n");
            return 1;
        }
    }
    return 0;
}



/**
 * @brief  Calculates bit 1 of fletcher checksum for the MAX20360 PMIC
 * Checksum bit 1: (SLAVE_ID + REG_ADD + DATA) ÷ 255
 * @param  slave_address: slave address of device
 * @param  register_address: register address of device
 * @param  register_data: data to be written to register
 * @retval fletcher checksum bit 1 (uint8_t)
*/
uint8_t calculate_fletcher_checksum_bit_1(uint8_t slave_address, uint8_t register_address, uint8_t register_data) {
    // calculate fletcher checksum bit 1
    return (slave_address + register_address + register_data) % 255;
}


/**
 * @brief  Calculates bit 2 of fletcher checksum for the MAX20360 PMIC
 * Checksum bit 2: ((3x SLAVE_ID) + (2x REG_ADD) + DATA) ÷ 255
 * @param  slave_address: slave address of device
 * @param  register_address: register address of device
 * @param  register_data: data to be written to register
 * @retval fletcher checksum bit 2 (uint8_t)
*/
uint8_t calculate_fletcher_checksum_bit_2(uint8_t slave_address, uint8_t register_address, uint8_t register_data) {
    // calculate fletcher checksum bit 2
    return ((3 * slave_address) + (2 * register_address) + register_data) % 255;
}

uint8_t i2c_write_to_pmic_regulators(uint8_t register_address, uint8_t register_data) {
    return i2c_write_to_dt(&max20360regulator_dev, register_address, register_data);
}

uint8_t  i2c_read_from_pmic_regulators(uint8_t register_address, uint8_t *register_data) {
    return i2c_reg_read_byte_dt(&max20360regulator_dev, register_address, register_data);
}


/**
 * @brief  Writes to MAX20360 PMIC including the 2 bit fletcher checksum
 * @param  slave_address: slave address of device
 * @param  register_address: register address of device
 * @param  register_data: data to be written to register
 * @retval 0 if successful, 1 if error
 * //todo: rename. we are not writing to dt, we are using dt to write to PMIC. consider pmic_write_dt
*/
uint8_t i2c_write_to_dt(const i2c_dt_spec *device, uint8_t register_address, uint8_t register_data) {

    uint8_t data_and_checksum[4];
    uint8_t slave_addr = device->addr << 1;
    data_and_checksum[0] = register_address;
    data_and_checksum[1] = register_data;
    data_and_checksum[2] = calculate_fletcher_checksum_bit_1(slave_addr, register_address, register_data);
    data_and_checksum[3] = calculate_fletcher_checksum_bit_2(slave_addr, register_address, register_data);
    if (i2c_write_dt(device, data_and_checksum, 4)) {
        printk(
                "Error writing to device at address 0x%02X, register 0x%02X with data 0x%02X \n",
                slave_addr, register_address, register_data
        );
        return 1;
    }
    return 0;
}


/**
 * @brief this function sets load switch 1 using HAL GPIO
*/
uint8_t max20360_set_load_switch_1(){
    // todo: abstract which pin controls the LSW and which stm GPIO is connected to the PMIC
    // load switch 1 is on MPC1. set it by setting the pin high

    gpio_pin_set_dt(&max20360regulator_gpio_1, 1);
    return 0;
}

/**
 * @brief this function clears load switch 1 using HAL GPIO
*/
uint8_t max20360_clear_load_switch_1(){
    // todo: abstract which pin controls the LSW and which stm GPIO is connected to the PMIC
    // load switch 1 is on MPC1. clear it by setting the pin low

    gpio_pin_set_dt(&max20360regulator_gpio_1, 0);
    return 0;
}

/**
 * @brief this function sets load switch 2 using HAL GPIO
*/
uint8_t max20360_set_load_switch_2(){
    // todo: abstract which pin controls the LSW and which stm GPIO is connected to the PMIC
    // load switch 2 is on MPC2. set it by setting the pin high

    gpio_pin_set_dt(&max20360regulator_gpio_2, 1);
    return 0;
}

/**
 * @brief this function clears load switch 2 using HAL GPIO
*/
uint8_t max20360_clear_load_switch_2(){
    // todo: abstract which pin controls the LSW and which stm GPIO is connected to the PMIC
    // load switch 2 is on MPC2. clear it by setting the pin low

    gpio_pin_set_dt(&max20360regulator_gpio_2, 0);
    return 0;
}

/**
 * @brief this function sets LDO1 using HAL GPIO
*/
uint8_t max20360_enable_ldo1(){
    // LDO1 is on MPC3. set it by setting the pin high
    //HAL_GPIO_WritePin(MAX20360_LDO1_ENABLE_PORT, MAX20360_LDO1_ENABLE_PIN, GPIO_PIN_SET);
    // write '1' to GPIO pin on main STM MAX20360_LDO1_ENABLE_PORT MAX20360_LDO1_ENABLE_PIN using zephyr:
    //gpio_pin_set(

    return 0;
}

/**
 * @brief this function clears LDO1 using HAL GPIO
*/
uint8_t max20360_disable_ldo1(){
    // LDO1 is on MPC3. clear it by setting the pin low
//    HAL_GPIO_WritePin(MAX20360_LDO1_ENABLE_PORT, MAX20360_LDO1_ENABLE_PIN, GPIO_PIN_RESET);
    return 0;
}

void max20360_config_gpios() {
    gpio_pin_configure_dt(&max20360regulator_gpio_0, GPIO_OUTPUT);
    gpio_pin_configure_dt(&max20360regulator_gpio_1, GPIO_OUTPUT);
    gpio_pin_configure_dt(&max20360regulator_gpio_2, GPIO_OUTPUT);
}




int power_init() {

    uint8_t bbstvset = 0x11;
    uint8_t bbstvset_readback = 0x00;
    uint8_t bbstvset_address = 0x42;

    max20360_config_gpios();
//
//    // read before config
//    i2c_read_from_pmic_regulators(bbstvset_address, &bbstvset_readback);
//    printk("BBstVSet before full pmic config: 0x%02X \n", bbstvset_readback);
//

//    printk("before programming \n");
//    max20360_read_script();

    // config entire PMIC and read
    max20360_unlock();
    config_pmic( pmic_c_to_proto3_reg_values, sizeof(pmic_c_to_proto3_reg_values) / sizeof(pmic_c_to_proto3_reg_values[0]));
    k_sleep(K_MSEC(1000));    // wait for the buck 3 regulator to stabilize (it takes 100ms)
    i2c_read_from_pmic_regulators(bbstvset_address, &bbstvset_readback);
    printk("BBstVSet after full pmic config: 0x%02X \n", bbstvset_readback);
    return 0;
//
//    // manual set
//    i2c_write_to_pmic_regulators(bbstvset_address, bbstvset);    // set BBstVSet to 0x11 (3.3V
//    //k_sleep(K_MSEC(1000));    // wait for the buck 3 regulator to stabilize
//    i2c_read_from_pmic_regulators(bbstvset_address, &bbstvset_readback);
//    printk("BBstVSet after manually setting BBstVSet to 0x11  : 0x%02X \n", bbstvset_readback);

//    printk("after programming \n");
//    max20360_read_script();

    // setup GPIOs
    // normally, there is a yaml file that has the bindings
    //max20360_config_gpios();
//
//    gpio_pin_configure_dt(&max20360regulator_gpio_0, GPIO_OUTPUT);
//    gpio_pin_toggle_dt(&max20360regulator_gpio_0);
//
//
//    gpio_pin_configure_dt(&max20360regulator_gpio_1, GPIO_OUTPUT);
//
//    gpio_pin_configure_dt(&max20360regulator_gpio_2, GPIO_OUTPUT);



//    // toggle gpio forever
//    while (1) {
//
//        k_sleep(K_MSEC(2000));
//        gpio_pin_toggle_dt(&max20360regulator_gpio_1);
//        gpio_pin_toggle_dt(&max20360regulator_gpio_2);



//        // improper implementation all in main:
//        k_sleep(K_MSEC(1000));
//        gpio_pin_set(pmic_gpio_device_2, PMIC_PIN_2, 1);
//        k_sleep(K_MSEC(1000));
//        gpio_pin_set(pmic_gpio_device_2, PMIC_PIN_2, 0);


//        // proper implementation using methods:
//        k_sleep(K_MSEC(1000));
//        max20360_set_load_switch_1();
//        max20360_clear_load_switch_2();
//        k_sleep(K_MSEC(1000));
//        max20360_clear_load_switch_1();
//        max20360_set_load_switch_2();
}
