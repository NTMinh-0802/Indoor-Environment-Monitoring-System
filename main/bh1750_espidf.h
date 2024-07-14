/**
 * @file       bh1750_espidf.h
 * @copyright  Copyright (C) 2024. All rights reserved.
 * @license    This project is released under Nguyen Thanh Minh.
 * @version    1.0.0
 * @date       2024-07-14
 * @author     Nguyen Thanh Minh
 *             
 * @brief      driver for BH1750
 *             
 * @note       Tested with ESP-IDF  
 */
/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BH1750_ESPIDF_H
#define __BH1750_ESPIDF_H
/* Includes ----------------------------------------------------------- */
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/task.h"
#include "i2c_config.h"
/* Public defines ----------------------------------------------------- */
#define I2C_MASTER_SCL_IO               GPIO_NUM_22             // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO               GPIO_NUM_21             // GPIO number for I2C master data

#define WRITE_BIT                       I2C_MASTER_WRITE        // I2C master write
#define READ_BIT                        I2C_MASTER_READ         // I2C master read
#define ACK_CHECK_EN                    0x1                     // I2C master will check ack from slave
#define ACK_CHECK_DIS                   0x0                     // I2C master will not check ack from slave
#define ACK_VAL                         0x0                     // I2C ack value
#define NACK_VAL                        0x1                     // I2C nack value

#define BH1750_SLAVE_ADDR               0x23    // Slave address
#define BH1750_PWR_DOWN                 0x00    // Power down the module
#define BH1750_PWR_ON                   0x01    // Power on the module, wait for measurement instruction
#define BH1750_RST                      0x07    // Reset data register value, valid in PowerOn mode
#define BH1750_CON_H                    0x10    // Continuous high resolution mode, 1lx, 120ms
#define BH1750_CON_H2                   0x11    // Continuous high resolution mode, 0.5lx, 120ms
#define BH1750_CON_L                    0x13    // Continuous low resolution mode, 4lx, 16ms
#define BH1750_ONE_H                    0x20    // One-time high resolution mode, 1lx, 120ms, module switches to PowerDown after measurement
#define BH1750_ONE_H2                   0x21    // One-time high resolution mode, 0.5lx, 120ms, module switches to PowerDown after measurement
#define BH1750_ONE_L                    0x23    // One-time low resolution mode, 4lx, 16ms, module switches to PowerDown after measurement
/* Public enumerate/structure ----------------------------------------- */

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
/**
 @brief Initialize I2C driver
 @param None
 @return None
*/
int I2C_Init(void);

/**
 @brief I2C write data function
 @param slaveAddr -[in] Slave device address
 @param regAddr -[in] Register address
 @param pData -[in] Data to write
 @param dataLen -[in] Length of data to write
 @return Error code
*/
int I2C_WriteData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen);

/**
 @brief I2C read data function
 @param slaveAddr -[in] Slave device address
 @param regAddr -[in] Register address
 @param pData -[in] Data to read
 @param dataLen -[in] Length of data to read
 @return Error code
*/
int I2C_ReadData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen);

/**
 @brief BH1750 initialization function
 @param None
 @return None
*/
void BH1750_Init(void);

/**
 @brief Get light intensity from BH1750
 @param None
 @return Light intensity
*/
float BH1750_ReadLightIntensity(void);

#endif // __BH1750_ESPIDF_H

/* End of file -------------------------------------------------------- */