/**
 * @file       sht30_espidf.h
 * @copyright  Copyright (C) 2024. All rights reserved.
 * @license    This project is released under Nguyen Thanh Minh.
 * @version    1.0.0
 * @date       2024-07-14
 * @author     Nguyen Thanh Minh
 *             
 * @brief      SHT30 sensor driver for ESP-IDF
 *             
 * @note       Tested with ESP-IDF
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __SHT30_ESPIDF_H
#define __SHT30_ESPIDF_H

/* Includes ----------------------------------------------------------- */
#include <stdio.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "i2c_config.h"

/* Public defines ----------------------------------------------------- */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000

#define I2C_MASTER_SCL_IO               GPIO_NUM_22               // gpio number for I2C master clock
#define I2C_MASTER_SDA_IO               GPIO_NUM_21               // gpio number for I2C master data 

#define SHT30_SENSOR_ADDR               0x44
#define WRITE_BIT                       I2C_MASTER_WRITE          // I2C master write
#define READ_BIT                        I2C_MASTER_READ           // I2C master read
#define ACK_CHECK_EN                    0x1                       // I2C master will check ack from slav
#define ACK_CHECK_DIS                   0x0                       // I2C master will not check ack from slave
#define ACK_VAL                         0x0                       // I2C ack value
#define NACK_VAL                        0x1                       // I2C nack value

/* Public enumerate/structure ----------------------------------------- */

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
/**
 * @brief Initialize I2C master
 * @param None
 * @return esp_err_t
 */
esp_err_t i2c_master_init(void);

/**
 * @brief Initialize SHT30 sensor
 * @param i2c_num I2C port number
 * @return esp_err_t
 */
esp_err_t i2c_master_sensor_sht30_init(i2c_port_t i2c_num);

/**
 * @brief Read temperature and humidity from SHT30 sensor
 * @param i2c_num I2C port number
 * @param temp_data Pointer to store temperature data
 * @param hum_data Pointer to store humidity data
 * @return esp_err_t
 */
esp_err_t i2c_master_sensor_sht30_read(i2c_port_t i2c_num, float *temp_data, float *hum_data);

/**
 * @brief Task to read SHT30 sensor data
 * @param arg Task argument
 * @return None
 */
void i2c_sht30_task(void *arg);

#endif // __SHT30_ESPIDF_H

/* End of file -------------------------------------------------------- */