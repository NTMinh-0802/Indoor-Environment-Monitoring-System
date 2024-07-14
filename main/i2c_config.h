/**
 * @file       i2c_config.h
 * @copyright  Copyright (C) 2024. All rights reserved.
 * @license    This project is released under Nguyen Thanh Minh.
 * @version    1.0.0
 * @date       2024-07-14
 * @author     Nguyen Thanh Minh
 *             
 * @brief      I2C configuration for ESP-IDF
 *             
 * @note       Not
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __I2C_CONFIG_H
#define __I2C_CONFIG_H

/* Includes ----------------------------------------------------------- */

/* Public defines ----------------------------------------------------- */
#define I2C_MASTER_RX_BUF_DISABLE   (0)
#define I2C_MASTER_TX_BUF_DISABLE   (0)
#define I2C_MASTER_FREQ_HZ          (100000)
#define I2C_MASTER_TIMEOUT_MS       (1000)
#define I2C_MASTER_NUM              (0)

/* Public enumerate/structure ----------------------------------------- */

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */

#endif // __I2C_CONFIG_H

/* End of file -------------------------------------------------------- */