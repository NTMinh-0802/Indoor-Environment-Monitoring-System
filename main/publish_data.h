/**
 * @file       publish_data.h
 * @copyright  Copyright (C) 2024. All rights reserved.
 * @license    This project is released under Nguyen Thanh Minh.
 * @version    1.0.0
 * @date       2024-07-14
 * @author     Nguyen Thanh Minh
 *             
 * @brief      Data publishing handler for ESP-IDF
 *             
 * @note       Tested with ESP-IDF
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __PUBLISH_DATA_H
#define __PUBLISH_DATA_H

/* Includes ----------------------------------------------------------- */
#include "esp_err.h"
#include "mqtt_client.h"

/* Public defines ----------------------------------------------------- */

/* Public enumerate/structure ----------------------------------------- */

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
/**
 * @brief Publish sensor data to MQTT broker
 * @param temperature Temperature value
 * @param humidity Humidity value
 * @param light Light intensity value
 * @return None
 */
void publish_sensor_data(float temperature, float humidity, float light);

#endif // __PUBLISH_DATA_H

/* End of file -------------------------------------------------------- */