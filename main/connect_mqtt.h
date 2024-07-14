/**
 * @file       connect_mqtt.h
 * @copyright  Copyright (C) 2024. All rights reserved.
 * @license    This project is released under Nguyen Thanh Minh.
 * @version    1.0.0
 * @date       2024-07-14
 * @author     Nguyen Thanh Minh
 *             
 * @brief      MQTT connection and communication handler for ESP-IDF
 *             
 * @note       Tested with ESP-IDF
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __CONNECT_MQTT_H
#define __CONNECT_MQTT_H

/* Includes ----------------------------------------------------------- */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

/* Public defines ----------------------------------------------------- */
#define MQTT_BROKER_URI "mqtt://thingsboard.srv524038.hstgr.cloud/"
#define MQTT_BROKER_PORT 1884
#define MQTT_USERNAME "hAtOjp2kejMeaJMFfpQE"
#define MQTT_KEEPALIVE 120

/* Public enumerate/structure ----------------------------------------- */

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */
extern esp_mqtt_client_handle_t mqtt_client;

/* Public function prototypes ----------------------------------------- */
#ifdef __cplusplus
extern "C" 
{
#endif

/**
 * @brief Log error message if error code is non-zero
 * @param message Error message
 * @param error_code Error code
 * @return None
 */
void log_error_if_nonzero(const char *message, int error_code);

/**
 * @brief MQTT event handler
 * @param handler_args Handler arguments
 * @param base Event base
 * @param event_id Event ID
 * @param event_data Event data
 * @return None
 */
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

/**
 * @brief Start MQTT application
 * @param None
 * @return None
 */
void mqtt_app_start(void);

/**
 * @brief Initialize and start MQTT connection
 * @param None
 * @return None
 */
void init_start_mqtt(void);

/**
 * @brief Check if MQTT is connected
 * @param None
 * @return true if connected, false otherwise
 */
bool is_mqtt_connected(void);

/**
 * @brief Get MQTT client handle
 * @param None
 * @return MQTT client handle
 */
esp_mqtt_client_handle_t get_mqtt_client(void);

#ifdef __cplusplus
}
#endif

#endif // __CONNECT_MQTT_H

/* End of file -------------------------------------------------------- */