/**
 * @file       publish_data.c
 * @copyright  Copyright (C) 2024. All rights reserved.
 * @license    This project is released under Nguyen Thanh Minh.
 * @version    1.0.0
 * @date       2024-07-14
 * @author     Nguyen Thanh Minh
 *             
 * @brief      Data publishing module for MQTT
 *             
 * @note       Tested with ESP-IDF  
 */
/* Includes ----------------------------------------------------------- */
#include "publish_data.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include "connect_mqtt.h"

/* Private defines ---------------------------------------------------- */

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables --------------------------------------------------- */
static const char *TAG = "PUBLISH_DATA";

/* Private function prototypes ---------------------------------------- */
extern esp_mqtt_client_handle_t get_mqtt_client(void);

/* Function definitions ----------------------------------------------- */
void publish_sensor_data(float temperature, float humidity, float light)
{
    esp_mqtt_client_handle_t mqtt_client = get_mqtt_client();
    ESP_LOGI(TAG, "mqtt_client: %p, is_mqtt_connected: %d", mqtt_client, is_mqtt_connected());

    if (mqtt_client == NULL || !is_mqtt_connected()) 
    {
        ESP_LOGE(TAG, "MQTT client is not initialized or not connected");
        return;
    }

    char json_data[150];
    snprintf(json_data, sizeof(json_data), 
             "{\"temperature_indoor\": %.2f, \"humidity_indoor\": %.2f, \"light_indoor\": %.2f}", 
             temperature, humidity, light);

    int msg_id = esp_mqtt_client_publish(mqtt_client, "v1/devices/me/telemetry", json_data, 0, 1, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish sensor data");
    } else {
        ESP_LOGI(TAG, "Published sensor data, msg_id=%d", msg_id);
    }
}

/* End of file -------------------------------------------------------- */