/**
 * @file       sht30_espidf.c
 * @copyright  Copyright (C) 2024. All rights reserved.
 * @license    This project is released under Nguyen Thanh Minh.
 * @version    1.0.0
 * @date       2024-07-14
 * @author     Nguyen Thanh Minh
 *             
 * @brief      Driver for SHT30
 *             
 * @note       
 */
/* Includes ----------------------------------------------------------- */
#include "sht30_espidf.h"

/* Private defines ---------------------------------------------------- */
static const char *TAG = "SHT30";
const uint8_t g_polynom = 0x31;

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */
static uint8_t crc8(uint8_t data[], int len);

/* Function definitions ----------------------------------------------- */
static uint8_t crc8(uint8_t data[], int len)
{
    uint8_t crc = 0xff;
    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];  
    
        for (int j = 0; j < 8; j++)
        {
            bool xor = crc & 0x80;
            crc = crc << 1;
            crc = xor ? crc ^ g_polynom : crc;
        }
    }
    return crc;
} 

esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter configuration error");
        return err;
    }
   
    ESP_LOGI(TAG, "Installing I2C driver");
    err = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation error");
    }
    
    return err;
}

esp_err_t i2c_master_sensor_sht30_init(i2c_port_t i2c_num)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT30_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_DIS);
    i2c_master_write_byte(cmd, 0xF3, ACK_CHECK_DIS);
    i2c_master_write_byte(cmd, 0x2D, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(30 / portTICK_PERIOD_MS);

    uint8_t data[3];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT30_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_DIS);
    i2c_master_read(cmd, data, sizeof(data), ACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    if (crc8(data, 2) != data[2])
    {
        ESP_LOGE(TAG, "CRC check failed");
        return ESP_FAIL;
    }
    return ret;
}

esp_err_t i2c_master_sensor_sht30_read(i2c_port_t i2c_num, float *temp_data, float *hum_data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT30_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x24, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 2000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error writing command to SHT30");
        return ret;
    }
    vTaskDelay(30 / portTICK_PERIOD_MS);    

    uint8_t data[6];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT30_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, sizeof(data) - 1, ACK_VAL);
    i2c_master_read_byte(cmd, &data[5], NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 5000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading data from SHT30");
        return ret;
    }

    if (crc8(data, 2) != data[2] || crc8(data + 3, 2) != data[5])
    {
        ESP_LOGE(TAG, "CRC check failed for SHT30 data");
        return ESP_FAIL;
    }

    *temp_data = ((((data[0] * 256.0) + data[1]) * 175) / 65535.0) - 45;
    *hum_data = ((((data[3] * 256.0) + data[4]) * 100) / 65535.0);

    return ESP_OK;
}

void i2c_sht30_task(void *arg)
{
    int ret;
    uint32_t task_idx = (uint32_t)arg;
    float temp_data, hum_data;
    
    int cnt = 0;
    while (1) {
        ESP_LOGI(TAG, "TASK[%ld] test cnt: %d", task_idx, cnt++);
        ret = i2c_master_sensor_sht30_read(I2C_MASTER_NUM, &temp_data, &hum_data);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.1f%%", temp_data, hum_data);
        } else {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }
        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

/* End of file -------------------------------------------------------- */