/**
 * @file       bh1750_espidf.c
 * @copyright  Copyright (C) 2024. All rights reserved.
 * @license    This project is released under Nguyen Thanh Minh.
 * @version    1.0.0
 * @date       2024-07-14
 * @author     Nguyen Thanh Minh
 *             
 * @brief      driver for BH1750
 *             
 * @note       not tested
 */
/* Includes ----------------------------------------------------------- */
#include "bh1750_espidf.h"
/* Private defines ---------------------------------------------------- */

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */
int i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


int i2c_write_data(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data_read, uint16_t data_leng)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    if (reg_addr != 0)
    {
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    }
    i2c_master_write(cmd, data_read, data_leng, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int i2c_read_data(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data_read, uint16_t data_leng)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | READ_BIT, ACK_CHECK_EN);
    if (reg_addr != 0)
    {
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    }
    i2c_master_read(cmd, data_read, data_leng, ACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void bh1750_init(void)
{
    uint8_t data;
    data = BH1750_PWR_ON;
    ESP_ERROR_CHECK(i2c_write_data(BH1750_SLAVE_ADDR, 0, &data, 1));
    vTaskDelay(pdMS_TO_TICKS(10)); // Đợi 10ms

    data = BH1750_CON_H;
    ESP_ERROR_CHECK(i2c_write_data(BH1750_SLAVE_ADDR, 0, &data, 1));
    vTaskDelay(pdMS_TO_TICKS(180)); // Đợi 180ms cho cảm biến ổn định
}

float read_light_intensity(void)
{
    float lux = 0.0;
    uint8_t sensorData[2] = {0};
    i2c_read_data(BH1750_SLAVE_ADDR, 0, sensorData, 2);
    lux = (sensorData[0] << 8 | sensorData[1]) / 1.2;
    return lux;
}
/* End of file -------------------------------------------------------- */