// драйвер для vl53l0x

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "VL53L0X_driver.h"

static const char *TAG_VL = "VL53L0X Driver";

const vl53l0x_t vl_default = {
    .value=0,
    .xshut=0,
    .addr=0x29
};

vl53l0x_t vl1 = vl_default;
vl53l0x_t vl2 = vl_default;
vl53l0x_t vl3 = vl_default;
vl53l0x_t vl4 = vl_default;

void vl53l0x_xshut_attach(vl53l0x_t * vl, uint8_t pin)
{
    gpio_reset_pin(pin);
    gpio_set_pull_mode(pin, GPIO_FLOATING);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
    vl->xshut = pin;
}

void vl53l0x_on(vl53l0x_t * vl)
{
    gpio_set_level(vl->xshut, 1);
}

void vl53l0x_off(vl53l0x_t * vl)
{
    gpio_set_level(vl->xshut, 0);
}

esp_err_t vl53l0x_write_byte(vl53l0x_t * vl, uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (vl->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t vl53l0x_read_byte(vl53l0x_t * vl, uint8_t reg, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (vl->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (vl->addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t vl53l0x_init(vl53l0x_t * vl)
{
    uint8_t data;
    esp_err_t ret;

    // Check if the sensor is responding
    ret = vl53l0x_read_byte(vl, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &data);
    if (ret != ESP_OK || data != 0xEE) {
        ESP_LOGE(TAG_VL, "VL53L0X not responding, ID: 0x%x", data);
        return ESP_FAIL;
    }
    else {
        ESP_LOGI(TAG_VL, "VL53L0X on gpio %d initialized successfully.", vl->xshut);
    }
    
    return ESP_OK;
}

void vl53l0x_setaddress(vl53l0x_t * vl, uint8_t new_addr)
{
    vl53l0x_write_byte(vl, VL53L0X_REG_I2C_ADDRESS, new_addr & 0x7F);
    vl->addr = new_addr;
}

void vl53l0x_measure( void *arg )
{
    uint16_t distance;
    uint8_t distance_data[2];
    vl53l0x_xshut_attach(&vl1, 6);
    vl53l0x_xshut_attach(&vl2, 7);
    vl53l0x_xshut_attach(&vl3, 3);
    vl53l0x_xshut_attach(&vl4, 46);
    vTaskDelay(pdMS_TO_TICKS(10));

    vl53l0x_off(&vl1);
    vl53l0x_off(&vl2);
    vl53l0x_off(&vl3);
    vl53l0x_off(&vl4);
    vTaskDelay(pdMS_TO_TICKS(10));

    vl53l0x_on(&vl1);
    vl53l0x_on(&vl2);
    vl53l0x_on(&vl3);
    vl53l0x_on(&vl4);
    vTaskDelay(pdMS_TO_TICKS(10));

    vl53l0x_on(&vl1);
    vl53l0x_off(&vl2);
    vl53l0x_off(&vl3);
    vl53l0x_off(&vl4);
    vTaskDelay(pdMS_TO_TICKS(10));

    vl53l0x_setaddress(&vl1, 0x2A);
    vl53l0x_init(&vl1);
    vTaskDelay(pdMS_TO_TICKS(10));

    vl53l0x_on(&vl2);
    vTaskDelay(pdMS_TO_TICKS(10));
    vl53l0x_setaddress(&vl2, 0x2B);
    vl53l0x_init(&vl2);
    vTaskDelay(pdMS_TO_TICKS(10));

    vl53l0x_on(&vl3);
    vTaskDelay(pdMS_TO_TICKS(10));
    vl53l0x_setaddress(&vl3, 0x2C);
    vl53l0x_init(&vl3);
    vTaskDelay(pdMS_TO_TICKS(10));

    vl53l0x_on(&vl4);
    vTaskDelay(pdMS_TO_TICKS(10));
    vl53l0x_setaddress(&vl4, 0x2D);
    vl53l0x_init(&vl4);
    vTaskDelay(pdMS_TO_TICKS(10));

    while (1)
    {
        // Start measurement
        vl53l0x_write_byte(&vl1, VL53L0X_REG_SYSRANGE_START, 0x01);
        vTaskDelay(pdMS_TO_TICKS(10));
        vl53l0x_read_byte(&vl1, VL53L0X_REG_RESULT_RANGE_MEASURE, &distance_data[0]);
        vl53l0x_read_byte(&vl1, VL53L0X_REG_RESULT_RANGE_MEASURE + 1, &distance_data[1]);
        distance = (distance_data[0] << 8) | distance_data[1];
        if(distance == 20 || distance > 1100)distance = 8191;
        vl1.value = distance;
        vTaskDelay(pdMS_TO_TICKS(10));

        vl53l0x_write_byte(&vl2, VL53L0X_REG_SYSRANGE_START, 0x01);
        vTaskDelay(pdMS_TO_TICKS(10));
        vl53l0x_read_byte(&vl2, VL53L0X_REG_RESULT_RANGE_MEASURE, &distance_data[0]);
        vl53l0x_read_byte(&vl2, VL53L0X_REG_RESULT_RANGE_MEASURE + 1, &distance_data[1]);
        distance = (distance_data[0] << 8) | distance_data[1];
        if(distance == 20 || distance > 1100)distance = 8191;
        vl2.value = distance;

        vl53l0x_write_byte(&vl3, VL53L0X_REG_SYSRANGE_START, 0x01);
        vTaskDelay(pdMS_TO_TICKS(10));
        vl53l0x_read_byte(&vl3, VL53L0X_REG_RESULT_RANGE_MEASURE, &distance_data[0]);
        vl53l0x_read_byte(&vl3, VL53L0X_REG_RESULT_RANGE_MEASURE + 1, &distance_data[1]);
        distance = (distance_data[0] << 8) | distance_data[1];
        if(distance == 20 || distance > 1100)distance = 8191;
        vl3.value = distance;

        vl53l0x_write_byte(&vl4, VL53L0X_REG_SYSRANGE_START, 0x01);
        vTaskDelay(pdMS_TO_TICKS(10));
        vl53l0x_read_byte(&vl4, VL53L0X_REG_RESULT_RANGE_MEASURE, &distance_data[0]);
        vl53l0x_read_byte(&vl4, VL53L0X_REG_RESULT_RANGE_MEASURE + 1, &distance_data[1]);
        distance = (distance_data[0] << 8) | distance_data[1];
        if(distance == 20 || distance > 1100)distance = 8191;
        vl4.value = distance;

        ESP_LOGI(TAG_VL, "%d, %d, %d, %d", vl1.value, vl2.value, vl3.value, vl4.value);

        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 1 second
    }
}

