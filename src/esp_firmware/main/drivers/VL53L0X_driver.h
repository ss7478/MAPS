// драйвер для vl53l0x

#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID 0xC0
#define VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO 0x0A
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR 0x0B
#define VL53L0X_REG_SYSRANGE_START 0x00
#define VL53L0X_REG_RESULT_RANGE_STATUS 0x14
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS 0x13
#define VL53L0X_REG_RESULT_RANGE_MEASURE 0x1E
#define VL53L0X_REG_I2C_ADDRESS 0x8A

#define I2C_MASTER_NUM I2C_NUM_1

typedef struct 
{
    int value;
    uint8_t xshut;
    uint8_t addr;
}   vl53l0x_t;

extern const vl53l0x_t vl_default;
extern vl53l0x_t vl1;
extern vl53l0x_t vl2;
extern vl53l0x_t vl3;
extern vl53l0x_t vl4;


void vl53l0x_xshut_attach(vl53l0x_t * vl, uint8_t pin);
void vl53l0x_on(vl53l0x_t * vl);
void vl53l0x_off(vl53l0x_t * vl);
esp_err_t vl53l0x_write_byte(vl53l0x_t * vl, uint8_t reg, uint8_t data);
esp_err_t vl53l0x_read_byte(vl53l0x_t * vl, uint8_t reg, uint8_t *data);
esp_err_t vl53l0x_init(vl53l0x_t * vl);
void vl53l0x_setaddress(vl53l0x_t * vl, uint8_t new_addr);
void vl53l0x_init_all( void );
void vl53l0x_measure( void *arg );