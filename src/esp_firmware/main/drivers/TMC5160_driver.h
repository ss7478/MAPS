// драйвер для tmc5160 с spi

#include "freertos/FreeRTOS.h" 
#include "freertos/task.h" 
#include "driver/spi_master.h" 
#include "esp_log.h" 
#include "TMC5160_reg.h"
#include "unistd.h"
#include "math.h"

#ifndef PIN_NUM_CLK
    #define min(a, b) ((a) < (b) ? a : b)

    #define PIN_NUM_MISO 13 
    #define PIN_NUM_MOSI 11 
    #define PIN_NUM_CLK  12 
    #define PIN_NUM_CS1   4
    #define PIN_NUM_CS2   5

    void tmc5160_write_register(spi_device_handle_t spi, uint8_t address, uint32_t value);
    uint32_t tmc5160_read_register(spi_device_handle_t spi, uint8_t address);
    int32_t accelFromHz(double accelHz);
    int32_t speedFromHz(float speed);
    void setMaxSpeed(float speed, spi_device_handle_t spi);
    void setAcceleration(float maxAccel, spi_device_handle_t spi);
    void tmc5160_init(spi_device_handle_t spi);
#endif