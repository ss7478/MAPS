// драйвер для tmc5160 с spi

#include "freertos/FreeRTOS.h" 
#include "freertos/task.h" 
#include "driver/spi_master.h" 
#include "esp_log.h" 
#include "TMC5160_reg.h"
#include "unistd.h"
#include "math.h"
#include "TMC5160_driver.h"

const char* TAG = "MAIN"; 
esp_err_t ret; 

void tmc5160_write_register(spi_device_handle_t spi, uint8_t address, uint32_t value) { 
    uint8_t tx_data[5]; 
    tx_data[0] = address | 0x80; // Set the write bit 
    tx_data[1] = (value >> 24) & 0xFF; 
    tx_data[2] = (value >> 16) & 0xFF; 
    tx_data[3] = (value >> 8) & 0xFF; 
    tx_data[4] = value & 0xFF; 

    spi_transaction_t transaction = { 
        .length = 40, // 5 bytes * 8 bits 
        .tx_buffer = tx_data 
    }; 

    ret = spi_device_transmit(spi, &transaction); 
    if (ret != ESP_OK) { 
        ESP_LOGE(TAG, "Failed to write register 0x%02X", address); 
    } 
} 

uint32_t tmc5160_read_register(spi_device_handle_t spi, uint8_t address) { 
    uint8_t tx_data[5] = {0}; 
    uint8_t rx_data[5] = {0}; 
    tx_data[0] = address & 0x7F; // Clear the write bit to read 

    spi_transaction_t transaction = { 
        .length = 40, // 5 bytes * 8 bits 
        .tx_buffer = tx_data, 
        .rx_buffer = rx_data 
    }; 

    ret = spi_device_transmit(spi, &transaction); 
    usleep(1);
    ret = spi_device_transmit(spi, &transaction);
    if (ret != ESP_OK) { 
        ESP_LOGE(TAG, "Failed to read register 0x%02X", address); 
        return 0; 
    } 

    uint32_t value = (rx_data[1] << 24) | (rx_data[2] << 16) | (rx_data[3] << 8) | rx_data[4]; 
    return value; 
} 

int32_t accelFromHz(double accelHz)
{
    return (int32_t)(accelHz / ((double)12000000.0 / (double)131072.0 / (double)16777216.0) * (double)256);
}

int32_t speedFromHz(float speed)
{
    return (int32_t)(speed / 0.7152 * 256);
}

void setMaxSpeed(float speed, spi_device_handle_t spi)
{
    tmc5160_write_register(spi, VMAX, min(0x7FFFFF, speedFromHz(fabs(speed))));
    tmc5160_write_register(spi, RAMPMODE, speed < 0.0f ? 0x02 : 0x01);   //0x01 is positive velocity mode 0x02 is negative velocity mode
}

void setAcceleration(float maxAccel, spi_device_handle_t spi)
{
    tmc5160_write_register(spi, AMAX, min(0xFFFF, accelFromHz(fabs(maxAccel))));
    tmc5160_write_register(spi, DMAX, min(0xFFFF, accelFromHz(fabs(maxAccel))));
}

void tmc5160_init(spi_device_handle_t spi)
{
    tmc5160_write_register(spi, GSTAT, 0x00000005);
    tmc5160_write_register(spi, DRV_CONF, 0x00040200);
    tmc5160_write_register(spi, GLOBAL_SCALER, 0x00000035); 
/*
    1. 0x...64 = 100 = 0.85 покоя ; 4 макс при езде A 
    2. 0x...55 = 85  = 0.63 покоя ; 3.23 макс при езде А после 10 проездов теплые, 39-40 C
    3. 0x...45 = 69  = 0.47 покоя ; 1.3 макс при езде А - начались небольшие пропуски на сменах направления 
    4. 0x...35 = 53  = 0.19 покоя ; 0.8 макс при езде А - пропуски на сменах напрявления стали серьезными
    5. 0x...50 = 80  = 
*/

    tmc5160_write_register(spi, IHOLD_IRUN, 0x000E1F10); 
    tmc5160_write_register(spi, PWMCONF, 0b11000010000010100000000000111000);
    tmc5160_write_register(spi, PWMCONF, 0b11000010000011100000000000111000);
    tmc5160_write_register(spi, CHOPCONF, 0x0000801A);

    setMaxSpeed(0, spi);   //defines speed 0 and sets the ramp mode to velocity_mode_pos
    tmc5160_write_register(spi, GCONF, 0x00000004);

    tmc5160_write_register(spi, VSTART, min(0x3FFFF, speedFromHz(fabs(0.0))));
    tmc5160_write_register(spi, VSTOP, min(0x3FFFF, speedFromHz(fabs(0.1))));
    tmc5160_write_register(spi, V_1, min(0xFFFFF, speedFromHz(fabs(0.0))));

    tmc5160_write_register(spi, D_1, 100);

    ESP_LOGI(TAG, "all written");
}