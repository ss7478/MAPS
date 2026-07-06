// драйвер для bno055 (в основном написан самостоятельно)

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "bno_driver.h"


bno_data_struct bno_data;
bno_calib_struct bno_calib_values;

bool calib_flag = false;

double kalmanangle = 0;
double kalmanuncertainityangle = 4;

kalman_data x_kalman_data;
kalman_data y_kalman_data;

// запись 1 байта в регистр
esp_err_t bno055_write_register(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BNO055_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// чтение 1 байта из регистра
esp_err_t bno055_read_register(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BNO055_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BNO055_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// инициализация bno055
void bno055_init(void)
{
    bno055_write_register(BNO055_PWR_MODE_ADDR, 0x00); // установка режима питания
    bno055_write_register(BNO055_OPR_MODE_ADDR, 0x0C); // установка режима работы imu во "fusion"
}

// считывание данных с датчика - акселерометр, гироскоп и температура
void bno055_read_data(void)
{
    uint8_t accel_data[6];
    uint8_t gyro_data[6];
    uint8_t mag_data[6];
    uint8_t temp_data;

    bno055_read_register(BNO055_ACCEL_DATA_X_LSB, accel_data, 6);
    bno055_read_register(BNO055_GYRO_DATA_X_LSB, gyro_data, 6);
    bno055_read_register(BNO055_MAG_DATA_X_LSB, mag_data, 6);
    bno055_read_register(BNO055_TEMP_ADDR, &temp_data, 1);

    int16_t accel_x = (accel_data[1] << 8) | accel_data[0];
    int16_t accel_y = (accel_data[3] << 8) | accel_data[2];
    int16_t accel_z = (accel_data[5] << 8) | accel_data[4];

    int16_t gyro_x = (gyro_data[1] << 8) | gyro_data[0];
    int16_t gyro_y = (gyro_data[3] << 8) | gyro_data[2];
    int16_t gyro_z = (gyro_data[5] << 8) | gyro_data[4];
    int16_t mag_x = (mag_data[1] << 8) | mag_data[0];
    int16_t mag_y = (mag_data[3] << 8) | mag_data[2];
    int16_t mag_z = (mag_data[5] << 8) | mag_data[4];

    bno_data.acc_x = (double)accel_x * 0.01;
    bno_data.acc_y = (double)accel_y * 0.01;
    bno_data.acc_z = (double)accel_z * 0.01;
    bno_data.gy_x = (double)gyro_x / 16 - bno_calib_values.gy_cal_x;
    bno_data.gy_y = (double)gyro_y / 16 - bno_calib_values.gy_cal_y;
    bno_data.gy_z = (double)gyro_z / 16 - bno_calib_values.gy_cal_z;
    bno_data.mag_x = (double)mag_x / 16;
    bno_data.mag_y = (double)mag_y / 16;
    bno_data.mag_z = (double)mag_z / 16;
    bno_data.temp = temp_data;

    bno_data.acc_angle_x = atan2(bno_data.acc_y, bno_data.acc_z) * RADIANS_TO_DEGREES;
    if(bno_data.acc_angle_x < -90)bno_data.acc_angle_x += 180;
    else bno_data.acc_angle_x -= 180;
    bno_data.acc_angle_y = atan2(bno_data.acc_x, bno_data.acc_z) * RADIANS_TO_DEGREES;
    
    kalman(&y_kalman_data, -bno_data.gy_y, bno_data.acc_angle_y);
    bno_data.kalman_y = y_kalman_data.kalman_angle;
    kalman(&x_kalman_data, bno_data.gy_x, bno_data.acc_angle_x);
    bno_data.kalman_x = x_kalman_data.kalman_angle;
}

// фильтр Калмана
void kalman(kalman_data *data, double input, double measurement)
{
    data->kalman_angle += input * 0.007;
    data->kalman_uncert_angle += 0.007 * 0.007 * 4 * 4;
    double gain = data->kalman_uncert_angle * 1 / (1 * data->kalman_uncert_angle + 9);
    data->kalman_angle += gain * (measurement - data->kalman_angle);
    data->kalman_uncert_angle *= (1 - gain);
}

// калибровка гироскопа
void bno055_calib(void)
{
    printf("BNO055 is calibrating.");
    for(int i = 0; i < 100; i++)
    {
        bno055_read_data();
        bno_calib_values.gy_cal_x += bno_data.gy_x;
        bno_calib_values.gy_cal_y += bno_data.gy_y;
        bno_calib_values.gy_cal_z += bno_data.gy_z;
        printf(".");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    bno_calib_values.gy_cal_x /= 100;
    bno_calib_values.gy_cal_y /= 100;
    bno_calib_values.gy_cal_z /= 100;
    printf("\nBNO055 calibrated successfully!\nCalibration values:\nX: %.3f Y: %.3f Z: %.3f\n\n\n\n\n", bno_calib_values.gy_cal_x, bno_calib_values.gy_cal_y, bno_calib_values.gy_cal_z);
    vTaskDelay(pdMS_TO_TICKS(1000));
    calib_flag = true;
}

bool bno_is_calibrated(void)
{
    return calib_flag;
}

bno_data_struct bno_get_data()
{
    return bno_data;
}

// задача для постоянного считывания показаний датчика в фоне
void bno055_task(void *arg)
{
    esp_task_wdt_add(NULL);
    bno055_init();
    bno055_calib();
    while (1)
    {
        bno055_read_data();

        esp_task_wdt_reset();
        uint64_t _bno_task_timer = esp_timer_get_time();
        vTaskDelay(pdMS_TO_TICKS(3));
        while(esp_timer_get_time() - _bno_task_timer < 4000){}
    }
}
