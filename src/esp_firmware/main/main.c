/*
    Основной исполняемый файл для ESP32S3 робота МАПС(Мобильный Автономный Помощник Сотрудника)
*/

// подключение библиотек
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/spi_master.h" 
#include "driver/adc.h"
#include "driver/uart.h"

#include "unistd.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "string.h"

#include "TMC5160_reg.h"
#include "TMC5160_driver.h"
#include "bno_driver.h"
#include "servo.h"
#include "ADS1115.h"
#include "balance.h"
#include "VL53L0X_driver.h"

// константы
#define UART_NUM      UART_NUM_1
#define TX_PIN        17
#define RX_PIN        18
#define BUF_SIZE      1024

static const char *TAG_UART = "UART";
static const char *TAG_MAIN = "MAIN";
static const char *TAG_BALANCE = "BAL";

// основные структуры данных

bno_data_struct data;

bal_data_t bal_data = {
    .steer=0,
    .balance_u=0,
    .curr_v=0,
    .target_v=0,
    .actual_target_v=0,
    .curr_angle=0,
    .old_angle=0,
    .angle_err=0,
    .angle_errold=0,
    .dt=0.01,
    .setpoint=-3,
    .flag=true,
};

bal_k_t bal_k = {
    .kp=20,
    .kd=0.05,
    .ki=50,
    .speed_kp=0.04,
    .speed_ki=0.007,
};

void xADS1115_thread(void *arg);

mcpwm_cmpr_handle_t servo1;
mcpwm_cmpr_handle_t servo2;

typedef struct
{
    int16_t value1;
    int16_t value2;

    int16_t buttonvalue;
}bott_sens_t;
bott_sens_t bott_sens;

bool flag1=true, flag2=true;

bool humanflag = 0;
bool raspi_start = 0;

ads1115_t ads1115_cfg = {
  .reg_cfg =  ADS1115_CFG_LS_COMP_MODE_TRAD |
              ADS1115_CFG_LS_COMP_LAT_NON |
              ADS1115_CFG_LS_COMP_POL_LOW |
              ADS1115_CFG_LS_COMP_QUE_DIS |
              ADS1115_CFG_LS_DR_1600SPS |
              ADS1115_CFG_MS_MODE_SS,
  .dev_addr = 0x48,
};

spi_bus_config_t buscfg = { 
    .mosi_io_num = PIN_NUM_MOSI, 
    .miso_io_num = PIN_NUM_MISO, 
    .sclk_io_num = PIN_NUM_CLK, 
    .quadwp_io_num = -1, 
    .quadhd_io_num = -1, 
    .max_transfer_sz = 32, 
}; 
spi_device_interface_config_t devcfg1 = { 
    .clock_speed_hz = 4000000, 
    .mode = 3, 
    .spics_io_num = PIN_NUM_CS1, 
    .queue_size = 7, 
}; 
spi_device_handle_t spi1;

spi_device_interface_config_t devcfg2 = { 
    .clock_speed_hz = 4000000, 
    .mode = 3, 
    .spics_io_num = PIN_NUM_CS2, 
    .queue_size = 7, 
}; 
spi_device_handle_t spi2; 

float kp_steer = 1.5;


// задача балансировки, содержит ПИД-регулятор по углу и ПИ-регулятор по скорости движения
void balancing_task(void *arg)
{
    esp_task_wdt_add(NULL);
    while(!bno_is_calibrated())vTaskDelay(pdMS_TO_TICKS(10));
    servo_write(servo1, 0);
    servo_write(servo2, 180);
    vTaskDelay(pdMS_TO_TICKS(5000));
    uint64_t balance_timer;
    data = bno_get_data();
    
    while(fabs(data.kalman_x - bal_data.setpoint) < 10){data = bno_get_data(); vTaskDelay(pdMS_TO_TICKS(10)); esp_task_wdt_reset();}
    while(fabs(data.kalman_x - bal_data.setpoint) > 2){data = bno_get_data(); vTaskDelay(pdMS_TO_TICKS(10)); esp_task_wdt_reset();}
    bal_data.flag = false;

    while(1)
    {
        balance_timer = esp_timer_get_time();
        data = bno_get_data();
        if(bal_data.target_v > bal_data.actual_target_v)bal_data.actual_target_v += 1;
        if(bal_data.target_v < bal_data.actual_target_v)bal_data.actual_target_v -= 1;
        bal_data.curr_angle = -data.kalman_x - bal_data.setpoint;
        
        // ПИД по углу
        bal_data.p = bal_data.curr_angle;
        bal_data.d = (bal_data.curr_angle - bal_data.old_angle) / bal_data.dt;
        bal_data.i += (bal_data.curr_angle * bal_data.dt);
        if(fabs(bal_data.i) > 600)bal_data.i-=(bal_data.curr_angle * bal_data.dt);
        
        bal_data.balance_u = bal_data.p * bal_k.kp + bal_data.d * bal_k.kd + bal_data.i * bal_k.ki;
        
        bal_data.old_angle = bal_data.curr_angle;

        // ПИ по скорости
        bal_data.curr_v = 0.92f * bal_data.curr_v + 0.08f * bal_data.balance_u;
        bal_data.v_err = bal_data.target_v - bal_data.curr_v;
        bal_data.v_errI += bal_data.v_err * bal_data.dt;

        bal_data.setpoint = bal_data.v_err * bal_k.speed_kp + bal_data.v_errI * bal_k.speed_ki;
        
        // движение моторов
        setMaxSpeed(-bal_data.balance_u + bal_data.steer, spi1);
        setMaxSpeed(bal_data.balance_u + bal_data.steer, spi2);
        //ESP_LOGI(TAG_BALANCE, "a: %f, u: %f, set: %f\n", bal_data.curr_angle, bal_data.balance_u, bal_data.setpoint);
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


// задача по общению с Raspberry Pi по uart
void uart_task(void *arg) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_param_config(UART_NUM, &uart_config);

    uint8_t *uart_data = (uint8_t *) malloc(BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(UART_NUM, uart_data, BUF_SIZE - 1, pdMS_TO_TICKS(20));
        if (len > 0) {
            uart_data[len] = '\0';
            float value = 0;
            humanflag = 0;

            int items_parsed = sscanf((char *)uart_data, "%f %d", &value, (int*)&humanflag);
            if(items_parsed != 2)
            {
                humanflag = 0;
                bal_data.steer = 0;
                ESP_LOGE(TAG_UART, "UART data parsed wrong");
            }
            else
            {
                raspi_start = 1;
                ESP_LOGI(TAG_UART, "Angle: %f, Flag: %d", value, humanflag);
                if(humanflag)bal_data.steer = (value - 90) * kp_steer;
                else bal_data.steer = 0;
            }

        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    free(uart_data);
}


// основная выполняемая задача
void app_main(void)
{
    // настройка task watchdog, инициализация пинов
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 10000,
        .idle_core_mask = (1 << 0),
        .trigger_panic = false,
    };
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&twdt_config));

    gpio_reset_pin(GPIO_NUM_15);
    gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(GPIO_NUM_15, GPIO_FLOATING);
    gpio_set_level(GPIO_NUM_15, 1);

    adc1_config_width(ADC_WIDTH_BIT_12);

    // проверка напряжения
    float voltage = (float)(4095-adc1_get_raw(ADC1_CHANNEL_1)) / 1241.2121 * 11;
    
    if(voltage < 19.0)
    {
        ESP_LOGI(TAG_MAIN, "The battery voltage is too low. The program will not start.\n");
        while(1)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_task_wdt_reset();
        }
    }
    else if(voltage > 25.0)
    {
        ESP_LOGE(TAG_MAIN, "Voltage: %f, calculation does not seem correct\n", voltage);
    }

    // инициализация сервоприводов
    init_pwm();
    servo1 = servo_init(48);
    servo2 = servo_init(21);

    servo_write(servo1, 60);
    servo_write(servo2, 120);

    // инициализация драйверов tmc5160 для nema23
    spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO); 

    spi_bus_add_device(SPI3_HOST, &devcfg1, &spi1); 

    spi_bus_add_device(SPI3_HOST, &devcfg2, &spi2); 

    tmc5160_init(spi1);
    tmc5160_init(spi2);

    setAcceleration(1000, spi1);
    setAcceleration(1000, spi2);
    
    // инициализация I2c, АЦП и лазерных дальномеров
    vTaskDelay(pdMS_TO_TICKS(1000));
    i2c_master_init();
    vTaskDelay(pdMS_TO_TICKS(500));
    ADS1115_initiate(&ads1115_cfg);
    xTaskCreate(xADS1115_thread, "xADS1115_thread", 2048, NULL, 6, NULL);
    vTaskDelay(pdMS_TO_TICKS(500));
    xTaskCreate(vl53l0x_measure, "vl53_task", 8192, NULL, 8, NULL);
    vTaskDelay(pdMS_TO_TICKS(500)); 

    // запуск всех параллельно выполняемых задач
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 7, NULL);
    
    ESP_LOGI(TAG_MAIN, "UART task started. Waiting for RPi to load.");
    
    ESP_LOGI(TAG_MAIN, "RPi loaded and transferred first data pack. Calibrating gyro...");
    
    xTaskCreate(bno055_task, "bno055_task", 4096, NULL, 9, NULL);
    bal_data.target_v = 0;
    xTaskCreate(balancing_task, "balancing_task", 4096, NULL, 10, NULL);
    
    while(bal_data.flag)vTaskDelay(pdMS_TO_TICKS(10));
    
    bal_data.target_v = -30;
    vTaskDelay(pdMS_TO_TICKS(5000));
    bal_data.target_v = 0;

    kp_steer = 0;
    
    // основной исполняемый цикл
    while (true) {
        // действия после загрузки и подключения raspberry pi
        // движение, поворот и остановка в соответствии с данными с raspberry и дальномеров
        if (raspi_start) {
            if (humanflag) {
                if (vl2.value < 250 || vl3.value < 250) {
                    if (flag1 && flag2) {
                        bal_data.target_v = 20;
                        kp_steer = 0.75;
                    }
                } else if (vl2.value < 400 || vl3.value < 400) {
                    bal_data.target_v = 0;
                    kp_steer = 0;
                } else {
                    if (flag1 && flag2) {
                        bal_data.target_v = 40;
                        kp_steer = 1.5;
                    }
                }
            } else {
                bal_data.target_v = 0;
                bal_data.steer = 0;
                kp_steer = 0;
            }
        }

        // закрытие и открытие захватов по наажтию кнопок и срабатыванию датчиков
        if (!flag1 && bott_sens.value1 < 4000) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            servo_write(servo1, 0);
            flag1 = true;
            if (!raspi_start) break;
        }
        if (!flag1 && abs(bott_sens.buttonvalue - 1575) < 50) {
            servo_write(servo1, 0);
            flag1 = true;
            if (!raspi_start) break;
        }
        if (flag1 && abs(bott_sens.buttonvalue - 938) < 50) {
            servo_write(servo1, 60);
            flag1 = false;
            vTaskDelay(pdMS_TO_TICKS(4000));
        }

        if (!flag2 && bott_sens.value2 < 4000) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            servo_write(servo2, 180);
            flag2 = true;
            if (!raspi_start) break;
        }
        if (!flag2 && abs(bott_sens.buttonvalue - 2575) < 50) {
            servo_write(servo2, 180);
            flag2 = true;
            if (!raspi_start) break;
        }
        if (flag2 && abs(bott_sens.buttonvalue - 6415) < 50) {
            servo_write(servo2, 120);
            flag2 = false;
            vTaskDelay(pdMS_TO_TICKS(4000));
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// задача работы с АЦП ads1115
void xADS1115_thread(void *arg)
{
  while(1) 
  {
    ADS1115_request_single_ended_AIN3();

    while(!ADS1115_get_conversion_state()) 
      vTaskDelay(pdMS_TO_TICKS(5));
    
    bott_sens.value1 = ADS1115_get_conversion();
    
    ADS1115_request_single_ended_AIN2();

    while(!ADS1115_get_conversion_state()) 
      vTaskDelay(pdMS_TO_TICKS(5));
    
    bott_sens.value2 = ADS1115_get_conversion();

    ADS1115_request_single_ended_AIN1(); 

    while(!ADS1115_get_conversion_state()) 
      vTaskDelay(pdMS_TO_TICKS(5));

    bott_sens.buttonvalue = ADS1115_get_conversion();
  }
}