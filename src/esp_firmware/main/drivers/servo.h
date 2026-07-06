// самостоятельно написанный драйвер для работы с сервоприводами через изменение скважности ШИМ в 50 гц

#ifndef SERVO_DRIVER
    #define SERVO_DRIVER
    #define SERVO_MIN_PULSEWIDTH_US 500  
    #define SERVO_MAX_PULSEWIDTH_US 2500
    #define SERVO_MIN_DEGREE        0
    #define SERVO_MAX_DEGREE        180

    #define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1МГц, 1000000 тиков в секунду
    #define SERVO_TIMEBASE_PERIOD        20000    // 20 мс, 20000 тиков или 20000 микросекунд

    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_log.h"
    #include "driver/mcpwm_prelude.h"
    #include <stdio.h>

    void init_pwm();
    mcpwm_cmpr_handle_t servo_init(int gpio_pin);
    uint32_t angle_to_compare(int angle);
    void servo_write(mcpwm_cmpr_handle_t comp, float angle);

#endif