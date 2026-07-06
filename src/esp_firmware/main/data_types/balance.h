#ifndef BALANCE_DRIVER_H
    #define BALANCE_DRIVER_H
    #include "driver/spi_master.h" 
    #include "TMC5160_driver.h"
    
    typedef struct
    {
        double balance_u;
        double curr_v;
        double target_v;
        double actual_target_v;
        double v_err;
        double v_errI;
        double steer;
        double setpoint;
        double curr_angle;
        double old_angle;
        double angle_err;
        double angle_errold;
        double p, i, d;
        double dt;
        bool flag;
    } bal_data_t;

    typedef struct
    {
        double kp;
        double kd;
        double ki;
        double speed_kp;
        double speed_ki;
    } bal_k_t;

    void i2c_master_init(void);
#endif