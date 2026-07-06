// драйвер для bno055 (в основном написан самостоятельно)


//#define I2C_MASTER_CONFIG_MAIN
#define BNO055_SENSOR_ADDR          0x28  // адрес датчика
#define BNO055_OPR_MODE_ADDR        0x3D  // адрес регистра с режимом работы
#define BNO055_PWR_MODE_ADDR        0x3E  // адрес регистра с режимом питания
#define BNO055_TEMP_ADDR            0x34  // адрес регистра с температурой
#define BNO055_ACCEL_DATA_X_LSB     0x08  // с данными акселерометра
#define BNO055_GYRO_DATA_X_LSB      0x14  // с данными гироскопа
#define BNO055_MAG_DATA_X_LSB       0x0E  // с данными магнетометра

#define RADIANS_TO_DEGREES          57.2958  /*Коэффициент конвертации углов из радиан в градусы*/

// константы для инициализации i2c
#ifndef I2C_MASTER_CONFIG_MAIN
    #define I2C_MASTER_SCL_IO           9    
    #define I2C_MASTER_SDA_IO           8   
    #define I2C_MASTER_NUM              I2C_NUM_1 
    #define I2C_MASTER_FREQ_HZ          400000 
    #define I2C_MASTER_TX_BUF_DISABLE   0     
    #define I2C_MASTER_RX_BUF_DISABLE   0     
#endif

// типы структур данных для работы с датчиком
typedef struct
{
    double gy_cal_x;
    double gy_cal_y;
    double gy_cal_z;

} bno_calib_struct;

typedef struct
{
    double acc_x;
    double acc_y;
    double acc_z;
    
    double gy_x;
    double gy_y;
    double gy_z;
    
    double mag_x;
    double mag_y;
    double mag_z;

    int16_t temp;

    double acc_angle_x;
    double acc_angle_y;

    double kalman_x;
    double kalman_y;
    
    double gy_angle_z;
    double mag_angle_z;

} bno_data_struct;

typedef struct 
{
    double kalman_angle;
    double kalman_uncert_angle;
} kalman_data;


void i2c_master_init(void);
esp_err_t bno055_write_register(uint8_t reg_addr, uint8_t data);
esp_err_t bno055_read_register(uint8_t reg_addr, uint8_t *data, size_t len);
void bno055_init(void);
void bno055_calib(void);
bool bno_is_calibrated(void);
void bno055_read_data(void);
void kalman(kalman_data *data, double input, double measurement);
void bno055_task(void *arg);
bno_data_struct bno_get_data();
