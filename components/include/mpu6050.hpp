#ifndef _MPU6050
#define _MPU6050

#include <cstdlib>
#include <cstdint>
#include "esp_log.h"
#include "driver/i2c.h"
#include <math.h>
#include "freertos/semphr.h"

//--------------------------------------------
// IMU 
#define I2C_MASTER_SCL_IO 23                    /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 22                    /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 0                        /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000               /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_VAL (i2c_ack_type_t)0x0             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define MPU6050_ADDR 0x68
#define MPU6050_RESET_ADDR 0x6B
#define GYRO_XOUT_H_ADDR 0x43 
#define ACCEL_XOUT_H_ADDR 0x3B
#define MPU6050_GYRO_CONFIG_ADDR 0x1B
#define MPU6050_ACCEL_CONFIG_ADDR 0x1C
#define MPU6050_USER_CTRL_ADDR 0x6A
#define MPU6050_1000D_SCALING 32.8
#define MPU6050_8G_SCALING  4096.0
#define MPU6050_TEMP_SCALING 340.0
#define MPU6050_TEMP_OFFSET_SCALING 35

#define BUFFER_SIZE 14
#define N_SAMPLES_BIAS 100
#define RAD2DEG 57.295779513

#define TWO_KP 1.0     // 2 * proportional gain
#define TWO_KI 0.1     // 2 * integral gain
#define BETA 0.3


class Mpu6050 {
    public:
    Mpu6050(uint8_t gpio_i2c_clk, uint8_t gpio_i2c_sda, float read_dt);

    virtual ~Mpu6050();
    float roll;
    float pitch;
    float yaw;
    float temperature;

    float bias_gx;
    float bias_gy;
    float bias_gz;

    float bias_ax;
    float bias_ay;
    float bias_az;

    esp_err_t read_mpu6050();
    void calculate_angles_madgwick();
    float data[6];

    private:
        uint8_t i2c_master_num;
        float read_dt;

        float q0;
        float q1;
        float q2;
        float q3;

        float integralFBx;
        float integralFBy;
        float integralFBz;

        uint8_t data_raw[BUFFER_SIZE];

        float inv_sqrt(float x);
        void calculate_bias_mpu6050();
        esp_err_t init_mpu6050();
        esp_err_t i2c_master_init(uint8_t gpio_i2c_clk, uint8_t gpio_i2c_sda);
};

#endif