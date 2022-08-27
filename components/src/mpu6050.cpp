#include <mpu6050.hpp>

using namespace std;

//------------------------------------------------------------------------------------
// gpio clk, gpio sda, imu read time [ms] used for angle integration
Mpu6050::Mpu6050(uint8_t gpio_i2c_clk, uint8_t gpio_i2c_sda, float read_dt) {
    this->read_dt = read_dt/1000;
    this->integralFBx = 0;
    this->integralFBy = 0;
    this->integralFBz = 0;

    this->q0 = 1.0;
    this->q1 = 0;
    this->q2 = 0;
    this->q3 = 0;

    this->bias_ax = 0;
    this->bias_ay = 0;
    this->bias_az = 0;
    this->bias_gx = 0;
    this->bias_gy = 0;
    this->bias_gz = 0;

    i2c_master_init(gpio_i2c_clk, gpio_i2c_sda);
    init_mpu6050();
    calculate_bias_mpu6050();
}

Mpu6050::~Mpu6050() {
}

//------------------------------------------------------------------------------------
// Read all imu registers (accelerometer, gyroscope, temperature) and save data
// Accelerometer: data[0,1,2] [m/s^2]
// Gyroscope: data[3,4,5] [degrees/sec]
// Temperature: [°C]
esp_err_t Mpu6050::read_mpu6050() {
    SemaphoreHandle_t print_mux = xSemaphoreCreateMutex();
    int ret = 0;

    //--------------------------------------------
    // GYRO_XOUT_H_ADDR: Base register
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret |= i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    ret |= i2c_master_write_byte(cmd, ACCEL_XOUT_H_ADDR, ACK_CHECK_EN);
    ret |= i2c_master_stop(cmd);

    ret |= i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    //--------------------------------------------

    //--------------------------------------------
    cmd = i2c_cmd_link_create();
    ret |= i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    // Read 14 Bytes: 2 * X/Y/Z high and low from Gyroscope, Accelerometer and temperature data
    ret |= i2c_master_read(cmd, data_raw, BUFFER_SIZE, ACK_VAL);
    ret |= i2c_master_stop(cmd);

    ret |= i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    //--------------------------------------------

    data[0] = ((int16_t)(data_raw[0] << 8) | data_raw[1]) / MPU6050_8G_SCALING - bias_ax;
    data[1] = ((int16_t)(data_raw[2] << 8) | data_raw[3]) / MPU6050_8G_SCALING - bias_ay;
    data[2] = ((int16_t)(data_raw[4] << 8) | data_raw[5]) / MPU6050_8G_SCALING - bias_az;
    data[3] = ((int16_t)(data_raw[8] << 8) | data_raw[9]) / MPU6050_1000D_SCALING - bias_gx;
    data[4] = ((int16_t)(data_raw[10] << 8) | data_raw[11]) / MPU6050_1000D_SCALING - bias_gy;
    data[5] = ((int16_t)(data_raw[12] << 8) | data_raw[13]) / MPU6050_1000D_SCALING - bias_gz;
    this->temperature = ((int16_t)(data_raw[6] << 8) | data_raw[7]) / MPU6050_TEMP_SCALING + MPU6050_TEMP_OFFSET_SCALING;

    vSemaphoreDelete(print_mux);
    return ret;
}


//------------------------------------------------------------------------------------
// Fast inverse square-root
float Mpu6050::inv_sqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}

//------------------------------------------------------------------------------------
// Adafruit AHRS Madgwick filter combining accelerometer and gyroscope data to calculate roll, pitch and yaw
// Adafruit licence: https://github.com/adafruit/Adafruit_AHRS
void Mpu6050::calculate_angles_madgwick() {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Convert gyroscope degrees/sec to radians/sec
    // Convert gyroscope degrees/sec to radians/sec
    float gx = data[3] * 0.0174533f;
    float gy = data[4] * 0.0174533f;
    float gz = data[5] * 0.0174533f;

    float ax = data[0];
    float ay = data[1];
    float az = data[2];

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in
    // accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 +
             _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
             _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 +
                            s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= BETA * s0;
        qDot2 -= BETA * s1;
        qDot3 -= BETA * s2;
        qDot4 -= BETA * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * read_dt;
    q1 += qDot2 * read_dt;
    q2 += qDot3 * read_dt;
    q3 += qDot4 * read_dt;

    // Normalise quaternion
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * RAD2DEG;   // roll
    pitch = asinf(-2.0f * (q1 * q3 - q0 * q2)) * RAD2DEG;                   // pitch
    yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * RAD2DEG;    // yaw
}

//-------------------------------------------------------------------------------------------
// Get average of a number of samples to compensate constant bias
void Mpu6050::calculate_bias_mpu6050() {
    uint16_t i = 0;

    for (i=0; i<N_SAMPLES_BIAS; i++) {
        read_mpu6050();
        bias_ax += ((int16_t)(data_raw[0]<<8) | data_raw[1])/MPU6050_8G_SCALING;
        bias_ay += ((int16_t)(data_raw[2]<<8) | data_raw[3])/MPU6050_8G_SCALING;
        bias_az += ((int16_t)(data_raw[4]<<8) | data_raw[5])/MPU6050_8G_SCALING - 1.0;

        bias_gx += ((int16_t)(data_raw[8]<<8) | data_raw[9])/MPU6050_1000D_SCALING;
        bias_gy += ((int16_t)(data_raw[10]<<8) | data_raw[11])/MPU6050_1000D_SCALING;
        bias_gz += ((int16_t)(data_raw[12]<<8) | data_raw[13])/MPU6050_1000D_SCALING;
    }

    bias_ax /= N_SAMPLES_BIAS;
    bias_ay /= N_SAMPLES_BIAS;
    bias_az /= N_SAMPLES_BIAS;

    bias_gx /= N_SAMPLES_BIAS;
    bias_gy /= N_SAMPLES_BIAS;
    bias_gz /= N_SAMPLES_BIAS;
}

//------------------------------------------------------------------------------------
// Mpu6050 register initialization
esp_err_t Mpu6050::init_mpu6050() {
    int ret = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    //--------------------------------------------
    // Make reset (place a 0 into 6B register)
    i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    ret |= i2c_master_write_byte(cmd, MPU6050_RESET_ADDR, ACK_CHECK_EN);
    ret |= i2c_master_write_byte(cmd, 0x00, ACK_VAL);
    ret |= i2c_master_stop(cmd);

    ret |= i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    //--------------------------------------------

    vTaskDelay(30 / portTICK_RATE_MS);

    //--------------------------------------------
    // Write to the GYRO_CONFIG register and set as 00010000 (1000dps full scale)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    ret |= i2c_master_write_byte(cmd, MPU6050_GYRO_CONFIG_ADDR, ACK_CHECK_EN);
    ret |= i2c_master_write_byte(cmd, 0x10, ACK_VAL);
    ret |= i2c_master_stop(cmd);

    ret |= i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    //--------------------------------------------

    //--------------------------------------------
    // Write to the ACC_CONFIG register and set as 00010000 (+/- 8g full scale range)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    ret |= i2c_master_write_byte(cmd, MPU6050_ACCEL_CONFIG_ADDR, ACK_CHECK_EN);
    ret |= i2c_master_write_byte(cmd, 0x10, ACK_VAL);
    ret |= i2c_master_stop(cmd);

    ret |= i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    //--------------------------------------------

    //--------------------------------------------
    // Enable I2C master mode for data burst (get multiple data)
    // USER_CTRL register: Bit 5 = 1
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    ret |= i2c_master_write_byte(cmd, MPU6050_USER_CTRL_ADDR, ACK_CHECK_EN);
    ret |= i2c_master_write_byte(cmd, 0x20, ACK_VAL);
    ret |= i2c_master_stop(cmd);

    ret |= i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    //--------------------------------------------

    return ret;
}

//------------------------------------------------------------------------------------
// I2C master initialization
esp_err_t Mpu6050::i2c_master_init(uint8_t gpio_i2c_clk, uint8_t gpio_i2c_sda) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = gpio_i2c_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = gpio_i2c_clk;
    conf.clk_flags = 0;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(
        i2c_master_port, conf.mode,
        I2C_MASTER_RX_BUF_DISABLE,
        I2C_MASTER_TX_BUF_DISABLE, 0
    );
}