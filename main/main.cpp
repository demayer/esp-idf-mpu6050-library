/* MPU6050 esp-idf library using Adafruit's AHRS Madgwick orientation filter to calculate roll, pitch and yaw angles  */
/* Adafruit AHRS: https://github.com/adafruit/Adafruit_AHRS */

#include <stdio.h>
#include <mpu6050.hpp>

extern "C" {
    void app_main(void);
}

void app_main(void) {

    // clk gpio pin 23
    // sda gpio pin 22
    // read speed 50 ms
    Mpu6050* imu = new Mpu6050(23, 22, 50);

    while (1) {
        imu->read_mpu6050();
        imu->calculate_angles_madgwick();
        printf("%.2f, %.2f, %.2f, Temperature: %.2f\n", imu->roll, imu->pitch, imu->yaw, imu->temperature);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
