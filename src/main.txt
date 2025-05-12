#include <stdio.h>
#include "MPU6050.h"

extern "C" void app_main(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 400000
        }
    };

    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

    MPU6050 mpu(I2C_NUM_0);

    if (!mpu.begin() || !mpu.testConnection()) {
        printf("MPU6050 no conectado.\n");
        return;
    }

    printf("Calibrando...\n");
    mpu.calibrate(6000);
    printf("Listo.\n");

    while (true) {
        mpu.update();
        printf("Yaw: %.2f, Pitch: %.2f, Roll: %.2f\n", mpu.getYaw(), mpu.getPitch(), mpu.getRoll());
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
