#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "MPU6050.h"
#include "pid.h"
#include "motorController.h"

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include <esp_mac.h>

// ======== CONFIGURACIÓN ========
#define LOOP_DELAY_MS 500 // Frecuencia de loop (10ms -> 100Hz)

// Rango PWM
const int pwm_min = -1023;
const int pwm_max = 1023;

// Objetivo de ángulo (equilibrio)
float setpoint = 0.0;

// Pines de los motores a usar
constexpr gpio_num_t AIN1 = GPIO_NUM_25;
constexpr gpio_num_t AIN2 = GPIO_NUM_26;
constexpr gpio_num_t PWMA = GPIO_NUM_27;
constexpr gpio_num_t BIN1 = GPIO_NUM_32;
constexpr gpio_num_t BIN2 = GPIO_NUM_33;
constexpr gpio_num_t PWMB = GPIO_NUM_14;

//motors(25, 26, 27, 32, 33, 14, 13);
// ======== OBJETOS GLOBALES ========

// PID ajustes iniciales
float Kp = 200.0;
float Ki = 0.5;
float Kd = 0.09;

motorController motors(AIN1, AIN2, PWMA, BIN1, BIN2, PWMB);

extern "C" void app_main()
{

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
    PID pid(Kp, Ki, Kd, 0.0);

    // Inicializar periféricos
    printf("Inicializando módulos...\n");

    mpu.begin();
    if (!mpu.testConnection())
    {
        printf("¡Error: MPU6050 no detectado!\n");
        while (true)
            vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    mpu.calibrate(); // Calibrar orientación inicial
    printf("MPU6050 calibrado\n");

    motors.init();

    printf("Sistema listo. Comenzando loop de control...\n");

    uint32_t motor;
    float last_update = 0.0f;

    for (;;)
    {
        // Step 1: calculate alpha ( Angle from the vertical)

        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (now - last_update) / 1000.0;
        last_update = now;

        // Step 2: Calculate motors using PID
        float roll = mpu.getPitch();
        motor = pid.update(roll, dt);
        motors.setSpeed(motor,motor);
        mpu.update();

        //ESP_LOGE("MAIN","Roll: %.2f", roll);
        ESP_LOGE("MAIN","Motor: %ld", motor);

        vTaskDelay(2);
    }
}
