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

// ======== DEFINICIONES ========

// Pines de los motores a usar
constexpr gpio_num_t AIN1 = GPIO_NUM_25;
constexpr gpio_num_t AIN2 = GPIO_NUM_26;
constexpr gpio_num_t PWMA = GPIO_NUM_27;
constexpr gpio_num_t BIN1 = GPIO_NUM_32;
constexpr gpio_num_t BIN2 = GPIO_NUM_33;
constexpr gpio_num_t PWMB = GPIO_NUM_14;

// PID ajustes iniciales
float Kp = 200.0;
float Ki = 0.5;
float Kd = 0.09;

// Instancia de la clase motorController
motorController motors(AIN1, AIN2, PWMA, BIN1, BIN2, PWMB);

extern "C" void app_main()
{
    // ========== CONFIGURACIONES INICIALES ===========
    // Definicion de caracteristicas del bus i2c
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

    // Aplicacion de configuracion del bus i2c
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

    // instancias de la clase MPU y PID
    MPU6050 mpu(I2C_NUM_0);
    PID pid(Kp, Ki, Kd, 0.0);

    // Inicializar periféricos
    ESP_LOGE("MAIN","Inicializando periféricos...");

    // Inicializar mpu6050 y verificar estado de la coneccion
    mpu.begin();
    if (!mpu.testConnection())
    {
        ESP_LOGE("MAIN","MPU6050 no detectado");
        while (true)
            vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    //Calibracion del sensor mpu respecto a un punto
    //? Actualmente en caso omiso
    mpu.calibrate();
    ESP_LOGE("MAIN","MPU6050 calibrado");


    // Inicializar motores
    motors.init();
    ESP_LOGE("MAIN","Sistema listo. Comenzando loop de control...");

    uint32_t motor;
    float last_update = 0.0f;

    // ========== LOOP DE CONTROL ===========
    for (;;)
    {
        // Guardar dt para nuestro PID
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (now - last_update) / 1000.0;
        last_update = now;

        // Calcular velocidad de motores respecto al vector pitch y dt
        float pitch = mpu.getPitch();
        motor = pid.update(pitch, dt);

        //Establecer velocidad de motores
        motors.setSpeed(motor,motor);
        mpu.update();

        //ESP_LOGE("MAIN","Pitch: %.2f", pitch);
        //ESP_LOGE("MAIN","Motor: %ld", motor);

        vTaskDelay(2);
    }
}
