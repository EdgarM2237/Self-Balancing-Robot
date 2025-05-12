#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "MPU6050.h"
#include "PIDController.h"
#include "MotorDriver.h"

// ======== CONFIGURACIÓN ========
#define LOOP_DELAY_MS 500 // Frecuencia de loop (10ms -> 100Hz)

// PID ajustes iniciales
float Kp = 30.0;
float Ki = 0.0;
float Kd = 0.8;

// Rango PWM
const int pwm_min = -1023;
const int pwm_max = 1023;

// Objetivo de ángulo (equilibrio)
float setpoint = 0.0;

// ======== OBJETOS GLOBALES ========

PIDController pid(Kp, Ki, Kd);
MotorDriver motors(16, 17, 4, 5, 18, 19, 33); // Pines ejemplo

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

    motors.begin();
    pid.setSetpoint(setpoint);

    printf("Sistema listo. Comenzando loop de control...\n");

    while (true)
    {
        mpu.update();
        float roll = mpu.getRoll();

        // Calcular PID
        float pid_output = pid.compute(roll);

        // Limitar PWM
        if (pid_output > pwm_max)
            pid_output = pwm_max;
        if (pid_output < pwm_min)
            pid_output = pwm_min;

        // Controlar motores (un solo valor)
        motors.move((int)pid_output);

        // Log para debugging
        printf("Roll: %.2f | PID: %.2f\n", roll, pid_output);

        vTaskDelay(LOOP_DELAY_MS / portTICK_PERIOD_MS);
    }
}
