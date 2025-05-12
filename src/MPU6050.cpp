/*!
 *  @file MPU6050.h
 *
 * 	I2C Driver for MPU6050 6-DoF Accelerometer and Gyro
 *
 * 	Esta es una libreria para la placa MPU6050 mediante comunicacion i2c
 *
 * 	Todo el codigo fuente de la libreria es completamente abierto para el uso
 *  teniendo como finalidad la facilidad de uso par aproyectos roboticos,
 *  domoticos, etc.
 *
 *  Toda la informacion requerida para su uso se encuentra en el github
 *  https://github.com/EdgarM2237/Self-Balancing-Robot
 *
 *  MIT license (see license.txt)
 */

#include "MPU6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

// definicion de registros por leer
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

static const char* TAG = "MPU6050";

MPU6050::MPU6050(i2c_port_t i2c_port, uint8_t address)
    : _i2c_port(i2c_port), _address(address), gyro_offset_x(0), gyro_offset_y(0), gyro_offset_z(0), last_update(0) {}

/*
    Despierta al sensor escribiendo 0 al registro PWR_MGMT_1 con la finalidad  de sacarlo del modo sleep al cual entra al encenderse el MPU6050
*/
bool MPU6050::begin() {
    uint8_t data = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, 0, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(_i2c_port, cmd, 1000 / portTICK_PERIOD_MS); //Escribe 0 en el registro para despertar el sensor
    i2c_cmd_link_delete(cmd);

    return err == ESP_OK; //retorna estado OK di logra iniciarse
}

/*
    Lee el registro WHO_AM_I del sensor (0x75) el cual nos devuelve un valor de 0x68 en caso tal de exito, comparamos el valor y si coincide *   retornamos      ESP_OK para continuar con la lectura, en caso contrario retornamos FALSE
*/
bool MPU6050::testConnection() {
    uint8_t who_am_i = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, WHO_AM_I_REG, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &who_am_i, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(_i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    ESP_LOGI(TAG, "WHO_AM_I: 0x%02X", who_am_i);
    return (err == ESP_OK && who_am_i == 0x68);
}

/*
    Lee los datos crudos del acelerómetro (accX, accY, accZ) y giroscopio (gyroX, gyroY, gyroZ). Convierte a unidades físicas:

       - Acelerómetro: divide por 16384.0 para obtener "g".

       - Giroscopio: divide por 131.0 para obtener grados/segundo.
*/
void MPU6050::readRawData() {
    /*
        Función auxiliar que lee dos bytes seguidos desde un registro del sensor y los convierte a un int16_t, esto nos retorna valores de 16 bits con signo
    */
    accX = (float)readWord(ACCEL_XOUT_H) / 16384.0;
    accY = (float)readWord(ACCEL_XOUT_H + 2) / 16384.0;
    accZ = (float)readWord(ACCEL_XOUT_H + 4) / 16384.0;

    gyroX = (float)readWord(GYRO_XOUT_H) / 131.0 - gyro_offset_x;
    gyroY = (float)readWord(GYRO_XOUT_H + 2) / 131.0 - gyro_offset_y;
    gyroZ = (float)readWord(GYRO_XOUT_H + 4) / 131.0 - gyro_offset_z;
}

/*
    Durante ese tiempo (Por defecto 3seg), toma muestras del giroscopio mientras el sensor está quieto. Calcula el promedio y lo guarda como offset en los tres ejes.
*/
void MPU6050::calibrate(int calibration_time_ms) {
    int samples = 0;
    float sumX = 0, sumY = 0, sumZ = 0;

    uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while ((xTaskGetTickCount() * portTICK_PERIOD_MS) - start < calibration_time_ms) {
        sumX += readWord(GYRO_XOUT_H) / 131.0;
        sumY += readWord(GYRO_XOUT_H + 2) / 131.0;
        sumZ += readWord(GYRO_XOUT_H + 4) / 131.0;
        samples++;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    gyro_offset_x = sumX / samples;
    gyro_offset_y = sumY / samples;
    gyro_offset_z = sumZ / samples;

    last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/*
    Funcion clave para la lectura de los datos Yaw, Pitch, Roll
    Debe llamarse repetidamente dentro de un bucle para extraer los datos del sensor y hacer los calculos correspondiente siguiendo con la logica de llamar a readRawData() para actualizar datos del sensor, integra el valor del gyroZ en el tiempo para calcular el Yaw y usa formulas trigonometricas para calcular el Pitch y Roll con los valores del acelerometro.
*/
void MPU6050::update() {
    readRawData();

    // Integración simple para yaw
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt = (now - last_update) / 1000.0;
    last_update = now;

    yaw += gyroZ * dt; // Integracion de gyroZ en el tiempo dt para obtener Yaw

    // Cálculo de pitch y roll con acelerómetro
    pitch = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / M_PI;
    roll  = atan2(-accX, accZ) * 180.0 / M_PI;
}

//Retorno de los ultimos valores calculados
float MPU6050::getYaw() const { return yaw; }
float MPU6050::getPitch() const { return pitch; }
float MPU6050::getRoll() const { return roll; }

int16_t MPU6050::readWord(uint8_t reg) {
    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(_i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (int16_t)((data[0] << 8) | data[1]);
}
