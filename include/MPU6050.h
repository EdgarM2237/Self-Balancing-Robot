#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"


/**
 * @struct mpu6050_sensor_data
 * @brief Estructura para almacenar datos tridimensionales del sensor MPU6050.
 *
 * Esta estructura guarda los valores de los tres ejes (x, y, z) ya sea del
 * acelerómetro o del giroscopio, junto con una marca de tiempo (`timestamp`)
 * correspondiente al momento de la medición.
 */
struct mpu6050_sensor_data {
    float x;          /**< Valor en el eje X (aceleración o velocidad angular). */
    float y;          /**< Valor en el eje Y (aceleración o velocidad angular). */
    float z;          /**< Valor en el eje Z (aceleración o velocidad angular). */
    float timestamp;  /**< Tiempo en milisegundos o microsegundos desde la última medición. */
};


/**
 * @file MPU6050.h
 * @author Edgar Mendez
 * @date 2025-05-25
 * @brief Clase para controlar el sensor MPU6050 utilizando ESP-IDF puro.
 *
 * Esta clase encapsula todas las funciones necesarias para iniciar el sensor,
 * calibrar el giroscopio, leer datos del acelerómetro y giroscopio,
 * calcular los ángulos de orientación (yaw, pitch, roll), y aplicar filtros
 * como el complemento o Kalman para estabilizar las mediciones.
 *
 * Compatible con ESP32 utilizando el framework ESP-IDF sin dependencias de Arduino.
 */
class MPU6050
{

public:
    /**
     * @brief Constructor de la clase MPU6050.
     *
     * Inicializa la instancia del sensor MPU6050 con el puerto I2C y la dirección del dispositivo.
     *
     * @param i2c_port  Puerto I2C que se utilizará (por ejemplo, I2C_NUM_0 o I2C_NUM_1).
     * @param address   Dirección I2C del sensor (por defecto 0x68).
     */
    MPU6050(i2c_port_t i2c_port, uint8_t address = 0x68);

    /**
     * @brief Inicializa el sensor MPU6050.
     *
     * Configura los registros necesarios del sensor para habilitar el acelerómetro y giroscopio.
     *
     * @return true si la inicialización fue exitosa, false en caso contrario.
     */
    bool begin();

    /**
     * @brief Verifica si el sensor está conectado correctamente.
     *
     * Realiza una lectura del registro WHO_AM_I para validar la conexión del sensor MPU6050.
     *
     * @return true si la conexión es exitosa, false si no se detecta el sensor.
     */
    bool testConnection();

    /**
     * @brief Lee los datos crudos del acelerómetro y giroscopio.
     *
     * Obtiene los valores de aceleración y velocidad angular en los tres ejes y los almacena
     * en las estructuras pasadas por referencia. También guarda el tiempo entre lecturas en microsegundos.
     *
     * @param acc  Estructura donde se almacenarán los datos del acelerómetro.
     * @param gyr  Estructura donde se almacenarán los datos del giroscopio.
     */
    void readRawData(mpu6050_sensor_data &acc, mpu6050_sensor_data &gyr);

    /**
     * @brief Calibra el giroscopio en una posición fija.
     *
     * Promedia las lecturas durante un intervalo determinado para establecer un offset en el giroscopio,
     * el cual será restado en futuras mediciones para mejorar la precisión.
     *
     * @param calibration_time_ms Tiempo (en milisegundos) que se usará para realizar la calibración. Por defecto 3000 ms.
     */
    void calibrate(int calibration_time_ms = 3000);

    /**
     * @brief Actualiza los valores de yaw, pitch y roll.
     *
     * Realiza una integración del giroscopio para obtener los ángulos de orientación,
     * y aplica un filtro complementario (o Kalman si está implementado) para mayor precisión.
     */
    void update();

    /**
     * @brief Obtiene el valor actual del ángulo de yaw.
     *
     * @return Valor del ángulo de yaw en grados.
     */
    float getYaw() const;

    /**
     * @brief Obtiene el valor actual del ángulo de pitch.
     *
     * @return Valor del ángulo de pitch en grados.
     */
    float getPitch() const;

    /**
     * @brief Obtiene el valor actual del ángulo de roll.
     *
     * @return Valor del ángulo de roll en grados.
     */
    float getRoll() const;

private:
    /**
     * @brief Puerto I2C utilizado para la comunicación con el sensor.
     */
    i2c_port_t _i2c_port;

    /**
     * @brief Dirección I2C del sensor MPU6050.
     */
    uint8_t _address;

    /**
     * @brief Valores de aceleración en los ejes X, Y y Z (en g).
     */
    float accX, accY, accZ;

    /**
     * @brief Valores de velocidad angular en los ejes X, Y y Z (en °/s).
     */
    float gyroX, gyroY, gyroZ;

    /**
     * @brief Ángulos calculados a partir de los datos del sensor (en grados).
     */
    float pitch, roll, yaw;

    /**
     * @brief Offset del giroscopio en los ejes X, Y y Z para calibración.
     */
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;

    /**
     * @brief Marca de tiempo de la última actualización en milisegundos.
     */
    uint32_t last_update;

    /**
     * @brief Marca de tiempo de la última lectura en microsegundos.
     */
    uint64_t last_read_time_us = 0;

    /**
     * @brief Lee dos bytes consecutivos del registro indicado y los combina en un entero con signo.
     *
     * @param reg Dirección del primer byte a leer.
     * @return int16_t Valor combinado de los dos bytes leídos.
     */
    int16_t readWord(uint8_t reg);

};

#endif
