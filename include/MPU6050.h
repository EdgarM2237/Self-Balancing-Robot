#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"

class MPU6050 {
public:
    MPU6050(i2c_port_t i2c_port, uint8_t address = 0x68);
    bool begin();
    bool testConnection();
    void calibrate(int calibration_time_ms = 3000);
    void update();
    float getYaw() const;
    float getPitch() const;
    float getRoll() const;

private:
    i2c_port_t _i2c_port;
    uint8_t _address;

    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float pitch, roll, yaw;
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;

    uint32_t last_update;

    void readRawData();
    int16_t readWord(uint8_t reg);
};

#endif
