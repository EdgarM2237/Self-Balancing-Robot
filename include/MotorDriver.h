#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

class MotorDriver {
public:
    MotorDriver(int ain1, int ain2, int pwma,
                int bin1, int bin2, int pwmb,
                int stby);

    void begin();
    void move(float speed); // velocidad positiva: adelante, negativa: atrás
    void stop();

private:
    int AIN1, AIN2, PWMA;
    int BIN1, BIN2, PWMB;
    int STBY;

    void setMotor(int in1, int in2, int pwm_channel, float speed);
};

#endif
