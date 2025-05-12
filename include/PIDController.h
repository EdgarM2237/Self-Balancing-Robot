#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd);

    void setTunings(float kp, float ki, float kd);
    void reset();
    void setSetpoint(float sp);
    float compute(float input); // Retorna la salida PID

    float getKp() const;
    float getKi() const;
    float getKd() const;

private:
    float kp, ki, kd;
    float setpoint;
    float integral;
    float lastError;
    uint64_t lastTime;
};

#endif
