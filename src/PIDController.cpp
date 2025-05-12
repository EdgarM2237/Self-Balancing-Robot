#include "PIDController.h"
#include "esp_timer.h"  // Para obtener tiempo en microsegundos

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), setpoint(0), integral(0), lastError(0), lastTime(esp_timer_get_time()) {}

void PIDController::setTunings(float newKp, float newKi, float newKd) {
    kp = newKp;
    ki = newKi;
    kd = newKd;
}

void PIDController::setSetpoint(float sp) {
    setpoint = sp;
}

float PIDController::compute(float input) {
    uint64_t now = esp_timer_get_time(); // microsegundos
    float dt = (now - lastTime) / 1000000.0f; // convertir a segundos

    float error = setpoint - input;
    integral += error * dt;
    float derivative = (error - lastError) / dt;

    float output = kp * error + ki * integral + kd * derivative;

    lastError = error;
    lastTime = now;

    return output;
}

void PIDController::reset() {
    integral = 0;
    lastError = 0;
    lastTime = esp_timer_get_time();
}

float PIDController::getKp() const { return kp; }
float PIDController::getKi() const { return ki; }
float PIDController::getKd() const { return kd; }
