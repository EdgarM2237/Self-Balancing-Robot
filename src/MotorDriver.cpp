#include "MotorDriver.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

MotorDriver::MotorDriver(int ain1, int ain2, int pwma,
                         int bin1, int bin2, int pwmb,
                         int stby)
    : AIN1(ain1), AIN2(ain2), PWMA(pwma),
      BIN1(bin1), BIN2(bin2), PWMB(pwmb), STBY(stby)
{}
void MotorDriver::begin() {
    gpio_set_direction((gpio_num_t)AIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)AIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)BIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)BIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)STBY, GPIO_MODE_OUTPUT);

    // Configurar PWM para PWMA y PWMB
    ledc_timer_config_t timer_conf;
    timer_conf.speed_mode       = LEDC_HIGH_SPEED_MODE;
    timer_conf.freq_hz          = 500;
    timer_conf.duty_resolution  = LEDC_TIMER_10_BIT;
    timer_conf.clk_cfg          = LEDC_AUTO_CLK;
    timer_conf.timer_num        = LEDC_TIMER_0;
    timer_conf.deconfigure      = 0;
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel0 = {
        .gpio_num       = PWMA,
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0,
        .flags          = { .output_invert = 0 },
    };

    ledc_channel_config_t channel1 = {
        .gpio_num       = PWMB,
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0,
        .flags          = { .output_invert = 0 },
    };


    ledc_channel_config(&channel0);
    ledc_channel_config(&channel1);

    gpio_set_level((gpio_num_t)STBY, 1);
}

void MotorDriver::setMotor(int in1, int in2, int pwm_channel, float speed) {
    bool forward = speed >= 0;
    int duty = (int)speed;

    duty = (duty > 1023) ? 1023 : ((duty < -1023) ? -1023 : duty);
    int abs_duty = abs(duty);

    gpio_set_level((gpio_num_t)in1, forward ? 1 : 0);
    gpio_set_level((gpio_num_t)in2, forward ? 0 : 1);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)pwm_channel, abs_duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)pwm_channel);
}

void MotorDriver::move(float speed) {
    setMotor(AIN1, AIN2, LEDC_CHANNEL_0, speed);
    setMotor(BIN1, BIN2, LEDC_CHANNEL_1, speed);
}

void MotorDriver::stop() {
    setMotor(AIN1, AIN2, LEDC_CHANNEL_0, 0);
    setMotor(BIN1, BIN2, LEDC_CHANNEL_1, 0);
}
