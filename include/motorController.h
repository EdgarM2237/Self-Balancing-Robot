#ifndef __MOTORCONTROLLER_H__
#define __MOTORCONTROLLER_H__

#include <driver/gpio.h>
#include <driver/ledc.h>


class motorController{
public:
    /**
     * @brief Constructor de la clase motorController.
     *
     * Inicializa los pines necesarios para el control de dos motores DC mediante el driver,
     * asignando los pines de dirección (A, B) y PWM para cada motor (izquierdo y derecho).
     *
     * @param leftA     Pin GPIO para la entrada A del motor izquierdo.
     * @param leftB     Pin GPIO para la entrada B del motor izquierdo.
     * @param leftPwm   Pin GPIO para la señal PWM del motor izquierdo.
     * @param rightA    Pin GPIO para la entrada A del motor derecho.
     * @param rightB    Pin GPIO para la entrada B del motor derecho.
     * @param rightPwm  Pin GPIO para la señal PWM del motor derecho.
     */
    motorController( gpio_num_t leftA, gpio_num_t leftB, gpio_num_t leftPwm ,
                     gpio_num_t rightA, gpio_num_t rightB, gpio_num_t rightPwm);

    /**
     * @brief Inicializar los módulos de hardware.
     */
    void init();

    /**
     * @brief Establezca la velocidad de cada motor. Los valores pueden ser negativos (rueda girando hacia atras).
     *        Ambos parametros deberan estar entre 0 - 1024
     *
     * @param leftSpeed     Velocidad para el motor izquierdo.
     * @param rightSpeed    Velocidad para el motor derecho.
     */
    void setSpeed( int32_t leftSpeed, int32_t rightSpeed);

private:

    gpio_num_t leftPinA, leftPinB, leftPinPwm;
    gpio_num_t rightPinA, rightPinB, rightPinPwm;

    ledc_channel_t leftPwmChannel, rightPwmChannel;

    typedef enum{
        DIR_FWD = 0,
        DIR_BCK
    }Direction;

    Direction dirL, dirR;



};

#endif //__MOTORCONTROLLER_H__
