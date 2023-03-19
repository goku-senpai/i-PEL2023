#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "pid_controller.h"
#include "constants.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"

class MotorController {
public:
    MotorController(TIM_HandleTypeDef* htim_pwm, uint32_t channel_pwm, GPIO_TypeDef* gpio_dir,
                    TIM_HandleTypeDef* htim_encoder, uint32_t pin_encoder_a, uint32_t pin_encoder_b,
                    float pos_kp, float pos_ki, float pos_kd, float max_output, float max_integral,
                    float target_start, uint32_t pin_direction);

    void set_direction(uint8_t direction);
    void set_target(float target);
    void set_mode(uint8_t mode);
    float get_error();
    void update(float sample_time);
    void set_output(float output);

private:
    TIM_HandleTypeDef* htim_pwm_;
    uint32_t channel_pwm_;
    GPIO_TypeDef* gpio_dir_;
    TIM_HandleTypeDef* htim_encoder_;
    uint32_t pin_encoder_a_;
    uint32_t pin_encoder_b_;
    uint32_t pin_direction_;

    PIDController pid_controller_;

    float current_position_;
    float current_output_;
};

#endif
