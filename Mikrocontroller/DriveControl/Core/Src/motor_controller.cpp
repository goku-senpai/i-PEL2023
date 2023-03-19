#include "motor_controller.h"
#include "cmath"

MotorController::MotorController(TIM_HandleTypeDef* htim_pwm, uint32_t channel_pwm, GPIO_TypeDef* gpio_dir,
                                 TIM_HandleTypeDef* htim_encoder, uint32_t pin_encoder_a, uint32_t pin_encoder_b,
                                 float pos_kp, float pos_ki, float pos_kd, float max_output, float max_integral,
                                 float target_start, uint32_t pin_direction)
        : htim_pwm_(htim_pwm), channel_pwm_(channel_pwm), gpio_dir_(gpio_dir), htim_encoder_(htim_encoder),
          pin_encoder_a_(pin_encoder_a), pin_encoder_b_(pin_encoder_b), pin_direction_(pin_direction),
          pid_controller_(pos_kp, pos_ki, pos_kd, max_output, max_integral, target_start),
          current_position_(0), current_output_(0) {
}

void MotorController::set_direction(uint8_t direction) {
    HAL_GPIO_WritePin(gpio_dir_, pin_direction_, static_cast<GPIO_PinState>(direction));
}

void MotorController::set_target(float target) {
    pid_controller_.set_target(target);
}

void MotorController::set_mode(uint8_t mode) {
    pid_controller_.set_mode(mode);
}

/*
float MotorController::get_error() {
    return pid_controller_.get_target() - current_position_;
}
*/

void MotorController::update(float sample_time) {

    // Read encoder values
    uint32_t encoder_value = htim_encoder_->Instance->CNT;
    bool encoder_a = HAL_GPIO_ReadPin(ENCODER_M1_A_PORT, pin_encoder_a_);
    bool encoder_b = HAL_GPIO_ReadPin(ENCODER_M1_B_PORT, pin_encoder_a_);
    // Update PID controller with current position and time delta
    pid_controller_.update(current_position_, sample_time);

    // Compute motor output based on PID controller output
    current_output_ = pid_controller_.get_output();

    // Update motor direction based on output sign
    set_direction(current_output_ >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Convert output magnitude to PWM duty cycle and write to PWM pin
    uint32_t duty_cycle = (uint32_t)(fabs(current_output_) / MAX_OUTPUT * UINT16_MAX);
    __HAL_TIM_SET_COMPARE(htim_pwm_, channel_pwm_, duty_cycle);

    // Update current position based on encoder values and direction
    if (encoder_a == 0 && encoder_b == 0) {
        current_position_--;
    } else if (encoder_a == 0 && encoder_b == 1) {
        current_position_++;
    } else if (encoder_a == 1 && encoder_b == 0) {
        current_position_++;
    } else {
        current_position_--;
    }
}

void MotorController::set_output(float output) {
    current_output_ = output;
    uint32_t duty_cycle = (uint32_t)(fabs(output) / MAX_OUTPUT * UINT16_MAX);
    __HAL_TIM_SET_COMPARE(htim_pwm_, channel_pwm_, duty_cycle);
}