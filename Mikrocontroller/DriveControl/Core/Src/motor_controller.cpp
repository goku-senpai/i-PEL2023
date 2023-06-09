#include "motor_controller.h"
#include "cmath"

MotorController::MotorController(TIM_HandleTypeDef* htim_pwm, uint32_t channel_pwm, GPIO_TypeDef* gpio_dir,
                                 float pos_kp, float pos_ki, float pos_kd, float max_output, float max_integral,
                                 float target_start, uint32_t pin_direction, bool is_position_controller)
        : htim_pwm_(htim_pwm), channel_pwm_(channel_pwm), gpio_dir_(gpio_dir), pin_direction_(pin_direction),
          pid_controller_(pos_kp, pos_ki, pos_kd, max_output, max_integral, target_start),
          current_position_(0), current_output_(0), is_position_controller_(is_position_controller) {
}

void MotorController::set_direction(uint8_t direction) {
    HAL_GPIO_WritePin(gpio_dir_, pin_direction_, static_cast<GPIO_PinState>(direction));
}


void MotorController::update(float sample_time, int32_t encoder) {
    // Read encoder values
    uint32_t encoder_value = encoder;

    if (is_position_controller_) {
        // Update PID controller with current position and time delta
        if (sample_time <= 0.0) {
            return;
        }
        else {
            pid_controller_.update(current_position_, sample_time);
        }

        // Compute motor output based on PID controller output
        current_output_ = pid_controller_.get_output();
    } else {
        // Update PID controller with current speed and time delta
        float current_speed = (encoder_value - prev_encoder_value_) / (sample_time * ENCODER_RESOLUTION);
        pid_controller_.update(current_speed, sample_time);

        // Compute motor output based on PID controller output and current speed
        //SPEED_KF is a constant that represents a scaling factor used to convert the speed of the motor (in encoder ticks per second) to a corresponding output value for the motor controller
        current_output_ = pid_controller_.get_output() + current_speed * SPEED_KF;
    }

    // Update motor direction based on output sign
    set_direction(current_output_ >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Convert output magnitude to PWM duty cycle and write to PWM pin
    uint32_t duty_cycle = (uint32_t)(fabs(current_output_) / MAX_OUTPUT * UINT16_MAX);
    __HAL_TIM_SET_COMPARE(htim_pwm_, channel_pwm_, duty_cycle);
    // Update previous encoder value for speed calculation
    prev_encoder_value_ = encoder_value;
}


void MotorController::set_output(float output) {
    current_output_ = output;
    uint32_t duty_cycle = (uint32_t)(fabs(output) / MAX_OUTPUT * UINT16_MAX);
    __HAL_TIM_SET_COMPARE(htim_pwm_, channel_pwm_, duty_cycle);
}