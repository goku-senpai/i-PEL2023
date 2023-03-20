#include "pid_controller.h"
#include "constants.h"

PIDController::PIDController(float kp, float ki, float kd, float max_output, float max_integral, float target)
        : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output), max_integral_(max_integral), target_(target), mode_((Mode)PID_MODE_POSITION_CONTROL) {
}

void PIDController::set_target(float target) {
    this->target_ = target;
}

float PIDController::get_target() {
    return this->target_;
}

void PIDController::set_gains(float kp, float ki, float kd) {
    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;
}

float PIDController::compute(float input) {
    float error = this->target_ - input;
    float derivative = error - this->last_error_;
    this->last_error_ = error;
    this->integral_ += error;

    // Limit integral term to prevent windup
    if (this->integral_ > this->max_integral_) {
        this->integral_ = this->max_integral_;
    } else if (this->integral_ < -this->max_integral_) {
        this->integral_ = -this->max_integral_;
    }

    // Calculate PID output
    float output = this->kp_ * error + this->ki_ * this->integral_ + this->kd_ * derivative;

    // Limit output to prevent saturation
    if (output > this->max_output_) {
        output = this->max_output_;
    } else if (output < -this->max_output_) {
        output = -this->max_output_;
    }

    return output;
}

void PIDController::update(float input, float sample_time) {
    if (mode_ == POSITION_CONTROL) {
        update_position_control(input, sample_time);
    } else if (mode_ == SPEED_CONTROL) {
        update_speed_control(input, sample_time);
    }
}

void PIDController::update_position_control(float input, float sample_time) {
    float error = this->target_ - input;
    update(error, sample_time);
}

void PIDController::update_speed_control(float input, float sample_time) {
    float error = this->target_ - input;
    float output = compute(error);
    float setpoint = this->target_ + output;
    update(setpoint - input, sample_time);
}

float PIDController::get_output() {
    return current_output_;
}

void PIDController::set_mode(Mode mode) {
    this->mode_ = (Mode)mode;
}
