#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd, float max_output, float max_integral, float target)
        : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output), integral_(max_integral), target_(target) {
     this->mode_ = PID_MODE_FREEWHEEL;
}
/*
void PIDController::set_setpoint(float setpoint) {
    this->setpoint = setpoint;
}
*/



void PIDController::set_target(float target) {
    this->target_ = target;
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



void PIDController::update(float error, float sample_time) {
    float derivative = (error - last_error_) / sample_time;
    last_error_ = error;
    integral_ += error * sample_time;

    // Limit integral term to prevent windup
    if (integral_ > max_integral_) {
        integral_ = max_integral_;
    } else if (integral_ < -max_integral_) {
        integral_ = -max_integral_;
    }

    // Calculate PID output
    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // Limit output to prevent saturation
    if (output > max_output_) {
        output = max_output_;
    } else if (output < -max_output_) {
        output = -max_output_;
    }

     current_output_ = output;
}

float PIDController::get_output() {
    return current_output_;
}


void PIDController::set_mode(int mode) {

}
