#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float max_output, float max_integral, float target);

    void set_mode(int mode);
    void set_target(float target);
    float compute(float input);
    void update(float error, float sample_time);
    float get_output();


private:
    float kp_;
    float ki_;
    float kd_;
    float max_output_;
    float max_integral_;
    float target_;
    int mode_;
    float integral_;
    float last_error_;
    float current_output_;
};

enum PIDMode {
    PID_MODE_FREEWHEEL,
    PID_MODE_POSITION,
    PID_MODE_VELOCITY
};

#endif
