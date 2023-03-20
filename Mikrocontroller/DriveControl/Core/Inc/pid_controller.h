#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    enum Mode {
        POSITION_CONTROL=0,
        SPEED_CONTROL=1
    };

    PIDController(float kp, float ki, float kd, float max_output, float max_integral, float target);

    void set_target(float target);

    float get_target();

    float compute(float input);

    void update(float error, float sample_time);

    float get_output();

    void set_mode(Mode mode);

    void update_position_control(float input, float sample_time);
    void update_speed_control(float input, float sample_time);


private:
    float kp_;
    float ki_;
    float kd_;
    float max_output_;
    float max_integral_;
    float target_;

    float last_error_;
    float integral_;
    float current_output_;

    Mode mode_;
};

#endif // PID_CONTROLLER_H
