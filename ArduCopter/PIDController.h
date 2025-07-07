#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H



class MY_PIDController {
public:
    MY_PIDController(float _kp, float _ki, float _kd, float _max_output, float _ramp_rate);
    float compute(float setpoint, float measured_value, float dt);

private:
    float kp;
    float ki;
    float kd;
    float max_output;
    float ramp_rate;
    float previous_error;
    float integral;
    float last_output;
};

#endif // PIDCONTROLLER_H