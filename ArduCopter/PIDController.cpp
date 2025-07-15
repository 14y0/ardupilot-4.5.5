#include "PIDController.h"
#include <cmath> 

//#include <algorithm> // for std::clamp


MY_PIDController::MY_PIDController(float _kp, float _ki, float _kd, float _max_output, float _ramp_rate)
    : kp(_kp), ki(_ki), kd(_kd), max_output(_max_output), ramp_rate(_ramp_rate),
      previous_error(0), integral(0), last_output(0) {}

float MY_PIDController::compute(float setpoint, float measured_value, float dt) {
    float error = setpoint - measured_value;
    integral += error * dt;
    float derivative = (error - previous_error) / dt;
    float output = kp * error + ki * integral + kd * derivative;
    if(output>max_output)output=max_output;
    if(output<-max_output)output=-max_output;
    if (fabsf(output - last_output) > ramp_rate) {
        if (output > last_output) {
            output = last_output + ramp_rate;
        } else {
            output = last_output - ramp_rate;
        }
    }
    previous_error = error;
    last_output = output;
    return output;
}
