// 极简源文件实现
#include "pid_controller.hpp"

PIDController::PIDController(double kp_val, double ki_val, double kd_val, double dt_val) {
    kp = kp_val;
    ki = ki_val;
    kd = kd_val;
    dt = dt_val;
    prev_err = 0.0;
    integral = 0.0;
    prev_pv = 0.0;
}

double PIDController::compute(double setpoint, double process_val) {
    double err = setpoint - process_val;
    double p_term = kp * err;
    
    integral += ki * err * dt;
    double i_term = integral;
    
    double d_term = kd * (process_val - prev_pv) / dt;
    
    double output = p_term + i_term - d_term;
    
    prev_err = err;
    prev_pv = process_val;
    
    return output;
}

void PIDController::reset() {
    prev_err = 0.0;
    integral = 0.0;
    prev_pv = 0.0;
}