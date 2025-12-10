// Controller Implementation
#include "pid_controller.hpp"
#include <cmath>

// Position PID Controller Implementation
PIDController::PIDController(double kp_val, double ki_val, double kd_val, double dt_val)
    : kp(kp_val), ki(ki_val), kd(kd_val), dt(dt_val), prev_err(0.0), integral(0.0), prev_pv(0.0)
{
}

double PIDController::compute(double setpoint, double process_val)
{
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

void PIDController::reset()
{
    prev_err = 0.0;
    integral = 0.0;
    prev_pv = 0.0;
}

// Incremental PID Controller Implementation
IncrementalPIDController::IncrementalPIDController(double kp_val, double ki_val, double kd_val, double dt_val)
    : kp(kp_val), ki(ki_val), kd(kd_val), dt(dt_val), prev_err1(0.0), prev_err2(0.0)
{
}

double IncrementalPIDController::compute(double setpoint, double process_val)
{
    double err = setpoint - process_val;
    double delta_u = kp * (err - prev_err1) + ki * err * dt + kd * (err - 2.0 * prev_err1 + prev_err2) / dt;
    
    prev_err2 = prev_err1;
    prev_err1 = err;
    
    return delta_u;
}

void IncrementalPIDController::reset()
{
    prev_err1 = 0.0;
    prev_err2 = 0.0;
}

// Fuzzy PID Controller Implementation

FuzzyPIDController::FuzzyPIDController(double kp_val, double ki_val, double kd_val, double dt_val)
    : kp(kp_val), ki(ki_val), kd(kd_val), dt(dt_val), prev_err(0.0), prev_pv(0.0), integral(0.0)
{
    // Optimized fuzzy rule tables for better performance
    // Rule table values: -3=NB, -2=NM, -1=NS, 0=ZO, 1=PS, 2=PM, 3=PB
    int kp_rules[7][7] = {
        {3, 3, 2, 2, 2, 1, 0}, 
        {3, 3, 2, 2, 1, 1, -1}, 
        {2, 2, 2, 1, 0, -1, -2}, 
        {2, 1, 0, -1, -1, -2, -2}, 
        {1, 1, 0, -1, -2, -2, -2}, 
        {1, 0, -1, -2, -2, -3, -3}, 
        {0, 0, -1, -2, -3, -3, -3}
    };
    
    int ki_rules[7][7] = {
        {-3, -3, -3, -2, -2, -1, 0}, 
        {-3, -3, -2, -2, -1, 0, 0}, 
        {-2, -2, -1, -1, 0, 1, 1}, 
        {-2, -1, 0, 1, 1, 2, 2}, 
        {-1, 0, 1, 1, 2, 2, 3}, 
        {0, 0, 1, 2, 2, 3, 3}, 
        {0, 1, 2, 2, 3, 3, 3}
    };
    
    int kd_rules[7][7] = {
        {2, 1, -1, -2, -2, -2, 0}, 
        {2, 1, -1, -2, -2, -1, 0}, 
        {1, 1, -1, -1, -1, -1, 0}, 
        {1, 0, 0, 0, 0, 0, 0}, 
        {0, 0, 0, 0, 0, 0, 0}, 
        {0, 1, 1, 1, 1, 1, 2}, 
        {0, 2, 2, 2, 1, 1, 2}
    };
    
    // Copy rule tables to member variables
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            delta_kp_rules[i][j] = kp_rules[i][j];
            delta_ki_rules[i][j] = ki_rules[i][j];
            delta_kd_rules[i][j] = kd_rules[i][j];
        }
    }
}

double FuzzyPIDController::gaussian(double x, double mean, double sigma)
{
    return exp(-pow(x - mean, 2) / (2 * pow(sigma, 2)));
}

void FuzzyPIDController::fuzzify(double err, double err_dot, FuzzyVariable& fuzzy_err, FuzzyVariable& fuzzy_err_dot)
{
    // Fuzzify error
    if (err <= -1.5)
    {
        fuzzy_err = NB;
    }
    else if (err <= -0.5)
    {
        fuzzy_err = NM;
    }
    else if (err <= -0.1)
    {
        fuzzy_err = NS;
    }
    else if (err <= 0.1)
    {
        fuzzy_err = ZO;
    }
    else if (err <= 0.5)
    {
        fuzzy_err = PS;
    }
    else if (err <= 1.5)
    {
        fuzzy_err = PM;
    }
    else
    {
        fuzzy_err = PB;
    }
    
    // Fuzzify error derivative
    if (err_dot <= -1.5)
    {
        fuzzy_err_dot = NB;
    }
    else if (err_dot <= -0.5)
    {
        fuzzy_err_dot = NM;
    }
    else if (err_dot <= -0.1)
    {
        fuzzy_err_dot = NS;
    }
    else if (err_dot <= 0.1)
    {
        fuzzy_err_dot = ZO;
    }
    else if (err_dot <= 0.5)
    {
        fuzzy_err_dot = PS;
    }
    else if (err_dot <= 1.5)
    {
        fuzzy_err_dot = PM;
    }
    else
    {
        fuzzy_err_dot = PB;
    }
}

void FuzzyPIDController::getFuzzyRules(FuzzyVariable fuzzy_err, FuzzyVariable fuzzy_err_dot, FuzzyVariable& delta_kp, FuzzyVariable& delta_ki, FuzzyVariable& delta_kd)
{
    // Convert fuzzy variables to array indices
    int err_idx = fuzzy_err + 3;
    int err_dot_idx = fuzzy_err_dot + 3;
    
    // Find corresponding PID parameter adjustments from rule tables
    delta_kp = static_cast<FuzzyVariable>(delta_kp_rules[err_idx][err_dot_idx]);
    delta_ki = static_cast<FuzzyVariable>(delta_ki_rules[err_idx][err_dot_idx]);
    delta_kd = static_cast<FuzzyVariable>(delta_kd_rules[err_idx][err_dot_idx]);
}

double FuzzyPIDController::defuzzify(FuzzyVariable fuzzy_value, double range)
{
    return fuzzy_value * (range / 3.0);
}

double FuzzyPIDController::calculateErrorDot(double err, double prev_err)
{
    return (err - prev_err) / dt;
}

double FuzzyPIDController::compute(double setpoint, double process_val)
{
    // Calculate current error
    double err = setpoint - process_val;
    
    // Calculate error derivative
    double err_dot_val = calculateErrorDot(err, this->prev_err);
    
    // Fuzzification
    FuzzyVariable fuzzy_err, fuzzy_err_dot;
    fuzzify(err, err_dot_val, fuzzy_err, fuzzy_err_dot);
    
    // Fuzzy rule inference
    FuzzyVariable delta_kp, delta_ki, delta_kd;
    getFuzzyRules(fuzzy_err, fuzzy_err_dot, delta_kp, delta_ki, delta_kd);
    
    // Defuzzification
    const double kp_range = 0.5;
    const double ki_range = 0.1;
    const double kd_range = 0.2;
    
    double dkp = defuzzify(delta_kp, kp_range);
    double dki = defuzzify(delta_ki, ki_range);
    double dkd = defuzzify(delta_kd, kd_range);
    
    // Dynamically adjust PID parameters
    double current_kp = kp + dkp;
    double current_ki = ki + dki;
    double current_kd = kd + dkd;
    
    // Calculate output using position PID algorithm
    double p_term = current_kp * err;
    integral += current_ki * err * dt;
    double i_term = integral;
    double d_term = current_kd * (process_val - prev_pv) / dt;
    
    double output = p_term + i_term - d_term;
    
    // Update history values
    this->prev_err = err;
    prev_pv = process_val;
    
    return output;
}

void FuzzyPIDController::reset()
{
    prev_err = 0.0;
    prev_pv = 0.0;
    integral = 0.0;
}

// Adaptive PID Controller Implementation
AdaptivePIDController::AdaptivePIDController(double kp_val, double ki_val, double kd_val, double dt_val, double gamma_val)
    : kp(kp_val), ki(ki_val), kd(kd_val), dt(dt_val), gamma(gamma_val), prev_err(0.0), prev_pv(0.0), integral(0.0), prev_u(0.0), prev_process_val(0.0)
{
}

double AdaptivePIDController::performance_index(double err)
{
    return 0.5 * err * err;
}

double AdaptivePIDController::d_err_d_kp(double err, double prev_err, double process_val, double prev_process_val)
{
    return -err * prev_err;
}

double AdaptivePIDController::d_err_d_ki(double err, double prev_err, double process_val, double prev_process_val)
{
    return -err * integral;
}

double AdaptivePIDController::d_err_d_kd(double err, double prev_err, double process_val, double prev_process_val)
{
    return -err * (process_val - 2 * prev_process_val + prev_pv);
}

double AdaptivePIDController::compute(double setpoint, double process_val)
{
    // Calculate current error
    double err = setpoint - process_val;
    
    // Calculate PID output
    double p_term = kp * err;
    integral += ki * err * dt;
    double i_term = integral;
    double d_term = kd * (process_val - prev_pv) / dt;
    double u = p_term + i_term - d_term;
    
    // Calculate performance index
    double J = performance_index(err);
    
    // Calculate partial derivatives of error with respect to PID parameters
    double dJ_d_kp = d_err_d_kp(err, prev_err, process_val, prev_process_val);
    double dJ_d_ki = d_err_d_ki(err, prev_err, process_val, prev_process_val);
    double dJ_d_kd = d_err_d_kd(err, prev_err, process_val, prev_process_val);
    
    // Adaptive learning rate: decrease as error decreases for better convergence
    double abs_err = fabs(err);
    double adaptive_gamma = gamma;
    
    // Adjust learning rate based on error magnitude
    if (abs_err < 0.1) {
        adaptive_gamma = gamma * 0.1;  // Slow down learning when close to setpoint
    } else if (abs_err < 0.5) {
        adaptive_gamma = gamma * 0.5;  // Medium learning rate
    } else {
        adaptive_gamma = gamma;        // Fast learning for large errors
    }
    
    // Adjust PID parameters using MIT rule with adaptive learning rate
    kp -= adaptive_gamma * dJ_d_kp;
    ki -= adaptive_gamma * dJ_d_ki;
    kd -= adaptive_gamma * dJ_d_kd;
    
    // Enhanced parameter limits with symmetric ranges for better performance
    const double min_kp = 0.0;
    const double max_kp = 15.0;      // Increased upper limit for more aggressive control
    const double min_ki = 0.0;
    const double max_ki = 3.0;       // Increased upper limit for better steady-state accuracy
    const double min_kd = 0.0;
    const double max_kd = 2.0;       // Increased upper limit for better transient response
    
    // Limit parameter values
    if (kp < min_kp) kp = min_kp;
    if (kp > max_kp) kp = max_kp;
    if (ki < min_ki) ki = min_ki;
    if (ki > max_ki) ki = max_ki;
    if (kd < min_kd) kd = min_kd;
    if (kd > max_kd) kd = max_kd;
    
    // Update history values
    prev_err = err;
    prev_pv = process_val;
    prev_u = u;
    prev_process_val = process_val;
    
    return u;
}

void AdaptivePIDController::reset()
{
    prev_err = 0.0;
    prev_pv = 0.0;
    integral = 0.0;
    prev_u = 0.0;
    prev_process_val = 0.0;
}