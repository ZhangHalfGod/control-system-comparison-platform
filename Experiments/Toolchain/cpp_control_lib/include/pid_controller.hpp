// PID controller definition
#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

#include "controller_base.hpp"

// Position PID controller
class PIDController : public Controller {
public:
    // Constructor
    PIDController(double kp_val, double ki_val, double kd_val, double dt_val);
    
    // Calculate PID output
    double compute(double setpoint, double process_val) override;
    
    // Reset controller
    void reset() override;

private:
    double kp;
    double ki;
    double kd;
    double dt;
    double prev_err;
    double integral;
    double prev_pv;
};

// Incremental PID controller
class IncrementalPIDController : public Controller {
public:
    // Constructor
    IncrementalPIDController(double kp_val, double ki_val, double kd_val, double dt_val);
    
    // Calculate PID output
    double compute(double setpoint, double process_val) override;
    
    // Reset controller
    void reset() override;

private:
    double kp;
    double ki;
    double kd;
    double dt;
    double prev_err1;
    double prev_err2;
};

// Fuzzy PID controller
class FuzzyPIDController : public Controller {
public:
    // Constructor
    FuzzyPIDController(double kp_val, double ki_val, double kd_val, double dt_val);
    
    // Calculate PID output
    double compute(double setpoint, double process_val) override;
    
    // Reset controller
    void reset() override;

private:
    // Fuzzy variable enumeration
    enum FuzzyVariable {
        NB = -3,  // Negative Big
        NM = -2,  // Negative Medium
        NS = -1,  // Negative Small
        ZO = 0,   // Zero
        PS = 1,   // Positive Small
        PM = 2,   // Positive Medium
        PB = 3    // Positive Big
    };
    
    // Calculate membership function - Gaussian function
    double gaussian(double x, double mean, double sigma);
    
    // Fuzzification - convert precise value to fuzzy value
    void fuzzify(double err, double err_dot, FuzzyVariable& fuzzy_err, FuzzyVariable& fuzzy_err_dot);
    
    // Fuzzy rule table - determine PID parameter adjustment based on error and error change rate
    void getFuzzyRules(FuzzyVariable fuzzy_err, FuzzyVariable fuzzy_err_dot, 
                     FuzzyVariable& delta_kp, FuzzyVariable& delta_ki, FuzzyVariable& delta_kd);
    
    // Defuzzification - convert fuzzy value to precise value
    double defuzzify(FuzzyVariable fuzzy_value, double range);
    
    // Calculate error change rate
    double calculateErrorDot(double err, double prev_err);
    
    // PID parameters
    double kp;
    double ki;
    double kd;
    double dt;
    
    // Error history
    double prev_err;
    double prev_pv;
    double integral;
    
    // Fuzzy rule database
    static const int rule_table_size = 7;
    int delta_kp_rules[7][7];
    int delta_ki_rules[7][7];
    int delta_kd_rules[7][7];
};

// Adaptive PID controller - using MIT rule
class AdaptivePIDController : public Controller {
public:
    // Constructor
    AdaptivePIDController(double kp_val, double ki_val, double kd_val, double dt_val, double gamma_val = 0.01);
    
    // Calculate PID output
    double compute(double setpoint, double process_val) override;
    
    // Reset controller
    void reset() override;

private:
    // PID parameters
    double kp;
    double ki;
    double kd;
    double dt;
    
    // Adaptive gain
    double gamma;
    
    // Error history
    double prev_err;
    double prev_pv;
    double integral;
    
    // Used to calculate partial derivatives of error with respect to parameters
    double prev_u;
    double prev_process_val;
    
    // Performance index calculation - simple implementation: error squared
    double performance_index(double err);
    
    // Calculate partial derivative of error with respect to Kp
    double d_err_d_kp(double err, double prev_err, double process_val, double prev_process_val);
    
    // Calculate partial derivative of error with respect to Ki
    double d_err_d_ki(double err, double prev_err, double process_val, double prev_process_val);
    
    // Calculate partial derivative of error with respect to Kd
    double d_err_d_kd(double err, double prev_err, double process_val, double prev_process_val);
};

#endif // _PID_CONTROLLER_H_