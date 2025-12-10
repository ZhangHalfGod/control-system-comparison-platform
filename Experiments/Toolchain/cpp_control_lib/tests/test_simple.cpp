#include <stdio.h>
#include <math.h>
#include "pid_controller.hpp"

// Performance metrics structure
typedef struct {
    double overshoot;        // Overshoot percentage
    double settling_time;    // Settling time in seconds
    double steady_state_err; // Steady state error
    double rise_time;        // Rise time in seconds
} PIDPerformance;

// Calculate performance metrics
PIDPerformance calculate_performance(double* process_vals, int steps, double setpoint) {
    PIDPerformance perf = {0, 0, 0, -1};
    double max_val = process_vals[0];
    int settling_idx = -1;
    int rise_idx = -1;
    
    // Find maximum value (for overshoot calculation)
    for (int i = 0; i < steps; i++) {
        if (process_vals[i] > max_val) {
            max_val = process_vals[i];
        }
        
        // Find rise time (first time reaching 90% of setpoint)
        if (rise_idx == -1 && process_vals[i] >= setpoint * 0.9) {
            rise_idx = i;
        }
        
        // Find settling time (enters ±2% error band and stays)
        if (settling_idx == -1 && i > steps/2) {
            int stable = 1;
            for (int j = i; j < steps; j++) {
                if (fabs(process_vals[j] - setpoint) > setpoint * 0.02) {
                    stable = 0;
                    break;
                }
            }
            if (stable) {
                settling_idx = i;
            }
        }
    }
    
    // Calculate overshoot
    if (max_val > setpoint) {
        perf.overshoot = (max_val - setpoint) / setpoint * 100;
    }
    
    // Calculate rise time
    if (rise_idx != -1) {
        perf.rise_time = rise_idx * 0.01; // Assume sampling time 0.01s
    }
    
    // Calculate settling time
    if (settling_idx != -1) {
        perf.settling_time = settling_idx * 0.01;
    }
    
    // Calculate steady state error
    perf.steady_state_err = fabs(process_vals[steps-1] - setpoint);
    
    return perf;
}

int main() {
    printf("PID Parameter Optimization Experiment\n");
    
    // [Parameter Tuning Area] Modify PID parameters here
    double kp = 7.0;    // Proportional gain - Increased for faster response
    double ki = 5.0;    // Integral gain - Significantly increased to eliminate steady state error
    double kd = 0.5;    // Derivative gain - Increased to control overshoot
    double dt = 0.01;   // Sampling time
    
    PIDController pid(kp, ki, kd, dt);
    
    double setpoint = 1.0;
    double process_val = 0.0;
    double process_vals[1000] = {0}; // Store process values for performance calculation
    double output_vals[1000] = {0};  // Store controller output values
    
    int steps = 200; // Increase simulation steps to observe steady state response
    
    printf("Parameters: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", kp, ki, kd);
    printf("Setpoint: %.2f\n", setpoint);
    printf("========================================\n");
    
    // Simulation loop
    for (int i = 0; i < steps; i++) {
        double output = pid.compute(setpoint, process_val);
        process_vals[i] = process_val;
        output_vals[i] = output;
        
        // Simple first-order system model: G(s) = 1/(s+1)
        process_val += dt * (output - process_val);
        
        // Output every 10 steps
        if (i % 10 == 0) {
            printf("Step %3d: Process=%.4f, Output=%.4f\n", i, process_val, output);
        }
    }
    
    // Calculate performance metrics
    PIDPerformance perf = calculate_performance(process_vals, steps, setpoint);
    printf("========================================\n");
    printf("Performance Metrics:\n");
    printf("Overshoot: %.2f%%\n", perf.overshoot);
    printf("Rise Time: %.2fs\n", perf.rise_time);
    printf("Settling Time: %.2fs\n", perf.settling_time);
    printf("Steady State Error: %.4f\n", perf.steady_state_err);
    
    // Generate data file for visualization
    FILE *fp = fopen("pid_response_data.txt", "w");
    if (fp != NULL) {
        // Write header
        fprintf(fp, "Step,Time,ProcessValue,Setpoint,ControllerOutput\n");
        
        // Write data
        for (int i = 0; i < steps; i++) {
            double time = i * dt;
            fprintf(fp, "%d,%.3f,%.6f,%.1f,%.6f\n", i, time, process_vals[i], setpoint, output_vals[i]);
        }
        
        fclose(fp);
        printf("========================================\n");
        printf("Data file generated: pid_response_data.txt\n");
        printf("You can use this file to plot response curves in MATLAB or Python.\n");
    }
    
    return 0;
}