// Test all controller types
#define _USE_MATH_DEFINES  // Enable math constants like M_PI
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <ctime>  // For time function
#include "pid_controller.hpp"

// System model base class
class SystemModel {
public:
    virtual ~SystemModel() = default;
    virtual double compute(double input) = 0;
    virtual void reset() = 0;
    virtual std::string get_name() const = 0;
};

// First-order system model
class FirstOrderSystem : public SystemModel {
private:
    double T;  // Time constant
    double K;  // Gain
    double prev_input;  // Previous input
    double prev_output;  // Previous output
    double dt;  // Sampling time

public:
    // Constructor
    FirstOrderSystem(double T_val, double K_val, double dt_val)
        : T(T_val), K(K_val), dt(dt_val), prev_input(0.0), prev_output(0.0) {}
    
    // Calculate system output
    double compute(double input) override {
        // Use Euler method to solve first-order differential equation
        // dx/dt = (K*u - x)/T
        double output = prev_output + (dt / T) * (K * input - prev_output);
        
        // Update history values
        prev_input = input;
        prev_output = output;
        
        return output;
    }
    
    // Reset system
    void reset() override {
        prev_input = 0.0;
        prev_output = 0.0;
    }
    
    // Get model name
    std::string get_name() const override {
        return "FirstOrderSystem";
    }
};

// Second-order system model
class SecondOrderSystem : public SystemModel {
private:
    double K;  // Gain
    double zeta;  // Damping ratio
    double omega_n;  // Natural frequency
    double dt;  // Sampling time
    double prev_input;  // Previous input
    double prev_output;  // Previous output
    double prev_derivative;  // Previous derivative

public:
    // Constructor
    SecondOrderSystem(double K_val, double zeta_val, double omega_n_val, double dt_val)
        : K(K_val), zeta(zeta_val), omega_n(omega_n_val), dt(dt_val),
          prev_input(0.0), prev_output(0.0), prev_derivative(0.0) {}
    
    // Calculate system output
    double compute(double input) override {
        // Second-order system: G(s) = K * omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2)
        // Using Euler method for numerical integration
        double acceleration = K * omega_n * omega_n * input - 
                             2 * zeta * omega_n * prev_derivative - 
                             omega_n * omega_n * prev_output;
        
        double derivative = prev_derivative + acceleration * dt;
        double output = prev_output + derivative * dt;
        
        // Update history values
        prev_input = input;
        prev_output = output;
        prev_derivative = derivative;
        
        return output;
    }
    
    // Reset system
    void reset() override {
        prev_input = 0.0;
        prev_output = 0.0;
        prev_derivative = 0.0;
    }
    
    // Get model name
    std::string get_name() const override {
        return "SecondOrderSystem";
    }
};

// Nonlinear system model (saturation + deadzone)
class NonlinearSystem : public SystemModel {
private:
    double K;  // Gain
    double T;  // Time constant
    double saturation_limit;  // Saturation limit
    double deadzone_width;  // Deadzone width
    double dt;  // Sampling time
    double prev_input;  // Previous input
    double prev_output;  // Previous output

public:
    // Constructor
    NonlinearSystem(double K_val, double T_val, double saturation_limit_val, 
                   double deadzone_width_val, double dt_val)
        : K(K_val), T(T_val), saturation_limit(saturation_limit_val),
          deadzone_width(deadzone_width_val), dt(dt_val),
          prev_input(0.0), prev_output(0.0) {}
    
    // Apply saturation nonlinearity
    double apply_saturation(double input) {
        if (input > saturation_limit) {
            return saturation_limit;
        } else if (input < -saturation_limit) {
            return -saturation_limit;
        }
        return input;
    }
    
    // Apply deadzone nonlinearity
    double apply_deadzone(double input) {
        if (input > deadzone_width) {
            return input - deadzone_width;
        } else if (input < -deadzone_width) {
            return input + deadzone_width;
        }
        return 0.0;
    }
    
    // Calculate system output
    double compute(double input) override {
        // Apply nonlinearities
        double nonlinear_input = apply_saturation(input);
        nonlinear_input = apply_deadzone(nonlinear_input);
        
        // First-order dynamics with nonlinear input
        double output = prev_output + (dt / T) * (K * nonlinear_input - prev_output);
        
        // Update history values
        prev_input = input;
        prev_output = output;
        
        return output;
    }
    
    // Reset system
    void reset() override {
        prev_input = 0.0;
        prev_output = 0.0;
    }
    
    // Get model name
    std::string get_name() const override {
        return "NonlinearSystem";
    }
};

// Noise generator class
class NoiseGenerator {
private:
    double mean;  // Mean value for Gaussian noise
    double std_dev;  // Standard deviation for Gaussian noise
    double max_impulse_amplitude;  // Maximum amplitude for impulse noise
    double impulse_probability;  // Probability of impulse noise

public:
    // Constructor
    NoiseGenerator(double mean_val = 0.0, double std_dev_val = 0.1, 
                  double max_impulse_amp = 1.0, double impulse_prob = 0.01)
        : mean(mean_val), std_dev(std_dev_val), 
          max_impulse_amplitude(max_impulse_amp), impulse_probability(impulse_prob) {
        // Seed the random number generator
        srand(static_cast<unsigned int>(time(nullptr)));
    }
    
    // Generate Gaussian noise
    double generate_gaussian_noise() {
        // Simple Box-Muller transform for Gaussian noise
        static bool has_spare = false;
        static double z1;
        
        if (has_spare) {
            has_spare = false;
            return mean + std_dev * z1;
        }
        
        has_spare = true;
        double u1 = static_cast<double>(rand()) / RAND_MAX;
        double u2 = static_cast<double>(rand()) / RAND_MAX;
        
        double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
        z1 = sqrt(-2.0 * log(u1)) * sin(2.0 * M_PI * u2);
        
        return mean + std_dev * z0;
    }
    
    // Generate impulse noise
    double generate_impulse_noise() {
        double prob = static_cast<double>(rand()) / RAND_MAX;
        if (prob < impulse_probability) {
            // Generate random impulse amplitude
            double amplitude = (static_cast<double>(rand()) / RAND_MAX - 0.5) * 2.0 * max_impulse_amplitude;
            return amplitude;
        }
        return 0.0;
    }
    
    // Generate combined noise (Gaussian + impulse)
    double generate_combined_noise() {
        return generate_gaussian_noise() + generate_impulse_noise();
    }
    
    // Reset random seed
    void reset_seed(unsigned int new_seed = 0) {
        if (new_seed == 0) {
            srand(static_cast<unsigned int>(time(nullptr)));
        } else {
            srand(new_seed);
        }
    }
};

// Performance metrics calculation class
class PerformanceMetrics {
private:
    std::vector<double> time;
    std::vector<double> setpoint;
    std::vector<double> process_val;
    std::vector<double> output;
    
public:
    // Add data point
    void add_data_point(double t, double sp, double pv, double u) {
        time.push_back(t);
        setpoint.push_back(sp);
        process_val.push_back(pv);
        output.push_back(u);
    }
    
    // Calculate rise time (from 10% to 90% of final value)
    double calculate_rise_time() {
        if (time.empty() || process_val.empty()) return 0.0;
        
        double final_value = setpoint.back();
        double low_threshold = 0.1 * final_value;
        double high_threshold = 0.9 * final_value;
        
        bool low_crossed = false;
        double rise_time = 0.0;
        
        for (size_t i = 0; i < process_val.size(); ++i) {
            if (!low_crossed && process_val[i] >= low_threshold) {
                low_crossed = true;
                continue;
            }
            
            if (low_crossed && process_val[i] >= high_threshold) {
                rise_time = time[i];
                break;
            }
        }
        
        return rise_time;
    }
    
    // Calculate overshoot percentage
    double calculate_overshoot() {
        if (time.empty() || process_val.empty()) return 0.0;
        
        double final_value = setpoint.back();
        double max_value = *std::max_element(process_val.begin(), process_val.end());
        
        if (max_value <= final_value) {
            return 0.0; // No overshoot
        }
        
        double overshoot = ((max_value - final_value) / final_value) * 100.0;
        return overshoot;
    }
    
    // Calculate settling time (time to enter 2% error band)
    double calculate_settling_time() {
        if (time.empty() || process_val.empty()) return 0.0;
        
        double final_value = setpoint.back();
        double tolerance = 0.02 * final_value;
        double lower_bound = final_value - tolerance;
        double upper_bound = final_value + tolerance;
        
        double settling_time = 0.0;
        bool settled = false;
        
        for (size_t i = 0; i < process_val.size(); ++i) {
            if (process_val[i] >= lower_bound && process_val[i] <= upper_bound) {
                // Check if all subsequent points are within error band
                settled = true;
                for (size_t j = i; j < process_val.size(); ++j) {
                    if (process_val[j] < lower_bound || process_val[j] > upper_bound) {
                        settled = false;
                        break;
                    }
                }
                
                if (settled) {
                    settling_time = time[i];
                    break;
                }
            }
        }
        
        return settling_time;
    }
    
    // Calculate steady state error
    double calculate_steady_state_error() {
        if (time.empty() || process_val.empty()) return 0.0;
        
        double final_value = setpoint.back();
        double steady_state_val = process_val.back();
        
        return std::abs(final_value - steady_state_val);
    }
    
    // Save data to file
    void save_to_file(const std::string& filename) {
        std::ofstream outfile(filename);
        if (!outfile.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return;
        }
        
        // Write header
        outfile << "time,setpoint,process_val,output\n";
        
        // Write data
        for (size_t i = 0; i < time.size(); ++i) {
            outfile << time[i] << "," << setpoint[i] << "," << process_val[i] << "," << output[i] << "\n";
        }
        
        outfile.close();
    }
    
    // Print performance metrics
    void print_metrics(const std::string& controller_name) {
        std::cout << "\n=== " << controller_name << " Performance Metrics ===" << std::endl;
        std::cout << "Rise Time: " << calculate_rise_time() << " seconds" << std::endl;
        std::cout << "Overshoot: " << calculate_overshoot() << "%" << std::endl;
        std::cout << "Settling Time: " << calculate_settling_time() << " seconds" << std::endl;
        std::cout << "Steady State Error: " << calculate_steady_state_error() << std::endl;
    }
};

// Test function to test all controllers with a given system model and noise configuration
void test_controllers(SystemModel& system, NoiseGenerator& noise_gen, const std::string& output_prefix, 
                     double dt, double simulation_time, double setpoint, 
                     double kp, double ki, double kd, bool add_noise = false) {
    int steps = static_cast<int>(simulation_time / dt);
    
    // Create controller instances
    PIDController pid(kp, ki, kd, dt);
    IncrementalPIDController incremental_pid(kp, ki, kd, dt);
    FuzzyPIDController fuzzy_pid(kp, ki, kd, dt);
    AdaptivePIDController adaptive_pid(kp, ki, kd, dt, 0.01);
    
    // Performance metrics objects
    PerformanceMetrics pid_metrics;
    PerformanceMetrics incremental_pid_metrics;
    PerformanceMetrics fuzzy_pid_metrics;
    PerformanceMetrics adaptive_pid_metrics;
    
    // Get system name
    std::string system_name = system.get_name();
    
    // Add noise information to output prefix
    std::string noise_suffix = add_noise ? "_with_noise" : "_no_noise";
    std::string full_prefix = output_prefix + system_name + noise_suffix + "_";
    
    std::cout << "\n=== Testing controllers with " << system_name << " (noise: " << (add_noise ? "enabled" : "disabled") << ") ===" << std::endl;
    
    // Test PID controller
    system.reset();
    double process_val_pid = 0.0;
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        double u = pid.compute(setpoint, process_val_pid);
        double system_output = system.compute(u);
        
        // Add noise if enabled
        if (add_noise) {
            double noise = noise_gen.generate_combined_noise();
            process_val_pid = system_output + noise;
        } else {
            process_val_pid = system_output;
        }
        
        pid_metrics.add_data_point(t, setpoint, process_val_pid, u);
    }
    
    // Test Incremental PID controller
    system.reset();
    double process_val_incremental = 0.0;
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        double u = incremental_pid.compute(setpoint, process_val_incremental);
        double system_output = system.compute(u);
        
        // Add noise if enabled
        if (add_noise) {
            double noise = noise_gen.generate_combined_noise();
            process_val_incremental = system_output + noise;
        } else {
            process_val_incremental = system_output;
        }
        
        incremental_pid_metrics.add_data_point(t, setpoint, process_val_incremental, u);
    }
    
    // Test Fuzzy PID controller
    system.reset();
    double process_val_fuzzy = 0.0;
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        double u = fuzzy_pid.compute(setpoint, process_val_fuzzy);
        double system_output = system.compute(u);
        
        // Add noise if enabled
        if (add_noise) {
            double noise = noise_gen.generate_combined_noise();
            process_val_fuzzy = system_output + noise;
        } else {
            process_val_fuzzy = system_output;
        }
        
        fuzzy_pid_metrics.add_data_point(t, setpoint, process_val_fuzzy, u);
    }
    
    // Test Adaptive PID controller
    system.reset();
    double process_val_adaptive = 0.0;
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        double u = adaptive_pid.compute(setpoint, process_val_adaptive);
        double system_output = system.compute(u);
        
        // Add noise if enabled
        if (add_noise) {
            double noise = noise_gen.generate_combined_noise();
            process_val_adaptive = system_output + noise;
        } else {
            process_val_adaptive = system_output;
        }
        
        adaptive_pid_metrics.add_data_point(t, setpoint, process_val_adaptive, u);
    }
    
    // Print performance metrics
    pid_metrics.print_metrics("Position PID Controller");
    incremental_pid_metrics.print_metrics("Incremental PID Controller");
    fuzzy_pid_metrics.print_metrics("Fuzzy PID Controller");
    adaptive_pid_metrics.print_metrics("Adaptive PID Controller");
    
    // Save data to files with prefix support
    pid_metrics.save_to_file(full_prefix + "pid_response.csv");
    incremental_pid_metrics.save_to_file(full_prefix + "incremental_pid_response.csv");
    fuzzy_pid_metrics.save_to_file(full_prefix + "fuzzy_pid_response.csv");
    adaptive_pid_metrics.save_to_file(full_prefix + "adaptive_pid_response.csv");
    
    std::cout << "\nData saved with prefix: " << full_prefix << std::endl;
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    std::string output_prefix = "";
    if (argc > 1) {
        output_prefix = argv[1];
        // Add underscore if prefix is not empty
        if (!output_prefix.empty()) {
            output_prefix += "_";
        }
    }
    
    // Simulation parameters for basic tests
    double dt = 0.1;  // Sampling time (seconds)
    double simulation_time = 20.0;  // Simulation time (seconds) for basic tests
    double long_simulation_time = 100.0;  // Simulation time for long-term stability test
    double setpoint = 1.0;  // Step input value
    
    // Initial PID parameters
    double kp = 0.5;
    double ki = 0.1;
    double kd = 0.05;
    
    // Create noise generator
    NoiseGenerator noise_gen(0.0, 0.05, 0.2, 0.02);  // Mean, std_dev, max_impulse, impulse_prob
    
    // Create system models
    FirstOrderSystem first_order_sys(1.0, 1.0, dt);  // Time constant, gain, dt
    SecondOrderSystem second_order_sys(1.0, 0.7, 1.0, dt);  // Gain, damping ratio, natural frequency, dt
    NonlinearSystem nonlinear_sys(1.0, 1.0, 0.5, 0.1, dt);  // Gain, time constant, saturation limit, deadzone width, dt
    
    std::cout << "=== Starting comprehensive PID controller tests ===" << std::endl;
    
    // Test 1: Basic tests without noise
    std::cout << "\n--- Test 1: Basic tests without noise ---" << std::endl;
    test_controllers(first_order_sys, noise_gen, output_prefix, dt, simulation_time, setpoint, kp, ki, kd, false);
    test_controllers(second_order_sys, noise_gen, output_prefix, dt, simulation_time, setpoint, kp, ki, kd, false);
    test_controllers(nonlinear_sys, noise_gen, output_prefix, dt, simulation_time, setpoint, kp, ki, kd, false);
    
    // Test 2: Tests with noise
    std::cout << "\n--- Test 2: Tests with noise ---" << std::endl;
    test_controllers(first_order_sys, noise_gen, output_prefix, dt, simulation_time, setpoint, kp, ki, kd, true);
    test_controllers(second_order_sys, noise_gen, output_prefix, dt, simulation_time, setpoint, kp, ki, kd, true);
    test_controllers(nonlinear_sys, noise_gen, output_prefix, dt, simulation_time, setpoint, kp, ki, kd, true);
    
    // Test 3: Long-term stability test with first-order system
    std::cout << "\n--- Test 3: Long-term stability test (100 seconds) ---" << std::endl;
    test_controllers(first_order_sys, noise_gen, output_prefix, dt, long_simulation_time, setpoint, kp, ki, kd, true);
    
    std::cout << "\n=== All tests completed successfully! ===" << std::endl;
    std::cout << "\nGenerated CSV files can be analyzed using MATLAB scripts for detailed performance evaluation." << std::endl;
    std::cout << "Use command line argument to specify different prefixes and avoid file overwriting." << std::endl;
    std::cout << "Example: test_all_controllers.exe run1" << std::endl;
    
    return 0;
}