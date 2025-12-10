// 简化版所有控制器测试
#include <iostream>
#include "pid_controller.hpp"

int main() {
    std::cout << "Testing All Controllers..." << std::endl;
    
    // 仿真参数
    double dt = 0.1;
    double simulation_time = 5.0;
    int steps = static_cast<int>(simulation_time / dt);
    double setpoint = 1.0;
    
    // PID参数
    double kp = 0.5;
    double ki = 0.1;
    double kd = 0.05;
    
    // 创建控制器实例
    PIDController pid(kp, ki, kd, dt);
    IncrementalPIDController incremental_pid(kp, ki, kd, dt);
    FuzzyPIDController fuzzy_pid(kp, ki, kd, dt);
    AdaptivePIDController adaptive_pid(kp, ki, kd, dt, 0.01);
    
    // 简单的系统模拟
    double process_val = 0.0;
    
    std::cout << "\n=== Testing PID Controller ===" << std::endl;
    process_val = 0.0;
    for (int i = 0; i < steps; ++i) {
        double output = pid.compute(setpoint, process_val);
        process_val += output * 0.1; // 简单的系统响应
        std::cout << "Step " << i << ": Output = " << output << ", Process Val = " << process_val << std::endl;
    }
    
    std::cout << "\n=== Testing Incremental PID Controller ===" << std::endl;
    process_val = 0.0;
    for (int i = 0; i < steps; ++i) {
        double output = incremental_pid.compute(setpoint, process_val);
        process_val += output * 0.1;
        std::cout << "Step " << i << ": Output = " << output << ", Process Val = " << process_val << std::endl;
    }
    
    std::cout << "\n=== Testing Fuzzy PID Controller ===" << std::endl;
    process_val = 0.0;
    for (int i = 0; i < steps; ++i) {
        double output = fuzzy_pid.compute(setpoint, process_val);
        process_val += output * 0.1;
        std::cout << "Step " << i << ": Output = " << output << ", Process Val = " << process_val << std::endl;
    }
    
    std::cout << "\n=== Testing Adaptive PID Controller ===" << std::endl;
    process_val = 0.0;
    for (int i = 0; i < steps; ++i) {
        double output = adaptive_pid.compute(setpoint, process_val);
        process_val += output * 0.1;
        std::cout << "Step " << i << ": Output = " << output << ", Process Val = " << process_val << std::endl;
    }
    
    std::cout << "\nAll tests completed!" << std::endl;
    return 0;
}