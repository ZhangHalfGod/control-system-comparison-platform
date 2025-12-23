// C++/Eigen PID控制器测试脚本
// 测试多种PID控制器类型的性能
// 版本：1.0
// 日期：2025-12-23

#include "pid_controller_eigen.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <fstream>

// 简单的一阶系统模型
class FirstOrderSystem {
private:
    double K_;      // 系统增益
    double tau_;    // 时间常数
    double Ts_;     // 采样时间
    double y_prev_; // 上一次输出
    
public:
    FirstOrderSystem(double K = 1.0, double tau = 1.0, double Ts = 0.1) :
        K_(K), tau_(tau), Ts_(Ts), y_prev_(0.0) {}
    
    double update(double u) {
        // 一阶系统离散化模型：y(k) = (1 - Ts/tau) * y(k-1) + (K*Ts/tau) * u(k)
        double y = (1.0 - Ts_/tau_) * y_prev_ + (K_*Ts_/tau_) * u;
        y_prev_ = y;
        return y;
    }
    
    void reset() {
        y_prev_ = 0.0;
    }
};

// 主函数
int main() {
    // 系统参数
    double K = 1.0;
    double tau = 1.0;
    double Ts = 0.1;
    double simulation_time = 10.0;
    int num_steps = static_cast<int>(simulation_time / Ts);
    
    // 测试不同类型的PID控制器
    std::vector<cpp_control_lib::PIDType> pid_types = {
        cpp_control_lib::PIDType::STANDARD,
        cpp_control_lib::PIDType::PARALLEL,
        cpp_control_lib::PIDType::IMC,
        cpp_control_lib::PIDType::TUNING
    };
    
    std::vector<std::string> pid_type_names = {
        "STANDARD",
        "PARALLEL",
        "IMC",
        "TUNING"
    };
    
    // 参考输入（阶跃信号）
    double setpoint = 1.0;
    
    // 打开输出文件
    std::ofstream outfile("pid_eigen_test_results.txt");
    outfile << "Time,Type,Setpoint,ProcessValue,Output\n";
    
    // 测试每种PID控制器类型
    for (int i = 0; i < pid_types.size(); ++i) {
        cpp_control_lib::PIDControllerEigen pid(2.0, 1.0, 0.5, Ts, pid_types[i]);
        FirstOrderSystem system(K, tau, Ts);
        
        // 设置PID参数和输出限制
        pid.setOutputLimits(-5.0, 5.0);
        pid.setAntiWindup(true);
        
        // 设置IMC参数（如果是IMC类型）
        if (pid_types[i] == cpp_control_lib::PIDType::IMC) {
            pid.setIMCParameters(0.5);
        }
        
        // 设置自整定参数（如果是TUNING类型）
        if (pid_types[i] == cpp_control_lib::PIDType::TUNING) {
            pid.setTuningParameters(2.0, 1.57); // 假设临界增益和周期
        }
        
        // 重置系统和控制器
        system.reset();
        pid.reset();
        
        std::cout << "Testing " << pid_type_names[i] << " PID controller..." << std::endl;
        
        // 运行仿真
        for (int k = 0; k < num_steps; ++k) {
            double time = k * Ts;
            
            // 获取当前过程值
            double process_value = system.update(0.0);
            
            // 计算控制输出
            double output = pid.compute(setpoint, process_value);
            
            // 更新系统
            process_value = system.update(output);
            
            // 记录结果
            outfile << time << "," << pid_type_names[i] << "," << setpoint << "," << process_value << "," << output << "\n";
        }
    }
    
    outfile.close();
    std::cout << "PID controller test completed. Results saved to pid_eigen_test_results.txt" << std::endl;
    
    return 0;
}