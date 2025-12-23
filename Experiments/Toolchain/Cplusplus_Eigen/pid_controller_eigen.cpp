// C++/Eigen PID控制器实现文件
// 实现了多种PID控制器类型
// 版本：1.0
// 日期：2025-12-23

#include "pid_controller_eigen.h"

namespace cpp_control_lib {

// 构造函数
PIDControllerEigen::PIDControllerEigen(double Kp, double Ki, double Kd, double Ts, PIDType type) :
    Kp_(Kp), Ki_(Ki), Kd_(Kd), Ts_(Ts), type_(type),
    error_prev_(0.0), integral_(0.0), derivative_(0.0),
    output_min_(-100.0), output_max_(100.0), anti_windup_(false),
    lambda_(1.0), Ku_(0.0), Tu_(0.0)
{}

// 设置PID参数
void PIDControllerEigen::setPIDParameters(double Kp, double Ki, double Kd) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}

// 设置采样时间
void PIDControllerEigen::setSampleTime(double Ts) {
    Ts_ = Ts;
}

// 设置PID类型
void PIDControllerEigen::setPIDType(PIDType type) {
    type_ = type;
}

// 设置输出限制
void PIDControllerEigen::setOutputLimits(double output_min, double output_max) {
    output_min_ = output_min;
    output_max_ = output_max;
}

// 启用/禁用抗积分饱和
void PIDControllerEigen::setAntiWindup(bool anti_windup) {
    anti_windup_ = anti_windup;
}

// 设置IMC参数
void PIDControllerEigen::setIMCParameters(double lambda) {
    lambda_ = lambda;
}

// 设置自整定参数
void PIDControllerEigen::setTuningParameters(double Ku, double Tu) {
    Ku_ = Ku;
    Tu_ = Tu;
}

// 计算控制输出
double PIDControllerEigen::compute(double setpoint, double process_value) {
    // 计算误差
    double error = setpoint - process_value;
    
    // 根据PID类型计算输出
    double output = 0.0;
    
    switch (type_) {
        case PIDType::STANDARD: {
            // 标准PID算法
            integral_ += error * Ts_;
            derivative_ = (error - error_prev_) / Ts_;
            output = Kp_ * (error + Ki_ * integral_ + Kd_ * derivative_);
            break;
        }
        
        case PIDType::IMC: {
            // 内模PID算法
            double tau = 0.1; // 过程时间常数
            Kp_ = (tau + lambda_ / 2.0) / (Kp_ * (lambda_ + Ts_ / 2.0));
            Ki_ = 1.0 / (lambda_ + Ts_ / 2.0);
            Kd_ = (tau) / (Kp_ * (lambda_ + Ts_ / 2.0));
            
            integral_ += error * Ts_;
            derivative_ = (error - error_prev_) / Ts_;
            output = Kp_ * (error + Ki_ * integral_ + Kd_ * derivative_);
            break;
        }
        
        case PIDType::PARALLEL: {
            // 并行PID算法
            integral_ += error * Ts_;
            derivative_ = (error - error_prev_) / Ts_;
            output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative_;
            break;
        }
        
        case PIDType::TUNING: {
            // 自整定PID算法 (Ziegler-Nichols方法)
            Kp_ = 0.6 * Ku_;
            Ki_ = 1.2 * Ku_ / Tu_;
            Kd_ = 0.075 * Ku_ * Tu_;
            
            integral_ += error * Ts_;
            derivative_ = (error - error_prev_) / Ts_;
            output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative_;
            break;
        }
    }
    
    // 应用输出限制
    double output_unclamped = output;
    output = std::max(output_min_, std::min(output_max_, output));
    
    // 抗积分饱和处理
    if (anti_windup_) {
        if (output != output_unclamped) {
            integral_ -= error * Ts_; // 积分回溯
        }
    }
    
    // 更新状态
    error_prev_ = error;
    
    return output;
}

// 重置控制器状态
void PIDControllerEigen::reset() {
    error_prev_ = 0.0;
    integral_ = 0.0;
    derivative_ = 0.0;
}

// 获取PID参数
Eigen::Vector3d PIDControllerEigen::getPIDParameters() const {
    return Eigen::Vector3d(Kp_, Ki_, Kd_);
}

// 获取控制器类型
PIDType PIDControllerEigen::getPIDType() const {
    return type_;
}

} // namespace cpp_control_lib