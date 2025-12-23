// C++/Eigen PID控制器头文件
// 实现了多种PID控制器类型
// 版本：1.0
// 日期：2025-12-23

#ifndef PID_CONTROLLER_EIGEN_H
#define PID_CONTROLLER_EIGEN_H

#include <Eigen/Dense>

namespace cpp_control_lib {

// PID控制器类型枚举
enum class PIDType {
    STANDARD,    // 标准PID
    IMC,         // 内模PID
    PARALLEL,    // 并行PID
    TUNING       // 自整定PID
};

// PID控制器类
class PIDControllerEigen {
private:
    // PID参数
    double Kp_;          // 比例增益
    double Ki_;          // 积分增益
    double Kd_;          // 微分增益
    double Ts_;          // 采样时间
    PIDType type_;       // PID类型
    
    // 内部状态
    double error_prev_;  // 上一次误差
    double integral_;    // 积分项
    double derivative_;  // 微分项
    
    // 输出限制
    double output_min_;  // 输出最小值
    double output_max_;  // 输出最大值
    
    // 抗积分饱和
    bool anti_windup_;   // 是否启用抗积分饱和
    
    // IMC参数
    double lambda_;      // 滤波时间常数
    
    // 自整定参数
    double Ku_;          // 临界增益
    double Tu_;          // 临界周期
    
public:
    // 构造函数
    PIDControllerEigen(double Kp = 0.0, double Ki = 0.0, double Kd = 0.0, double Ts = 0.1, PIDType type = PIDType::STANDARD);
    
    // 析构函数
    ~PIDControllerEigen() = default;
    
    // 设置PID参数
    void setPIDParameters(double Kp, double Ki, double Kd);
    
    // 设置采样时间
    void setSampleTime(double Ts);
    
    // 设置PID类型
    void setPIDType(PIDType type);
    
    // 设置输出限制
    void setOutputLimits(double output_min, double output_max);
    
    // 启用/禁用抗积分饱和
    void setAntiWindup(bool anti_windup);
    
    // 设置IMC参数
    void setIMCParameters(double lambda);
    
    // 设置自整定参数
    void setTuningParameters(double Ku, double Tu);
    
    // 计算控制输出
    double compute(double setpoint, double process_value);
    
    // 重置控制器状态
    void reset();
    
    // 获取PID参数
    Eigen::Vector3d getPIDParameters() const;
    
    // 获取控制器类型
    PIDType getPIDType() const;
};

} // namespace cpp_control_lib

#endif // PID_CONTROLLER_EIGEN_H