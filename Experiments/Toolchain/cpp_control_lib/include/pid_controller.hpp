// 极简版本，避免任何可能的循环包含
#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

class PIDController {
public:
    // 构造函数
    PIDController(double kp_val, double ki_val, double kd_val, double dt_val);
    
    // 计算PID输出
    double compute(double setpoint, double process_val);
    
    // 重置控制器
    void reset();

private:
    double kp;
    double ki;
    double kd;
    double dt;
    double prev_err;
    double integral;
    double prev_pv;
};

#endif // _PID_CONTROLLER_H_