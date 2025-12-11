% 模糊PID控制器实现
% 功能：实现基于49条模糊规则和高斯隶属度函数的模糊PID控制器
% 输入：e - 当前误差
%       ec - 误差变化率
%       kp0 - 初始比例系数
%       ki0 - 初始积分系数
%       kd0 - 初始微分系数
%       e_range - 误差论域范围（默认[-3, 3]）
%       ec_range - 误差变化率论域范围（默认[-3, 3]）
% 输出：kp - 调整后的比例系数
%       ki - 调整后的积分系数
%       kd - 调整后的微分系数

function [kp, ki, kd] = fuzzy_pid_controller(e, ec, kp0, ki0, kd0, e_range, ec_range)
    % 设置默认参数
    if nargin < 6
        e_range = [-3, 3]; % 默认误差论域
    end
    
    if nargin < 7
        ec_range = [-3, 3]; % 默认误差变化率论域
    end
    
    % 输入变量归一化（将实际值映射到论域[-3, 3]）
    e_normalized = normalize_input(e, e_range, [-3, 3]);
    ec_normalized = normalize_input(ec, ec_range, [-3, 3]);
    
    % 调用模糊推理引擎获取PID参数调整量
    [delta_kp, delta_ki, delta_kd] = fuzzy_inference(e_normalized, ec_normalized);
    
    % PID参数调整（使用增益系数调整调整量的影响程度）
    % 可根据实际系统调整这些增益系数
    kp_gain = 0.7 * kp0; % Kp调整量增益（增大为0.5）
    ki_gain = 0.4 * ki0; % Ki调整量增益（增大为0.5）
    kd_gain = 0.6 * kd0; % Kd调整量增益（增大为0.5）
    
    % 计算调整后的PID参数
    kp = kp0 + kp_gain * delta_kp;
    ki = ki0 + ki_gain * delta_ki;
    kd = kd0 + kd_gain * delta_kd;
    
    % 确保PID参数为标量
    kp = mean(kp(:));
    ki = mean(ki(:));
    kd = mean(kd(:));
    
    % 确保PID参数为正值
    kp = max(kp, 0);
    ki = max(ki, 0);
    kd = max(kd, 0);
end

% 输入归一化函数
function normalized_val = normalize_input(val, input_range, target_range)
    % 将输入值从原始范围映射到目标范围
    input_min = input_range(1);
    input_max = input_range(2);
    target_min = target_range(1);
    target_max = target_range(2);
    
    % 线性映射
    normalized_val = target_min + (val - input_min) * (target_max - target_min) / (input_max - input_min);
    
    % 确保映射后的值在目标范围内
    normalized_val = max(normalized_val, target_min);
    normalized_val = min(normalized_val, target_max);
end