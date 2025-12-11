% MPC控制器核心文件
% 10步预测时域，5步控制时域

function u = mpc_controller(y, yd, A, B, C, D, Np, Nc, Q, R, u_min, u_max)
% mpc_controller 模型预测控制器实现
% 输入:
%   y - 当前输出
%   yd - 期望输出轨迹 (Np x 1)
%   A, B, C, D - 系统状态空间矩阵
%   Np - 预测时域 (10)
%   Nc - 控制时域 (5)
%   Q - 输出权重矩阵
%   R - 控制权重矩阵
%   u_min, u_max - 控制输入约束
% 输出:
%   u - 控制输入

% 系统维度
n = size(A, 1);  % 状态维度
m = size(B, 2);  % 输入维度
p = size(C, 1);  % 输出维度

% 初始状态（假设已知）
x = zeros(n, 1);
if length(y) == p
    x = [y; zeros(n-p, 1)];  % 简单扩展为完整状态
end

% 构建预测模型
Phi = zeros(Np*p, Nc*m);
Gamma = zeros(Np*p, n);

% 初始化预测
x_pred = x;
for i = 1:Np
    % 计算输出预测
    y_pred = C * x_pred + D * zeros(m, 1);
    
    % 构建Phi矩阵（控制输入到输出的映射）
    if i <= Nc
        % 对于控制时域内的输入，直接映射
        if i == 1
            Phi(1:p, 1:m) = C * B;
        else
            % 累积系统矩阵的作用
            temp = C * A^(i-1) * B;
            Phi((i-1)*p+1:i*p, 1:m) = temp;
        end
    end
    
    % 构建Gamma矩阵（初始状态到输出的映射）
    Gamma((i-1)*p+1:i*p, :) = C * A^i;
    
    % 更新状态预测
    x_pred = A * x_pred;
end

% 计算参考轨迹偏差
ref = yd - Gamma * x;

% 简化的MPC求解（无约束）
% 实际应用中应使用优化求解器处理约束
u = (Phi' * Q * Phi + R) \ (Phi' * Q * ref);

% 只取第一个控制输入
u = u(1:m);

% 应用控制约束
u = max(u, u_min);
u = min(u, u_max);
end
