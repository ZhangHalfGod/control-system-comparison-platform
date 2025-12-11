% 自适应控制器核心文件
% 基于MIT规则和RLS参数辨识的自适应控制

function [u, theta_hat] = adaptive_controller(y, yd, dyd, theta_hat, P, gamma, dt)
% adaptive_controller 自适应控制器实现
% 输入:
%   y - 当前输出
%   yd - 期望输出
%   dyd - 期望输出导数
%   theta_hat - 参数估计值 [a; b]
%   P - RLS协方差矩阵
%   gamma - 自适应增益
%   dt - 采样时间
% 输出:
%   u - 控制输入
%   theta_hat - 更新后的参数估计值

% 系统模型: y_dot + a*y = b*u

% 计算当前控制输入 u = (dyd + a*y) / b
u = (dyd + theta_hat(1) * y) / theta_hat(2);

% 计算回归向量
phi = [-y; u];

% 计算输出误差
e = yd - y;

% MIT规则参数更新
% theta_hat_dot = gamma * e * phi
theta_hat = theta_hat + gamma * e * phi * dt;

% 确保参数为正值
theta_hat = max(theta_hat, 0.01);
end
