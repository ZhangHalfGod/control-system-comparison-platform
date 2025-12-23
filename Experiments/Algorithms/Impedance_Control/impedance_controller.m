function [tau, x, dx] = impedance_controller(xd, dxd, ddxd, x, dx, m, b, k, dt)
% impedance_controller 二阶质量-弹簧-阻尼系统阻抗控制器
% 输入:
%   xd - 期望位置
%   dxd - 期望速度
%   ddxd - 期望加速度
%   x - 当前位置
%   dx - 当前速度
%   m - 质量参数
%   b - 阻尼系数
%   k - 刚度系数
%   dt - 采样时间
% 输出:
%   tau - 控制力矩
%   x - 更新后的位置
%   dx - 更新后的速度

% 计算位置误差和速度误差
x_err = xd - x;
dx_err = dxd - dx;

% 阻抗控制律：tau = m*(ddxd + k*x_err + b*dx_err) + k*x + b*dx
% 或者简化为：tau = m*ddxd + b*(dxd + dx_err) + k*(xd + x_err)
tau = m * ddxd + b * dx_err + k * x_err;

% 系统动力学：m*ddx = tau - b*dx - k*x
% 计算加速度
xdd = (tau - b * dx - k * x) / m;

% 使用欧拉积分更新位置和速度
dx = dx + xdd * dt;
x = x + dx * dt;
end
