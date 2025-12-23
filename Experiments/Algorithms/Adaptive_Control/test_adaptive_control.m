% 自适应控制器测试脚本
% 测试基于MIT规则的自适应控制性能

clear all;
close all;
clc;

% 系统参数
dt = 0.01;    % 采样时间 (s)
t_end = 10.0;  % 仿真时间 (s)
t = 0:dt:t_end;  % 时间向量
N = length(t);  % 仿真步数

% 期望轨迹：阶跃信号
yd = ones(1, N) * 1.0;
dyd = zeros(1, N);

% 实际系统参数（未知）
a_real = 2.0;   % 实际阻尼系数
b_real = 5.0;   % 实际增益

% 初始化状态
y = 0;          % 初始输出
u = 0;          % 初始控制输入

% 自适应参数初始化
theta_hat = [1.0; 1.0];  % 初始参数估计值 [a_hat; b_hat]
gamma = 0.05;    % 自适应增益
P = eye(2) * 100;  % RLS协方差矩阵

% 存储仿真结果
y_history = zeros(1, N);
u_history = zeros(1, N);
theta_hat_history = zeros(2, N);
error_history = zeros(1, N);

% 仿真循环
for i = 1:N
    % 获取当前期望输出和导数
    yd_i = yd(i);
    dyd_i = dyd(i);
    
    % 调用自适应控制器
    [u, theta_hat] = adaptive_controller(y, yd_i, dyd_i, theta_hat, P, gamma, dt);
    
    % 实际系统动力学：y_dot = -a_real*y + b_real*u
    y_dot = -a_real * y + b_real * u;
    y = y + y_dot * dt;
    
    % 计算误差
    error = yd_i - y;
    
    % 存储结果
    y_history(i) = y;
    u_history(i) = u;
    theta_hat_history(:, i) = theta_hat;
    error_history(i) = error;
end

% 绘制结果
figure('Name', '自适应控制性能测试', 'Position', [100, 100, 1200, 800]);

% 输出跟踪
subplot(3, 1, 1);
plot(t, yd, 'r-', 'LineWidth', 2, 'DisplayName', '期望输出');
hold on;
plot(t, y_history, 'b--', 'LineWidth', 2, 'DisplayName', '实际输出');
hold off;
grid on;
xlabel('时间 (s)');
ylabel('输出');
title('自适应控制输出跟踪');
legend('Location', 'best');

% 控制输入
subplot(3, 1, 2);
plot(t, u_history, 'g-', 'LineWidth', 2, 'DisplayName', '控制输入');
grid on;
xlabel('时间 (s)');
ylabel('控制输入');
title('自适应控制输入');
legend('Location', 'best');

% 参数估计
subplot(3, 1, 3);
plot(t, theta_hat_history(1, :), 'm-', 'LineWidth', 2, 'DisplayName', 'a_hat');
hold on;
plot(t, ones(1, N)*a_real, 'm:', 'LineWidth', 2, 'DisplayName', 'a_real');
plot(t, theta_hat_history(2, :), 'c--', 'LineWidth', 2, 'DisplayName', 'b_hat');
plot(t, ones(1, N)*b_real, 'c:', 'LineWidth', 2, 'DisplayName', 'b_real');
hold off;
grid on;
xlabel('时间 (s)');
ylabel('参数估计值');
title('自适应控制器参数估计');
legend('Location', 'best');

% 计算性能指标
rmse = sqrt(mean(error_history.^2));
mse = mean(error_history.^2);
mae = mean(abs(error_history));

fprintf('性能指标：\n');
fprintf('均方根误差 (RMSE): %.6f\n', rmse);
fprintf('均方误差 (MSE): %.6f\n', mse);
fprintf('平均绝对误差 (MAE): %.6f\n', mae);

% 保存结果
save('adaptive_control_results.mat', 't', 'yd', 'y_history', 'u_history', 'theta_hat_history', 'error_history', 'a_real', 'b_real', 'dt');

fprintf('\n仿真完成！结果已保存到 adaptive_control_results.mat\n');
