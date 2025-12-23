% 阻抗控制器测试脚本
% 测试二阶质量-弹簧-阻尼系统的阻抗控制性能

clear all;
close all;
clc;

% 系统参数
m = 1.0;      % 质量 (kg)
b = 10.0;     % 阻尼系数 (N·s/m)
k = 100.0;    % 刚度系数 (N/m)
dt = 0.01;    % 采样时间 (s)
t_end = 5.0;  % 仿真时间 (s)
t = 0:dt:t_end;  % 时间向量
N = length(t);  % 仿真步数

% 期望轨迹：正弦波
amp = 0.5;    % 振幅 (m)
freq = 1.0;   % 频率 (Hz)
x_d = amp * sin(2*pi*freq*t);           % 期望位置
dx_d = amp * 2*pi*freq * cos(2*pi*freq*t);  % 期望速度
ddx_d = -amp * (2*pi*freq)^2 * sin(2*pi*freq*t);  % 期望加速度

% 初始化状态
x = 0;        % 初始位置
v = 0;        % 初始速度

% 存储仿真结果
x_history = zeros(1, N);
v_history = zeros(1, N);
tau_history = zeros(1, N);

% 仿真循环
for i = 1:N
    % 获取当前期望轨迹
    xd = x_d(i);
    dxd = dx_d(i);
    ddxd = ddx_d(i);
    
    % 调用阻抗控制器
    [tau, x, v] = impedance_controller(xd, dxd, ddxd, x, v, m, b, k, dt);
    
    % 存储结果
    x_history(i) = x;
    v_history(i) = v;
    tau_history(i) = tau;
end

% 绘制结果
figure('Name', '阻抗控制性能测试', 'Position', [100, 100, 1200, 800]);

% 位置跟踪
subplot(3, 1, 1);
plot(t, x_d, 'r-', 'LineWidth', 2, 'DisplayName', '期望位置');
hold on;
plot(t, x_history, 'b--', 'LineWidth', 2, 'DisplayName', '实际位置');
hold off;
grid on;
xlabel('时间 (s)');
ylabel('位置 (m)');
title('位置跟踪性能');
legend('Location', 'best');

% 速度跟踪
subplot(3, 1, 2);
plot(t, dx_d, 'r-', 'LineWidth', 2, 'DisplayName', '期望速度');
hold on;
plot(t, v_history, 'b--', 'LineWidth', 2, 'DisplayName', '实际速度');
hold off;
grid on;
xlabel('时间 (s)');
ylabel('速度 (m/s)');
title('速度跟踪性能');
legend('Location', 'best');

% 控制力矩
subplot(3, 1, 3);
plot(t, tau_history, 'g-', 'LineWidth', 2, 'DisplayName', '控制力矩');
grid on;
xlabel('时间 (s)');
ylabel('控制力矩 (N)');
title('控制力矩输出');
legend('Location', 'best');

% 计算性能指标
error = x_d - x_history;
rmse = sqrt(mean(error.^2));
mse = mean(error.^2);
mae = mean(abs(error));

fprintf('性能指标：\n');
fprintf('均方根误差 (RMSE): %.6f m\n', rmse);
fprintf('均方误差 (MSE): %.6f m²\n', mse);
fprintf('平均绝对误差 (MAE): %.6f m\n', mae);

% 保存结果
save('impedance_control_results.mat', 't', 'x_d', 'dx_d', 'ddx_d', 'x_history', 'v_history', 'tau_history', 'm', 'b', 'k', 'dt');

fprintf('\n仿真完成！结果已保存到 impedance_control_results.mat\n');
