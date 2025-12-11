% MPC控制器测试脚本
% 测试10步预测时域，5步控制时域的MPC控制器

clear all;
close all;
clc;

% 系统参数
dt = 0.1;    % 采样时间 (s)
t_end = 10.0;  % 仿真时间 (s)
t = 0:dt:t_end;
N = length(t);

% 期望轨迹：阶跃信号
yd = ones(1, N) * 1.0;

% 二阶系统状态空间模型（连续时间）
a = 1.0;
b = 1.0;
A_cont = [0 1; -a^2 -2*0.7*a];
B_cont = [0; b];
C_cont = [1 0];
D_cont = 0;

% 离散化（零阶保持）
A = expm(A_cont*dt);
B = (A - eye(2)) / A_cont * B_cont;
C = C_cont;
D = D_cont;

% MPC参数
Np = 10;  % 预测时域
Nc = 5;   % 控制时域
Q = eye(Np);  % 输出权重矩阵
R = 0.1 * eye(Nc);  % 控制权重矩阵
u_min = -2.0;
u_max = 2.0;

% 初始化状态
y = 0;
x = [0; 0];
u = 0;

% 存储仿真结果
y_history = zeros(1, N);
u_history = zeros(1, N);

% 仿真循环
for i = 1:N
    % 生成期望输出轨迹片段
    yd_segment = ones(Np, 1) * yd(i);
    
    % 调用MPC控制器
    u = mpc_controller(y, yd_segment, A, B, C, D, Np, Nc, Q, R, u_min, u_max);
    
    % 更新系统状态
    x = A * x + B * u;
    y = C * x + D * u;
    
    % 存储结果
    y_history(i) = y;
    u_history(i) = u;
end

% 绘制结果
figure('Name', 'MPC控制器性能测试', 'Position', [100, 100, 1200, 600]);

% 输出跟踪
subplot(2, 1, 1);
plot(t, yd, 'r-', 'LineWidth', 2, 'DisplayName', '期望输出');
hold on;
plot(t, y_history, 'b--', 'LineWidth', 2, 'DisplayName', '实际输出');
hold off;
grid on;
xlabel('时间 (s)');
ylabel('输出');
title('MPC控制输出跟踪');
legend('Location', 'best');

% 控制输入
subplot(2, 1, 2);
plot(t, u_history, 'g-', 'LineWidth', 2, 'DisplayName', '控制输入');
hold on;
plot(t, u_min*ones(1, N), 'r--', 'LineWidth', 1, 'DisplayName', '输入下限');
plot(t, u_max*ones(1, N), 'm--', 'LineWidth', 1, 'DisplayName', '输入上限');
hold off;
grid on;
xlabel('时间 (s)');
ylabel('控制输入');
title('MPC控制输入');
legend('Location', 'best');

% 计算性能指标
error = yd - y_history;
rmse = sqrt(mean(error.^2));
mse = mean(error.^2);
mae = mean(abs(error));

fprintf('性能指标：\n');
fprintf('均方根误差 (RMSE): %.6f\n', rmse);
fprintf('均方误差 (MSE): %.6f\n', mse);
fprintf('平均绝对误差 (MAE): %.6f\n', mae);

% 保存结果
save('mpc_results.mat', 't', 'yd', 'y_history', 'u_history', 'A', 'B', 'C', 'D', 'Np', 'Nc', 'dt');

fprintf('\n仿真完成！结果已保存到 mpc_results.mat\n');
