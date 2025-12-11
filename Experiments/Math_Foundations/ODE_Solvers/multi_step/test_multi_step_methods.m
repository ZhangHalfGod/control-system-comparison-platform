% TEST_MULTI_STEP_METHODS 测试Adams-Bashforth和Adams-Moulton多步方法
%   测试多步ODE方法的性能和准确性

% 修复后的test_multi_step_methods.m文件开头部分
% 清除工作区和命令窗口
clear all;
close all;
clc;

% 添加runge_kutta目录到MATLAB路径，以便找到rk4函数
addpath('..\runge_kutta');


fprintf('=== 测试多步ODE方法 ===\n');

%% 测试1：一阶线性ODE（有解析解）
fprintf('\n=== 测试一阶线性ODE ===\n');

% 定义一阶线性ODE：dy/dt = -y + sin(t)
% 解析解：y(t) = (sin(t) - cos(t))/2 + C*exp(-t)
f1 = @(t, y) -y + sin(t);
analytic_solution1 = @(t, y0) (sin(t) - cos(t))/2 + (y0 + 0.5)*exp(-t);

% 初始条件和参数
t0 = 0;
y0 = 1;
t_end = 10;
h = 0.1;

% 精确解
t_exact = t0:0.01:t_end;
y_exact = analytic_solution1(t_exact, y0);

% 使用不同阶数的Adams-Bashforth方法
orders = [2, 3, 4];
colors = {'r', 'g', 'b'};
markers = {'o', 's', 'd'};

figure('Name', 'Adams-Bashforth方法测试（一阶线性ODE）', 'Position', [100, 100, 800, 600]);
plot(t_exact, y_exact, 'k-', 'LineWidth', 2, 'DisplayName', '精确解');

for i = 1:length(orders)
    order = orders(i);
    [t_ab, y_ab] = adams_bashforth(f1, t0, y0, t_end, h, order);
    plot(t_ab, y_ab, [colors{i}, markers{i}], 'LineWidth', 1.5, 'DisplayName', sprintf('Adams-Bashforth %d阶', order));
end

legend('Location', 'best');
title('Adams-Bashforth方法求解一阶线性ODE');
xlabel('时间 t');
ylabel('状态 y');
grid on;

% 计算误差
fprintf('\n=== 一阶线性ODE误差分析 ===\n');
fprintf('方法                | 最大误差\n');
fprintf('----------------------------\n');

for i = 1:length(orders)
    order = orders(i);
    [t_ab, y_ab] = adams_bashforth(f1, t0, y0, t_end, h, order);
    y_exact_ab = analytic_solution1(t_ab, y0);
    max_error = max(abs(y_ab - y_exact_ab'));
    fprintf('Adams-Bashforth %d阶 | %.4e\n', order, max_error);
end

%% 测试2：使用Adams-Moulton方法
title('Adams-Moulton方法求解一阶线性ODE');
figure('Name', 'Adams-Moulton方法测试（一阶线性ODE）', 'Position', [100, 100, 800, 600]);
plot(t_exact, y_exact, 'k-', 'LineWidth', 2, 'DisplayName', '精确解');

for i = 1:length(orders)
    order = orders(i);
    [t_am, y_am] = adams_moulton(f1, t0, y0, t_end, h, order);
    plot(t_am, y_am, [colors{i}, markers{i}], 'LineWidth', 1.5, 'DisplayName', sprintf('Adams-Moulton %d阶', order));
end

legend('Location', 'best');
title('Adams-Moulton方法求解一阶线性ODE');
xlabel('时间 t');
ylabel('状态 y');
grid on;

% 计算Adams-Moulton方法的误差
fprintf('\n=== Adams-Moulton方法误差分析 ===\n');
fprintf('方法                | 最大误差\n');
fprintf('----------------------------\n');

for i = 1:length(orders)
    order = orders(i);
    [t_am, y_am] = adams_moulton(f1, t0, y0, t_end, h, order);
    y_exact_am = analytic_solution1(t_am, y0);
    max_error = max(abs(y_am - y_exact_am'));
    fprintf('Adams-Moulton %d阶 | %.4e\n', order, max_error);
end

%% 测试3：二阶非线性ODE（范德波尔振荡器）
fprintf('\n=== 测试二阶非线性ODE（范德波尔振荡器） ===\n');

% 范德波尔振荡器：d²y/dt² - μ(1 - y²)dy/dt + y = 0
% 转换为一阶系统：y1' = y2, y2' = μ(1 - y1²)y2 - y1
mu = 1.0;
f2 = @(t, y) [y(2); mu*(1 - y(1)^2)*y(2) - y(1)];

% 初始条件和参数
t0 = 0;
y0 = [2; 0];
t_end = 20;
h = 0.05;

% 使用不同方法求解
[t_rk4, y_rk4] = rk4(f2, t0, y0, t_end, h);
[t_ab4, y_ab4] = adams_bashforth(f2, t0, y0, t_end, h, 4);
[t_am4, y_am4] = adams_moulton(f2, t0, y0, t_end, h, 4);

% 绘制结果
figure('Name', '范德波尔振荡器测试', 'Position', [100, 100, 800, 600]);
subplot(1, 2, 1);
plot(t_rk4, y_rk4(:, 1), 'k-', 'LineWidth', 2, 'DisplayName', 'RK4');
hold on;
plot(t_ab4, y_ab4(:, 1), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Adams-Bashforth 4阶');
plot(t_am4, y_am4(:, 1), 'b-.', 'LineWidth', 1.5, 'DisplayName', 'Adams-Moulton 4阶');
hold off;
legend('Location', 'best');
title('范德波尔振荡器：位移响应');
xlabel('时间 t');
ylabel('位移 y');
grid on;

subplot(1, 2, 2);
plot(y_rk4(:, 1), y_rk4(:, 2), 'k-', 'LineWidth', 2, 'DisplayName', 'RK4');
hold on;
plot(y_ab4(:, 1), y_ab4(:, 2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Adams-Bashforth 4阶');
plot(y_am4(:, 1), y_am4(:, 2), 'b-.', 'LineWidth', 1.5, 'DisplayName', 'Adams-Moulton 4阶');
hold off;
legend('Location', 'best');
title('范德波尔振荡器：相图');
xlabel('位移 y');
ylabel('速度 dy/dt');
grid on;

% 计算与RK4方法的误差
fprintf('\n=== 范德波尔振荡器误差分析（与RK4对比） ===\n');

% 由于时间步长相同，直接比较对应索引
    % 确保时间向量长度相同
    min_len = min(length(y_ab4), length(y_rk4));
    y_rk4_ab = y_rk4(1:min_len, :);
    y_ab4_trimmed = y_ab4(1:min_len, :);
    error_ab4 = max(max(abs(y_ab4_trimmed - y_rk4_ab)));

    min_len = min(length(y_am4), length(y_rk4));
    y_rk4_am = y_rk4(1:min_len, :);
    y_am4_trimmed = y_am4(1:min_len, :);
    error_am4 = max(max(abs(y_am4_trimmed - y_rk4_am)));

fprintf('Adams-Bashforth 4阶 | 最大误差: %.4e\n', error_ab4);
fprintf('Adams-Moulton 4阶   | 最大误差: %.4e\n', error_am4);

fprintf('\n=== 测试完成 ===\n');
