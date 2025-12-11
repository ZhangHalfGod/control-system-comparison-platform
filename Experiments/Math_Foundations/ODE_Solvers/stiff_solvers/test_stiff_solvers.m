% TEST_STIFF_SOLVERS 测试刚性方程求解方法
%   测试Backward Euler方法处理刚性方程的性能和准确性

% 清除工作区和命令窗口
clear all;
close all;
clc;

fprintf('=== 测试刚性方程求解方法 ===\n');

%% 测试1：刚性一阶线性ODE
fprintf('\n=== 测试刚性一阶线性ODE ===\n');

% 定义刚性ODE：dy/dt = -1000y + 1000sin(t) + cos(t)
% 这个方程的刚性比约为1000
% 解析解：y(t) = sin(t) + C*exp(-1000t)
f1 = @(t, y) -1000*y + 1000*sin(t) + cos(t);
analytic_solution1 = @(t, y0) sin(t) + (y0 - 0)*exp(-1000*t);

% 初始条件和参数
t0 = 0;
y0 = 0;
t_end = 10;
h = 0.01; % 对于刚性方程，步长需要足够小

% 精确解
t_exact = t0:0.001:t_end;
y_exact = analytic_solution1(t_exact, y0);

% 使用Backward Euler方法（适合刚性方程）
fprintf('使用Backward Euler方法求解刚性方程...\n');
[t_be, y_be] = backward_euler(f1, t0, y0, t_end, h);

% 尝试使用RK4方法（可能不稳定）
fprintf('使用RK4方法求解刚性方程...\n');
[t_rk4, y_rk4] = rk4(f1, t0, y0, t_end, h);

% 绘制结果
figure('Name', '刚性方程测试', 'Position', [100, 100, 800, 600]);
subplot(2, 1, 1);
plot(t_exact, y_exact, 'k-', 'LineWidth', 2, 'DisplayName', '精确解');
hold on;
plot(t_be, y_be, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Backward Euler');
plot(t_rk4, y_rk4, 'b-.', 'LineWidth', 1.5, 'DisplayName', 'RK4');
hold off;
legend('Location', 'best');
title('刚性方程：dy/dt = -1000y + 1000sin(t) + cos(t)');
xlabel('时间 t');
ylabel('状态 y');
grid on;

subplot(2, 1, 2);
plot(t_be, abs(y_be - analytic_solution1(t_be, y0)'), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Backward Euler误差');
hold on;
plot(t_rk4, abs(y_rk4 - analytic_solution1(t_rk4, y0)'), 'b-.', 'LineWidth', 1.5, 'DisplayName', 'RK4误差');
hold off;
legend('Location', 'best');
title('误差对比');
xlabel('时间 t');
ylabel('绝对误差');
grid on;
yscale('log');

% 计算误差
fprintf('\n=== 刚性方程误差分析 ===\n');
fprintf('方法                | 最大误差\n');
fprintf('----------------------------\n');

max_error_be = max(abs(y_be - analytic_solution1(t_be, y0)'));
max_error_rk4 = max(abs(y_rk4 - analytic_solution1(t_rk4, y0)'));

fprintf('Backward Euler      | %.4e\n', max_error_be);
fprintf('RK4                 | %.4e\n', max_error_rk4);

%% 测试2：Van der Pol振荡器（强刚性情况）
fprintf('\n=== 测试强刚性Van der Pol振荡器 ===\n');

% 范德波尔振荡器：d²y/dt² - μ(1 - y²)dy/dt + y = 0
% 当μ很大时，这是一个刚性系统
mu = 1000; % 强刚性情况
f2 = @(t, y) [y(2); mu*(1 - y(1)^2)*y(2) - y(1)];

% 初始条件和参数
t0 = 0;
y0 = [2; 0];
t_end = 10;
h = 0.01;

% 使用Backward Euler方法
[t_be2, y_be2] = backward_euler(f2, t0, y0, t_end, h);

% 绘制结果
figure('Name', '强刚性范德波尔振荡器测试', 'Position', [100, 100, 800, 600]);
subplot(1, 2, 1);
plot(t_be2, y_be2(:, 1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Backward Euler');
legend('Location', 'best');
title(sprintf('强刚性范德波尔振荡器（μ=%d）：位移', mu));
xlabel('时间 t');
ylabel('位移 y');
grid on;

subplot(1, 2, 2);
plot(y_be2(:, 1), y_be2(:, 2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Backward Euler');
legend('Location', 'best');
title(sprintf('强刚性范德波尔振荡器（μ=%d）：相图', mu));
xlabel('位移 y');
ylabel('速度 dy/dt');
grid on;

fprintf('\n=== 测试完成 ===\n');
