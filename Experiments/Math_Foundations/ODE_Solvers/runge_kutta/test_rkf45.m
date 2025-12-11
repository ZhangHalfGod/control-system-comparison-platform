% TEST_RKF45 测试RKF45自适应步长方法
%   测试Runge-Kutta-Fehlberg方法的性能和准确性

% 清除工作区和命令窗口
clear all;
close all;
clc;

fprintf('=== 测试RKF45自适应步长方法 ===\n');

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

% 测试不同误差容限
noise_tols = [1e-3, 1e-5, 1e-7];
colors = {'r', 'g', 'b'};

% 精确解
t_exact = t0:0.001:t_end;
y_exact = analytic_solution1(t_exact, y0);

figure('Name', 'RKF45方法测试', 'Position', [100, 100, 800, 600]);
subplot(2, 1, 1);
plot(t_exact, y_exact, 'k-', 'LineWidth', 2, 'DisplayName', '精确解');
hold on;

% 存储不同容限下的结果
results = struct();

for i = 1:length(noise_tols)
    tol = noise_tols(i);
    fprintf('使用RKF45方法，容限tol=%.1e...\n', tol);
    
    % 使用RKF45方法（自适应步长）
    [t_rkf45, y_rkf45, h_used] = rkf45(f1, t0, y0, t_end, 0.1, tol);
    
    % 计算误差
    y_exact_rkf45 = analytic_solution1(t_rkf45, y0);
    max_error = max(abs(y_rkf45 - y_exact_rkf45'));
    
    % 存储结果
    results(i).tol = tol;
    results(i).t = t_rkf45;
    results(i).y = y_rkf45;
    results(i).h_used = h_used;
    results(i).max_error = max_error;
    results(i).num_steps = length(t_rkf45) - 1;
    
    % 绘制结果
    plot(t_rkf45, y_rkf45, [colors{i}, 'o'], 'LineWidth', 1.5, 'MarkerSize', 3, 'DisplayName', sprintf('RKF45 (tol=%.1e)', tol));
end

hold off;
legend('Location', 'best');
title('RKF45自适应步长方法：一阶线性ODE');
xlabel('时间 t');
ylabel('状态 y');
grid on;

% 绘制步长变化
subplot(2, 1, 2);
for i = 1:length(noise_tols)
    t_rkf45 = results(i).t;
    h_used = results(i).h_used;
    plot(t_rkf45(1:end-1), h_used, colors{i}, 'LineWidth', 1.5, 'DisplayName', sprintf('tol=%.1e', results(i).tol));
    hold on;
end
hold off;
legend('Location', 'best');
title('RKF45方法：步长变化');
xlabel('时间 t');
ylabel('步长 h');
grid on;
yscale('log');

% 打印性能对比
fprintf('\n=== RKF45性能分析 ===\n');
fprintf('容限tol    | 最大误差 | 时间步数 | 平均步长\n');
fprintf('-------------------------------\n');

for i = 1:length(results)
    tol = results(i).tol;
    max_error = results(i).max_error;
    num_steps = results(i).num_steps;
    avg_h = mean(results(i).h_used);
    fprintf('%.1e    | %.4e | %5d | %.4f\n', tol, max_error, num_steps, avg_h);
end

%% 测试2：与固定步长RK4对比
fprintf('\n=== 与固定步长RK4对比 ===\n');

% 定义二阶非线性ODE：范德波尔振荡器
mu = 1.0;
f2 = @(t, y) [y(2); mu*(1 - y(1)^2)*y(2) - y(1)];

% 初始条件和参数
t0 = 0;
y0 = [2; 0];
t_end = 20;

% 使用RKF45方法（自适应步长）
fprintf('使用RKF45方法求解范德波尔振荡器...\n');
[t_rkf45, y_rkf45, h_used] = rkf45(f2, t0, y0, t_end, 0.1, 1e-5);

% 使用固定步长RK4方法
fprintf('使用固定步长RK4方法求解范德波尔振荡器...\n');
h_fixed = 0.01;
[t_rk4, y_rk4] = rk4(f2, t0, y0, t_end, h_fixed);

% 绘制结果
figure('Name', 'RKF45 vs 固定步长RK4', 'Position', [100, 100, 800, 600]);
subplot(1, 2, 1);
plot(t_rk4, y_rk4(:, 1), 'k-', 'LineWidth', 2, 'DisplayName', sprintf('RK4 (h=%.2f)', h_fixed));
hold on;
plot(t_rkf45, y_rkf45(:, 1), 'r--', 'LineWidth', 1.5, 'DisplayName', 'RKF45 (自适应步长)');
hold off;
legend('Location', 'best');
title('范德波尔振荡器：位移响应');
xlabel('时间 t');
ylabel('位移 y');
grid on;

subplot(1, 2, 2);
plot(t_rkf45(1:end-1), h_used, 'b-.', 'LineWidth', 1.5, 'DisplayName', 'RKF45步长变化');
hold on;
plot(y_rk4(:, 1), y_rk4(:, 2), 'k-', 'LineWidth', 2, 'DisplayName', sprintf('RK4 (h=%.2f)', h_fixed));
hold off;
legend('Location', 'best');
title('RKF45自适应步长变化');
xlabel('时间 t');
ylabel('步长 h');
grid on;

% 性能对比
fprintf('\n=== 性能对比 ===\n');
fprintf('方法          | 容限tol | 时间步数 | 平均步长 | 总计算时间\n');
fprintf('-----------------------------------------------\n');

% 计算计算时间
tic;
t_rkf45, y_rkf45, h_used = rkf45(f2, t0, y0, t_end, 0.1, 1e-5);
time_rkf45 = toc;

num_steps_rkf45 = length(t_rkf45) - 1;
avg_h_rkf45 = mean(h_used);

fprintf('RKF45         | 1.0e-05 | %5d | %.4f | %.4f s\n', num_steps_rkf45, avg_h_rkf45, time_rkf45);

fprintf('\n=== 测试完成 ===\n');
