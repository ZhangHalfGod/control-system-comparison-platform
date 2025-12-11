% TEST_STOCHASTIC_SYSTEMS 测试随机系统分析工具
%   测试伊藤微分方程求解器和随机系统分析功能

% 清除工作区和命令窗口
clear all;
close all;
clc;

fprintf('=== 测试随机系统分析工具 ===\n');

%% 测试1：几何布朗运动（GBM）
fprintf('\n=== 测试几何布朗运动 ===\n');

% 几何布朗运动的伊藤方程：dS(t) = μS(t)dt + σS(t)dW(t)
% 其中μ是漂移率，σ是波动率
mu = 0.1; % 漂移率
sigma = 0.2; % 波动率

% 定义漂移项和扩散项
f_gbm = @(t, S) mu*S;
g_gbm = @(t, S) sigma*S;

% 初始条件和参数
t0 = 0;
S0 = 100; % 初始价格
t_end = 1; % 1年
h = 0.001; % 小步长
num_paths = 50; % 50条模拟路径

fprintf('模拟%d条几何布朗运动路径...\n', num_paths);
[t, S] = ito_solver(f_gbm, g_gbm, t0, S0, t_end, h, num_paths);

% 绘制结果
figure('Name', '几何布朗运动模拟', 'Position', [100, 100, 800, 600]);

% 绘制所有路径
for i = 1:num_paths
    plot(t, squeeze(S(i, :, :)), 'Color', [0.7, 0.7, 0.7], 'LineWidth', 0.5);
    hold on;
end

% 绘制均值路径
mean_S = mean(S, 1);
plot(t, squeeze(mean_S), 'b-', 'LineWidth', 2, 'DisplayName', '均值路径');

% 绘制理论期望：E[S(t)] = S0*exp(μ*t)
expected_S = S0*exp(mu*t);
plot(t, expected_S, 'r--', 'LineWidth', 2, 'DisplayName', '理论期望');

hold off;
legend('Location', 'best');
title(sprintf('几何布朗运动模拟 (μ=%.2f, σ=%.2f)', mu, sigma));
xlabel('时间 t');
ylabel('价格 S(t)');
grid on;

% 计算统计矩
fprintf('\n=== 统计矩分析 ===\n');
[mean_y, var_y] = stochastic_moments(f_gbm, g_gbm, t0, S0, t_end, h, num_paths);

final_mean = mean_y(end, 1);
final_var = var_y(end, 1);
theoretical_mean = S0*exp(mu*t_end);
theoretical_var = S0^2*exp(2*mu*t_end)*(exp(sigma^2*t_end) - 1);

fprintf('最终时间步统计结果：\n');
fprintf('实际均值: %.4f\n', final_mean);
fprintf('理论均值: %.4f\n', theoretical_mean);
fprintf('均值相对误差: %.2f%%\n', abs(final_mean - theoretical_mean)/theoretical_mean * 100);

fprintf('\n实际方差: %.4f\n', final_var);
fprintf('理论方差: %.4f\n', theoretical_var);
fprintf('方差相对误差: %.2f%%\n', abs(final_var - theoretical_var)/theoretical_var * 100);

%% 测试2：Ornstein-Uhlenbeck过程
fprintf('\n=== 测试Ornstein-Uhlenbeck过程 ===\n');

% Ornstein-Uhlenbeck过程的伊藤方程：dX(t) = θ(μ - X(t))dt + σdW(t)
% 这是一个均值回复过程，常用于利率模型

% 参数设置
theta = 5; % 均值回复速度
mu_ou = 0; % 长期均值
sigma_ou = 1; % 波动率

% 定义漂移项和扩散项
f_ou = @(t, X) theta*(mu_ou - X);
g_ou = @(t, X) sigma_ou*eye(length(X)); % 扩散项矩阵

% 初始条件和参数
t0 = 0;
X0 = 2; % 初始偏离长期均值
t_end = 2;
h = 0.001;
num_paths = 30;

fprintf('模拟%d条Ornstein-Uhlenbeck过程路径...\n', num_paths);
[t_ou, X] = ito_solver(f_ou, g_ou, t0, X0, t_end, h, num_paths);

% 绘制结果
figure('Name', 'Ornstein-Uhlenbeck过程模拟', 'Position', [100, 100, 800, 600]);

% 绘制所有路径
for i = 1:num_paths
    plot(t_ou, squeeze(X(i, :, :)), 'Color', [0.7, 0.7, 0.7], 'LineWidth', 0.5);
    hold on;
end

% 绘制均值路径
mean_X = mean(X, 1);
plot(t_ou, squeeze(mean_X), 'b-', 'LineWidth', 2, 'DisplayName', '均值路径');

% 绘制理论长期均值
plot(t_ou, mu_ou*ones(size(t_ou)), 'r--', 'LineWidth', 2, 'DisplayName', '长期均值');

hold off;
legend('Location', 'best');
title(sprintf('Ornstein-Uhlenbeck过程 (θ=%.1f, μ=%.1f, σ=%.1f)', theta, mu_ou, sigma_ou));
xlabel('时间 t');
ylabel('X(t)');
grid on;

%% 测试3：李雅普诺夫指数计算
fprintf('\n=== 测试李雅普诺夫指数计算 ===\n');

% 使用简单的随机系统：dX = a*X*dt + b*X*dW
% 这个系统的最大李雅普诺夫指数应该接近a - b²/2

% 参数设置
a = 0.1;
b = 0.2;
f_lyap = @(t, X) a*X;
g_lyap = @(t, X) b*X;

% 初始条件和参数
t0 = 0;
X0 = 1;
t_end = 5;
h = 0.01;
num_paths = 10;

fprintf('计算随机系统的李雅普诺夫指数...\n');
lyapunov_exponents = compute_lyapunov_exponents(f_lyap, g_lyap, t0, X0, t_end, h, num_paths);

mean_lyapunov = mean(lyapunov_exponents);
theoretical_lyapunov = a - b^2/2;

fprintf('李雅普诺夫指数计算结果：\n');
fprintf('平均李雅普诺夫指数: %.4f\n', mean_lyapunov);
fprintf('理论李雅普诺夫指数: %.4f\n', theoretical_lyapunov);
fprintf('相对误差: %.2f%%\n', abs(mean_lyapunov - theoretical_lyapunov)/abs(theoretical_lyapunov) * 100);

%% 测试4：Milstein方法与Euler-Maruyama方法对比
fprintf('\n=== 测试Milstein方法 ===\n');

% 定义一个需要高阶方法的随机系统：dX = μX dt + σX dW
% Milstein方法应该比Euler-Maruyama方法更准确

% 参数设置
mu = 0.5;
sigma = 1.0;

% 漂移项和扩散项
f_milstein = @(t, X) mu*X;
g_milstein = @(t, X) sigma*X;

% Milstein方法需要的扩散项导数
dg_milstein = @(t, X, k) sigma; % dg/dX = sigma

% 初始条件和参数
t0 = 0;
X0 = 1;
t_end = 1;
h = 0.1; % 较大步长以突出差异
num_paths = 50;

% 使用Euler-Maruyama方法
fprintf('使用Euler-Maruyama方法模拟...\n');
[t_em, X_em] = ito_solver(f_milstein, g_milstein, t0, X0, t_end, h, num_paths);

% 使用Milstein方法
fprintf('使用Milstein方法模拟...\n');
[t_mil, X_mil] = milstein_solver(f_milstein, g_milstein, dg_milstein, t0, X0, t_end, h, num_paths);

% 理论解：X(t) = X0*exp((μ - σ²/2)t + σW(t))
% 这里我们用数值方法生成的Wiener过程来计算理论解
% 为简化，我们直接比较两种方法的方差

% 计算两种方法的方差
var_em = var(X_em, 1);
var_mil = var(X_mil, 1);

% 绘制结果
figure('Name', 'Euler-Maruyama vs Milstein方法', 'Position', [100, 100, 800, 600]);

% 绘制两种方法的均值路径
mean_em = mean(X_em, 1);
mean_mil = mean(X_mil, 1);

plot(t_em, squeeze(mean_em), 'b-', 'LineWidth', 2, 'DisplayName', 'Euler-Maruyama均值');
hold on;
plot(t_mil, squeeze(mean_mil), 'r-', 'LineWidth', 2, 'DisplayName', 'Milstein均值');

hold off;
legend('Location', 'best');
title('Euler-Maruyama vs Milstein方法对比');
xlabel('时间 t');
ylabel('X(t)');
grid on;

fprintf('\n=== 两种方法方差对比 ===\n');
fprintf('Euler-Maruyama方差: %.4f\n', var_em(end, 1));
fprintf('Milstein方差: %.4f\n', var_mil(end, 1));

fprintf('\n=== 测试完成 ===\n');
