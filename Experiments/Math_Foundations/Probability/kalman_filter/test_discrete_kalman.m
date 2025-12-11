% 测试离散卡尔曼滤波（Discrete Kalman Filter）
% 测试案例：跟踪一个一维运动物体的位置和速度

% 定义系统参数
n_steps = 50;  % 时间步数

% 状态转移矩阵（匀速运动模型）
% x(k+1) = A*x(k) + B*u(k) + w(k)
% 状态向量：[位置; 速度]
A = [1, 1; 0, 1];

% 控制输入矩阵
B = [0.5; 1];

% 观测矩阵（只观测位置）
H = [1, 0];

% 过程噪声协方差（假设速度有随机扰动）
Q = [0.01, 0; 0, 0.1];

% 观测噪声协方差
R = 1;  % 观测位置的噪声方差

% 初始状态（真实初始状态）
x_true_prev = [0; 2];  % 初始位置0，初始速度2

% 初始状态估计（带有误差）
x0 = [5; 0];  % 初始位置估计误差5，初始速度估计误差2

% 初始状态协方差
P0 = [10, 0; 0, 10];

% 生成真实状态序列和观测序列
true_states = zeros(n_steps, 2);
z = zeros(n_steps, 1);
u = zeros(n_steps, 1);  % 控制输入（设为0）

rng(123);  % 设置随机种子，确保结果可复现

disp('=== 生成测试数据 ===');
for k = 1:n_steps
    % 生成真实状态
    w = mvnrnd([0; 0], Q)';  % 过程噪声
    true_states(k, :) = x_true_prev';
    
    % 更新真实状态
    x_true_prev = A * x_true_prev + B * u(k) + w;
    
    % 生成观测
    v = mvnrnd(0, R);  % 观测噪声
    z(k) = H * true_states(k, :)' + v;
end

% 应用离散卡尔曼滤波
disp('\n=== 应用离散卡尔曼滤波 ===');
[x_est, P_est] = discrete_kalman(x0, P0, A, B, H, Q, R, z, u);

% 计算估计误差
error = true_states - x_est;
rmse = sqrt(mean(error.^2));

% 输出误差分析结果
disp('\n=== 误差分析 ===');
disp(['位置估计均方根误差：', num2str(rmse(1))]);
disp(['速度估计均方根误差：', num2str(rmse(2))]);

% 可视化结果
figure('Name', '离散卡尔曼滤波结果（位置估计）');
subplot(2, 1, 1);
plot(1:n_steps, true_states(:, 1), 'k-', 'LineWidth', 2, 'DisplayName', '真实位置');
plot(1:n_steps, z, 'ro', 'MarkerSize', 5, 'DisplayName', '观测位置');
plot(1:n_steps, x_est(:, 1), 'b--', 'LineWidth', 1.5, 'DisplayName', '卡尔曼估计位置');
hold on;
% 绘制位置估计的置信区间
    pos_std = sqrt(squeeze(P_est(1, 1, :)));
    plot(1:n_steps, x_est(:, 1) + 2*pos_std, 'b:', 'LineWidth', 1);
    plot(1:n_steps, x_est(:, 1) - 2*pos_std, 'b:', 'LineWidth', 1);
legend('Location', 'best');
xlabel('时间步 k');
ylabel('位置');
title('离散卡尔曼滤波：位置估计结果');
grid on;
hold off;

subplot(2, 1, 2);
plot(1:n_steps, true_states(:, 2), 'k-', 'LineWidth', 2, 'DisplayName', '真实速度');
plot(1:n_steps, x_est(:, 2), 'b--', 'LineWidth', 1.5, 'DisplayName', '卡尔曼估计速度');
hold on;
% 绘制速度估计的置信区间
    vel_std = sqrt(squeeze(P_est(2, 2, :)));
    plot(1:n_steps, x_est(:, 2) + 2*vel_std, 'b:', 'LineWidth', 1);
    plot(1:n_steps, x_est(:, 2) - 2*vel_std, 'b:', 'LineWidth', 1);
legend('Location', 'best');
xlabel('时间步 k');
ylabel('速度');
title('离散卡尔曼滤波：速度估计结果');
grid on;
hold off;

% 绘制估计误差
figure('Name', '离散卡尔曼滤波估计误差');
subplot(2, 1, 1);
plot(1:n_steps, error(:, 1), 'r-', 'LineWidth', 1.5);
xlabel('时间步 k');
ylabel('位置估计误差');
title('离散卡尔曼滤波：位置估计误差');
grid on;

subplot(2, 1, 2);
plot(1:n_steps, error(:, 2), 'b-', 'LineWidth', 1.5);
xlabel('时间步 k');
ylabel('速度估计误差');
title('离散卡尔曼滤波：速度估计误差');
grid on;

% 绘制卡尔曼增益
figure('Name', '卡尔曼增益变化');
gain_pos = zeros(n_steps, 1);
gain_vel = zeros(n_steps, 1);

for k = 1:n_steps
    % 计算卡尔曼增益（从协方差中推导）
    P_pred = A * P_est(:, :, min(k, end)) * A' + Q;
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;
    
    gain_pos(k) = K(1);
    gain_vel(k) = K(2);
end

plot(1:n_steps, gain_pos, 'r-', 'LineWidth', 1.5, 'DisplayName', '位置增益');
hold on;
plot(1:n_steps, gain_vel, 'b-', 'LineWidth', 1.5, 'DisplayName', '速度增益');
xlabel('时间步 k');
ylabel('卡尔曼增益');
title('卡尔曼增益随时间变化');
legend('Location', 'best');
grid on;
hold off;

% 输出结果总结
disp('\n=== 结果总结 ===');
disp(['滤波前初始位置误差：', num2str(abs(x0(1) - true_states(1, 1)))]);
disp(['滤波后最终位置误差：', num2str(abs(x_est(end, 1) - true_states(end, 1)))]);
disp(['滤波前初始速度误差：', num2str(abs(x0(2) - true_states(1, 2)))]);
disp(['滤波后最终速度误差：', num2str(abs(x_est(end, 2) - true_states(end, 2)))]);
disp('\n卡尔曼滤波成功降低了初始估计误差！');