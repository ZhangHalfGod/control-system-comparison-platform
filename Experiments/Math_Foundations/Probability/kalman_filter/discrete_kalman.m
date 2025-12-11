% 离散卡尔曼滤波（Discrete Kalman Filter）
% 输入：
%   x0 - 初始状态估计向量
%   P0 - 初始状态协方差矩阵
%   A - 状态转移矩阵
%   B - 控制输入矩阵
%   H - 观测矩阵
%   Q - 过程噪声协方差矩阵
%   R - 观测噪声协方差矩阵
%   u - 控制输入序列（可选）
%   z - 观测序列
% 输出：
%   x_est - 状态估计序列
%   P_est - 状态协方差估计序列

function [x_est, P_est] = discrete_kalman(x0, P0, A, B, H, Q, R, z, u)
    % 初始化
    n_steps = size(z, 1);
    n_states = size(A, 1);
    
    % 检查控制输入是否提供
    if nargin < 9 || isempty(u)
        u = zeros(n_steps, size(B, 2));
    end
    
    % 初始化估计序列
    x_est = zeros(n_steps, n_states);
    P_est = zeros(n_states, n_states, n_steps);
    
    % 设置初始状态
    x_est_prev = x0;
    P_est_prev = P0;
    
    disp('离散卡尔曼滤波开始执行：');
    disp(['初始状态估计：', num2str(x0')]);
    disp(['初始状态协方差：']);
    disp(P0);
    
    % 卡尔曼滤波迭代
    for k = 1:n_steps
        % 预测步骤
        x_pred = A * x_est_prev + B * u(k, :)';
        P_pred = A * P_est_prev * A' + Q;
        
        % 更新步骤
        z_k = z(k, :)';
        y = z_k - H * x_pred;  % 观测残差
        S = H * P_pred * H' + R;  % 残差协方差
        K = P_pred * H' / S;  % 卡尔曼增益
        
        x_est_prev = x_pred + K * y;
        P_est_prev = (eye(n_states) - K * H) * P_pred;
        
        % 保存估计结果
        x_est(k, :) = x_est_prev';
        P_est(:, :, k) = P_est_prev;
        
        % 显示中间结果
        if mod(k, 10) == 0 || k == n_steps
            disp(['时间步 k=', num2str(k), '：']);
            disp(['状态估计：', num2str(x_est_prev')]);
            disp(['观测值：', num2str(z_k')]);
        end
    end
    
    disp('离散卡尔曼滤波执行完成。');
end