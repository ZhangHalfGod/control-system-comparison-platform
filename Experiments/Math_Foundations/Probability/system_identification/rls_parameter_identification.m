% 递归最小二乘（RLS）参数辨识
% 输入：
%   phi - 回归矩阵（每一行是一个观测时刻的回归向量）
%   y - 输出序列（列向量）
%   lambda - 遗忘因子（0 < lambda <= 1）
%   theta0 - 初始参数估计（可选）
%   P0 - 初始协方差矩阵（可选）
% 输出：
%   theta_est - 参数估计序列（每一行是一个观测时刻的参数估计）
%   P - 协方差矩阵序列
%   e - 估计误差序列

function [theta_est, P, e] = rls_parameter_identification(phi, y, lambda, theta0, P0)
    % 初始化
    n_steps = size(phi, 1);
    n_params = size(phi, 2);
    
    % 检查初始参数和协方差矩阵是否提供
    if nargin < 4 || isempty(theta0)
        theta0 = zeros(n_params, 1);
    end
    
    if nargin < 5 || isempty(P0)
        P0 = eye(n_params) * 1000;  % 初始协方差矩阵设为较大值
    end
    
    % 初始化估计序列
    theta_est = zeros(n_steps, n_params);
    P = zeros(n_params, n_params, n_steps);
    e = zeros(n_steps, 1);
    
    % 设置初始状态
    theta_prev = theta0;
    P_prev = P0;
    
    disp(['RLS参数辨识开始执行：']);
    disp(['遗忘因子 lambda = ', num2str(lambda)]);
    disp(['初始参数估计：', num2str(theta0')]);
    disp(['初始协方差矩阵：']);
    disp(P0);
    
    % RLS迭代
    for k = 1:n_steps
        phi_k = phi(k, :)';
        y_k = y(k);
        
        % 计算估计误差
        e(k) = y_k - phi_k' * theta_prev;
        
        % 计算增益矩阵
        K = P_prev * phi_k / (lambda + phi_k' * P_prev * phi_k);
        
        % 更新参数估计
        theta_prev = theta_prev + K * e(k);
        
        % 更新协方差矩阵
        P_prev = (P_prev - K * phi_k' * P_prev) / lambda;
        
        % 保存估计结果
        theta_est(k, :) = theta_prev';
        P(:, :, k) = P_prev;
        
        % 显示中间结果
        if mod(k, 10) == 0 || k == n_steps
            disp(['时间步 k=', num2str(k), '：']);
            disp(['参数估计：', num2str(theta_prev')]);
            disp(['估计误差：', num2str(e(k))]);
        end
    end
    
    disp('RLS参数辨识执行完成。');
    disp(['最终参数估计：', num2str(theta_prev')]);
end