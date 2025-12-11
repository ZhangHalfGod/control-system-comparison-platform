function [gp_model, y_pred, y_std] = gaussian_process_regression(x_train, y_train, x_test, kernel_type, kernel_params, noise_var)
% GAUSSIAN_PROCESS_REGRESSION 实现高斯过程回归
%   [gp_model, y_pred, y_std] = gaussian_process_regression(x_train, y_train, x_test, kernel_type, kernel_params, noise_var)
%   使用高斯过程回归对数据进行建模和预测
%   
%   输入参数：
%       x_train - 训练数据输入，n_train×d矩阵，n_train为样本数，d为特征维度
%       y_train - 训练数据输出，n_train×1向量
%       x_test - 测试数据输入，n_test×d矩阵
%       kernel_type - 核函数类型（'rbf', 'matern', 'linear', 'periodic'）
%       kernel_params - 核函数参数，根据核函数类型不同而不同
%       noise_var - 观测噪声方差（默认1e-3）
%   
%   输出参数：
%       gp_model - 高斯过程模型结构体，包含训练数据和协方差矩阵
%       y_pred - 测试数据预测均值，n_test×1向量
%       y_std - 测试数据预测标准差，n_test×1向量

    % 设置默认参数
    if nargin < 6
        noise_var = 1e-3;
    end
    
    % 确保输入数据格式正确
    x_train = x_train(:,:);
    y_train = y_train(:);
    x_test = x_test(:,:);
    
    n_train = size(x_train, 1);
    n_test = size(x_test, 1);
    
    % 计算训练数据的协方差矩阵
    K = compute_covariance_matrix(x_train, x_train, kernel_type, kernel_params);
    K_noisy = K + noise_var * eye(n_train);
    
    % 计算测试数据与训练数据的协方差矩阵
    K_star = compute_covariance_matrix(x_test, x_train, kernel_type, kernel_params);
    
    % 计算测试数据自身的协方差矩阵
    K_star_star = compute_covariance_matrix(x_test, x_test, kernel_type, kernel_params);
    
    % 计算预测均值和协方差
    % 求解线性方程组：alpha = K_noisy \ y_train
    L = chol(K_noisy, 'lower');
    alpha = L' \ (L \ y_train);
    
    % 预测均值
    y_pred = K_star * alpha;
    
    % 预测协方差
    v = L \ K_star';
    cov_pred = K_star_star - v' * v;
    
    % 预测标准差
    y_std = sqrt(diag(cov_pred));
    y_std = y_std(:);
    
    % 保存模型
    gp_model.x_train = x_train;
    gp_model.y_train = y_train;
    gp_model.K = K;
    gp_model.K_noisy = K_noisy;
    gp_model.L = L;
    gp_model.alpha = alpha;
    gp_model.kernel_type = kernel_type;
    gp_model.kernel_params = kernel_params;
    gp_model.noise_var = noise_var;
end

function K = compute_covariance_matrix(x1, x2, kernel_type, kernel_params)
% COMPUTE_COVARIANCE_MATRIX 计算协方差矩阵
%   K = compute_covariance_matrix(x1, x2, kernel_type, kernel_params) 计算x1和x2之间的协方差矩阵

    n1 = size(x1, 1);
    n2 = size(x2, 1);
    K = zeros(n1, n2);
    
    switch lower(kernel_type)
        case 'rbf'
            % RBF核函数：k(x1, x2) = exp(-||x1-x2||²/(2σ²))
            sigma = kernel_params(1);
            for i = 1:n1
                for j = 1:n2
                    K(i,j) = exp(-norm(x1(i,:)-x2(j,:))^2/(2*sigma^2));
                end
            end
            
        case 'matern'
            % Matérn核函数（ν=5/2）：k(x1,x2) = (1+sqrt(5)*r/sigma + 5*r^2/(3*sigma^2))*exp(-sqrt(5)*r/sigma)
            % 其中r=||x1-x2||
            sigma = kernel_params(1);
            for i = 1:n1
                for j = 1:n2
                    r = norm(x1(i,:)-x2(j,:));
                    arg = sqrt(5)*r/sigma;
                    K(i,j) = (1 + arg + 5*r^2/(3*sigma^2)) * exp(-arg);
                end
            end
            
        case 'linear'
            % 线性核函数：k(x1, x2) = x1'*x2 + c
            if length(kernel_params) >= 1
                c = kernel_params(1);
            else
                c = 0;
            end
            K = x1 * x2' + c;
            
        case 'periodic'
            % 周期性核函数：k(x1,x2) = exp(-2*sin^2(pi*||x1-x2||/p)/(2*sigma^2))
            sigma = kernel_params(1);
            period = kernel_params(2);
            for i = 1:n1
                for j = 1:n2
                    r = norm(x1(i,:)-x2(j,:));
                    K(i,j) = exp(-2*sin(pi*r/period)^2/(sigma^2));
                end
            end
            
        otherwise
            error('不支持的核函数类型');
    end
end

function [best_params, best_log_likelihood] = optimize_hyperparameters(x_train, y_train, kernel_type, initial_params, noise_var)
% OPTIMIZE_HYPERPARAMETERS 优化高斯过程核函数超参数
%   [best_params, best_log_likelihood] = optimize_hyperparameters(x_train, y_train, kernel_type, initial_params, noise_var)
%   使用最大似然估计优化核函数超参数

    % 定义目标函数：负对数边际似然
    objective = @(params) -compute_log_marginal_likelihood(x_train, y_train, kernel_type, params, noise_var);
    
    % 使用fmincon进行优化
    options = optimset('Display', 'off', 'MaxIter', 100);
    best_params = fmincon(objective, initial_params, [], [], [], [], 1e-3*ones(size(initial_params)), 1e3*ones(size(initial_params)), [], options);
    
    % 计算最佳对数边际似然
    best_log_likelihood = -objective(best_params);
end

function log_likelihood = compute_log_marginal_likelihood(x_train, y_train, kernel_type, kernel_params, noise_var)
% COMPUTE_LOG_MARGINAL_LIKELIHOOD 计算对数边际似然

    n_train = length(y_train);
    K = compute_covariance_matrix(x_train, x_train, kernel_type, kernel_params);
    K_noisy = K + noise_var * eye(n_train);
    
    % 计算Cholesky分解
    L = chol(K_noisy, 'lower');
    
    % 计算对数边际似然
    alpha = L' \ (L \ y_train);
    log_likelihood = -0.5 * y_train' * alpha - sum(log(diag(L))) - n_train/2 * log(2*pi);
end