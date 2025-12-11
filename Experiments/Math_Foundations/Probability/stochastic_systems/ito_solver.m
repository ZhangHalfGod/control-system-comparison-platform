function [t, y] = ito_solver(f, g, t0, y0, t_end, h, num_paths)
% ITO_SOLVER 实现伊藤随机微分方程的数值求解
%   [t, y] = ito_solver(f, g, t0, y0, t_end, h, num_paths)
%   使用欧拉-马里亚纳方法求解伊藤随机微分方程
%   
%   伊藤随机微分方程形式：dy(t) = f(t, y(t))dt + g(t, y(t))dW(t)
%   其中W(t)是维纳过程
%   
%   输入参数：
%       f - 漂移项函数，形式为f(t, y)，返回y的漂移项导数
%       g - 扩散项函数，形式为g(t, y)，返回扩散项矩阵
%       t0 - 初始时间
%       y0 - 初始状态向量
%       t_end - 结束时间
%       h - 时间步长
%       num_paths - 模拟路径数量（默认1）
%   
%   输出参数：
%       t - 时间向量
%       y - 状态向量矩阵，num_paths×(N+1)×n数组，N为时间步数，n为状态维度

    % 设置默认参数
    if nargin < 7
        num_paths = 1;
    end
    
    % 计算时间步数
    N = ceil((t_end - t0) / h);
    t = t0:h:t_end;
    if t(end) < t_end
        t = [t, t_end];
        N = length(t) - 1;
    end
    
    % 初始化状态矩阵
    y0 = y0(:); % 确保y0是列向量
    n = length(y0);
    y = zeros(num_paths, N+1, n);
    y(:, 1, :) = repmat(y0', num_paths, 1);
    
    % 主循环
    for i = 1:N
        current_t = t(i);
        current_y = squeeze(y(:, i, :))';
        
        % 计算漂移项
        f_values = zeros(n, num_paths);
        for j = 1:num_paths
            f_values(:, j) = f(current_t, current_y(:, j));
        end
        
        % 计算扩散项
        g_values = zeros(n, n, num_paths);
        for j = 1:num_paths
            g_values(:, :, j) = g(current_t, current_y(:, j));
        end
        
        % 生成维纳过程增量（布朗运动）
        dW = sqrt(h) * randn(n, num_paths);
        
        % 应用欧拉-马里亚纳方法更新状态
        for j = 1:num_paths
            y(j, i+1, :) = current_y(:, j)' + f_values(:, j)'*h + g_values(:, :, j)*dW(:, j)';
        end
    end
end

function [t, y] = milstein_solver(f, g, dg, t0, y0, t_end, h, num_paths)
% MILSTEIN_SOLVER 实现Milstein方法求解伊藤随机微分方程
%   [t, y] = milstein_solver(f, g, dg, t0, y0, t_end, h, num_paths)
%   使用Milstein方法求解伊藤随机微分方程，具有更高的收敛阶
%   
%   输入参数：
%       f - 漂移项函数，形式为f(t, y)
%       g - 扩散项函数，形式为g(t, y)
%       dg - 扩散项的导数，形式为dg(t, y, k)，返回∂g/∂y_k
%       其他参数同ito_solver
%   
%   输出参数：
%       t - 时间向量
%       y - 状态向量矩阵，num_paths×(N+1)×n数组

    % 设置默认参数
    if nargin < 8
        num_paths = 1;
    end
    
    % 计算时间步数
    N = ceil((t_end - t0) / h);
    t = t0:h:t_end;
    if t(end) < t_end
        t = [t, t_end];
        N = length(t) - 1;
    end
    
    % 初始化状态矩阵
    y0 = y0(:); % 确保y0是列向量
    n = length(y0);
    y = zeros(num_paths, N+1, n);
    y(:, 1, :) = repmat(y0', num_paths, 1);
    
    % 主循环
    for i = 1:N
        current_t = t(i);
        current_y = squeeze(y(:, i, :))';
        
        % 计算漂移项
        f_values = zeros(n, num_paths);
        for j = 1:num_paths
            f_values(:, j) = f(current_t, current_y(:, j));
        end
        
        % 计算扩散项
        g_values = zeros(n, n, num_paths);
        for j = 1:num_paths
            g_values(:, :, j) = g(current_t, current_y(:, j));
        end
        
        % 生成维纳过程增量
        dW = sqrt(h) * randn(n, num_paths);
        
        % 生成Milstein修正项需要的增量
        dW_sq = dW.^2 - h;
        
        % 应用Milstein方法更新状态
        for j = 1:num_paths
            % 欧拉-马里亚纳项
            y_euler = current_y(:, j) + f_values(:, j)*h + g_values(:, :, j)*dW(:, j);
            
            % Milstein修正项
            milstein_corr = zeros(n, 1);
            for k = 1:n
                for l = 1:n
                    % 计算dg/dy_l * g_lk
                    dg_dyl = dg(current_t, current_y(:, j), l);
                    milstein_corr = milstein_corr + 0.5 * dg_dyl * g_values(:, l, j) * dW_sq(k, j);
                end
            end
            
            y(j, i+1, :) = (y_euler + milstein_corr)';
        end
    end
end

function lyapunov_exponents = compute_lyapunov_exponents(f, g, t0, y0, t_end, h, num_paths)
% COMPUTE_LYAPUNOV_EXPONENTS 计算随机系统的李雅普诺夫指数
%   lyapunov_exponents = compute_lyapunov_exponents(f, g, t0, y0, t_end, h, num_paths)
%   计算随机系统的最大李雅普诺夫指数
%   
%   输入参数：
%       同ito_solver
%   
%   输出参数：
%       lyapunov_exponents - 李雅普诺夫指数，num_paths×n数组

    % 求解伊藤方程，得到多条路径
    [t, y] = ito_solver(f, g, t0, y0, t_end, h, num_paths);
    
    N = length(t) - 1;
    n = size(y, 3);
    lyapunov_exponents = zeros(num_paths, n);
    
    % 对每条路径计算李雅普诺夫指数
    for path = 1:num_paths
        % 计算状态轨迹的范数
        y_path = squeeze(y(path, :, :));
        norm_y = zeros(N+1, 1);
        for i = 1:N+1
            norm_y(i) = norm(y_path(i, :)');
        end
        
        % 计算相邻时间步的状态变化率
        delta_y = zeros(N, n);
        for i = 1:N
            delta_y(i, :) = (y_path(i+1, :) - y_path(i, :)) / h;
        end
        
        % 计算最大李雅普诺夫指数
        for dim = 1:n
            % 对每个维度计算李雅普诺夫指数
            log_ratio = log(norm(y_path(:, dim)') / norm(y_path(1, dim)'));
            lyapunov_exponents(path, dim) = log_ratio / (t_end - t0);
        end
    end
end

function [mean_y, var_y] = stochastic_moments(f, g, t0, y0, t_end, h, num_paths)
% STOCHASTIC_MOMENTS 计算随机系统的统计矩
%   [mean_y, var_y] = stochastic_moments(f, g, t0, y0, t_end, h, num_paths)
%   计算随机系统的均值和方差
%   
%   输入参数：
%       同ito_solver
%   
%   输出参数：
%       mean_y - 均值向量，(N+1)×n数组
%       var_y - 方差向量，(N+1)×n数组

    % 求解伊藤方程，得到多条路径
    [t, y] = ito_solver(f, g, t0, y0, t_end, h, num_paths);
    
    N = length(t) - 1;
    n = size(y, 3);
    mean_y = zeros(N+1, n);
    var_y = zeros(N+1, n);
    
    % 计算每个时间步的均值和方差
    for i = 1:N+1
        for dim = 1:n
            y_dim = squeeze(y(:, i, dim));
            mean_y(i, dim) = mean(y_dim);
            var_y(i, dim) = var(y_dim);
        end
    end
end