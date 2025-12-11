function [t, y] = adams_moulton(f, t0, y0, t_end, h, order, max_iter, tol)
% ADAMS_MOULTON 实现Adams-Moulton多步方法求解常微分方程
%   [t, y] = adams_moulton(f, t0, y0, t_end, h, order, max_iter, tol) 使用Adams-Moulton方法求解常微分方程
%   
%   输入参数：
%       f - 导数函数，形式为f(t, y)，返回y的导数
%       t0 - 初始时间
%       y0 - 初始状态向量
%       t_end - 结束时间
%       h - 时间步长
%       order - 方法阶数（2, 3, 或4）
%       max_iter - 最大迭代次数（默认100）
%       tol - 迭代收敛 tolerance（默认1e-6）
%   
%   输出参数：
%       t - 时间向量
%       y - 状态向量矩阵，每行对应一个时间步的状态

    % 设置默认参数
    if nargin < 7
        max_iter = 100;
    end
    if nargin < 8
        tol = 1e-6;
    end
    
    % 检查阶数是否有效
    if ~ismember(order, [2, 3, 4])
        error('Adams-Moulton方法只支持2、3、4阶');
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
    y = zeros(N+1, n);
    y(1, :) = y0';
    
    % Adams-Moulton系数（隐式部分）
    coefficients = {
        [1/2, 1/2],         % 2阶
        [5/12, 8/12, -1/12],  % 3阶
        [9/24, 19/24, -5/24, 1/24] % 4阶
    };
    
    coeff = coefficients{order-1};
    
    % Adams-Bashforth系数（用于预测）
    ab_coefficients = {
        [3/2, -1/2],       % 2阶
        [23/12, -16/12, 5/12],  % 3阶
        [55/24, -59/24, 37/24, -9/24] % 4阶
    };
    
    ab_coeff = ab_coefficients{order-1};
    
    % 使用RK4方法初始化前几个时间步
    f_values = zeros(order, n);
    f_values(1, :) = f(t(1), y(1, :)')';
    
    for i = 1:order-1
        % 使用RK4方法计算下一个时间步
        [t_rk, y_rk] = rk4(f, t(i), y(i, :)', t(i+1), h);
        y(i+1, :) = y_rk(end, :);
        f_values(i+1, :) = f(t(i+1), y(i+1, :)')';
    end
    
    % 主循环
    for i = order:N
        % 步骤1：使用Adams-Bashforth方法预测
        % 计算加权和
        ab_weighted_sum = zeros(1, n);
        for j = 1:order
            ab_weighted_sum = ab_weighted_sum + ab_coeff(j) * f_values(order-j+1, :);
        end
        
        % 预测值
        y_pred = y(i, :) + h * ab_weighted_sum;
        
        % 步骤2：使用Adams-Moulton方法校正（迭代求解）
        y_corr = y_pred;
        for iter = 1:max_iter
            y_prev = y_corr;
            
            % 计算当前导数
            f_curr = f(t(i+1), y_corr(:))';
            
            % 计算Adams-Moulton加权和
            am_weighted_sum = zeros(1, n);
            am_weighted_sum = am_weighted_sum + coeff(1) * f_curr;
            
            % 只循环到系数数组的长度
            for j = 2:length(coeff)
                am_weighted_sum = am_weighted_sum + coeff(j) * f_values(j-1, :);
            end
            
            % 校正值
            y_corr_new = y(i, :) + h * am_weighted_sum;
            
            % 检查收敛性
            if norm(y_corr_new - y_prev) < tol
                break;
            end
            
            if iter == max_iter
                warning('Adams-Moulton迭代未收敛，达到最大迭代次数');
            end
            
            y_corr = y_corr_new;
        end
        
        % 保存校正后的结果
        y(i+1, :) = y_corr;
        
        % 更新导数历史记录
        for j = order:-1:2
            f_values(j, :) = f_values(j-1, :);
        end
        f_values(1, :) = f(t(i+1), y(i+1, :)')';
    end
end