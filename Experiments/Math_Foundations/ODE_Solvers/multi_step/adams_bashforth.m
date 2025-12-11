function [t, y] = adams_bashforth(f, t0, y0, t_end, h, order)
% ADAMS_BASHFORTH 实现Adams-Bashforth多步方法求解常微分方程
%   [t, y] = adams_bashforth(f, t0, y0, t_end, h, order) 使用Adams-Bashforth方法求解常微分方程
%   
%   输入参数：
%       f - 导数函数，形式为f(t, y)，返回y的导数
%       t0 - 初始时间
%       y0 - 初始状态向量
%       t_end - 结束时间
%       h - 时间步长
%       order - 方法阶数（2, 3, 或4）
%   
%   输出参数：
%       t - 时间向量
%       y - 状态向量矩阵，每行对应一个时间步的状态

    % 检查阶数是否有效
    if ~ismember(order, [2, 3, 4])
        error('Adams-Bashforth方法只支持2、3、4阶');
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
    
    % 使用RK4方法初始化前几个时间步
    % 先计算前order-1个时间步的导数
    f_values = zeros(order, n);
    f_values(1, :) = f(t(1), y(1, :)')';
    
    for i = 1:order-1
        % 使用RK4方法计算下一个时间步
        [t_rk, y_rk] = rk4(f, t(i), y(i, :)', t(i+1), h);
        y(i+1, :) = y_rk(end, :);
        f_values(i+1, :) = f(t(i+1), y(i+1, :)')';
    end
    
    % Adams-Bashforth系数
    coefficients = {
        [3/2, -1/2],       % 2阶
        [23/12, -16/12, 5/12],  % 3阶
        [55/24, -59/24, 37/24, -9/24] % 4阶
    };
    
    coeff = coefficients{order-1};
    
    % 主循环
    for i = order:N
        % 计算当前时间步的解
        % Adams-Bashforth公式：y_{n+1} = y_n + h * (a0*f_n + a1*f_{n-1} + ... + ak*f_{n-k})
        % 确保系数与导数历史正确匹配
        weighted_sum = zeros(1, n);
        for j = 1:order
            weighted_sum = weighted_sum + coeff(j) * f_values(order-j+1, :);
        end
        
        % 计算下一个状态
        y(i+1, :) = y(i, :) + h * weighted_sum;
        
        % 更新导数历史记录
        % 将新的导数添加到开头，移除最旧的
        for j = order:-1:2
            f_values(j, :) = f_values(j-1, :);
        end
        f_values(1, :) = f(t(i+1), y(i+1, :)')';
    end
end