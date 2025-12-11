function [t, y, h_used] = rkf45(f, t0, y0, t_end, h_init, tol, h_min, h_max)
% RKF45 实现Runge-Kutta-Fehlberg (RKF45)方法，带有自适应步长控制
%   [t, y, h_used] = rkf45(f, t0, y0, t_end, h_init, tol, h_min, h_max) 使用RKF45方法求解常微分方程
%   
%   输入参数：
%       f - 导数函数，形式为f(t, y)，返回y的导数
%       t0 - 初始时间
%       y0 - 初始状态向量
%       t_end - 结束时间
%       h_init - 初始时间步长（默认0.1）
%       tol - 误差容限（默认1e-6）
%       h_min - 最小允许步长（默认1e-10）
%       h_max - 最大允许步长（默认1e+3）
%   
%   输出参数：
%       t - 时间向量
%       y - 状态向量矩阵，每行对应一个时间步的状态
%       h_used - 实际使用的时间步长向量

    % 设置默认参数
    if nargin < 6
        h_init = 0.1;
    end
    if nargin < 7
        tol = 1e-6;
    end
    if nargin < 8
        h_min = 1e-10;
    end
    if nargin < 9
        h_max = 1e+3;
    end
    
    % 初始化
    t = t0;
    y0 = y0(:); % 确保y0是列向量
    y = y0';
    h_used = [];
    
    % RKF45系数
    a = [0, 1/4, 3/8, 12/13, 1, 1/2];
    b = [
        [0, 0, 0, 0, 0],
        [1/4, 0, 0, 0, 0],
        [3/32, 9/32, 0, 0, 0],
        [1932/2197, -7200/2197, 7296/2197, 0, 0],
        [439/216, -8, 3680/513, -845/4104, 0],
        [-8/27, 2, -3544/2565, 1859/4104, -11/40]
    ];
    c4 = [25/216, 0, 1408/2565, 2197/4104, -1/5, 0];
    c5 = [16/135, 0, 6656/12825, 28561/56430, -9/50, 2/55];
    
    % 安全系数
    safety = 0.9;
    max_factor = 5.0;
    min_factor = 0.2;
    
    % 主循环
    current_t = t0;
    current_y = y0;
    current_h = h_init;
    
    while current_t < t_end
        % 确保最后一步不会超过t_end
        if current_t + current_h > t_end
            current_h = t_end - current_t;
        end
        
        % 计算k1到k6
        k = zeros(6, length(y0));
        k(1, :) = current_h * f(current_t, current_y)';
        
        for i = 2:6
            y_temp = current_y + sum(b(i, 1:i-1)'.*k(1:i-1, :), 1)';
            k(i, :) = current_h * f(current_t + a(i)*current_h, y_temp)';
        end
        
        % 计算4阶和5阶估计
        y4 = current_y + sum(c4'.*k, 1)';
        y5 = current_y + sum(c5'.*k, 1)';
        
        % 计算局部截断误差
        error = norm(y5 - y4);
        
        % 计算新的步长
        if error < tol
            % 步长可接受，更新时间和状态
            current_t = current_t + current_h;
            current_y = y5;
            t = [t, current_t];
            y = [y; current_y'];
            h_used = [h_used, current_h];
            
            % 计算下一个步长
            if error == 0
                h_new = current_h * max_factor;
            else
                h_new = safety * current_h * (tol/error)^(1/5);
            end
            h_new = min(max(h_new, min_factor*current_h), max_factor*current_h);
            h_new = min(h_new, h_max);
            h_new = max(h_new, h_min);
        else
            % 步长不可接受，缩小步长重试
            h_new = safety * current_h * (tol/error)^(1/5);
            h_new = max(h_new, min_factor*current_h);
            h_new = max(h_new, h_min);
        end
        
        current_h = h_new;
    end
end