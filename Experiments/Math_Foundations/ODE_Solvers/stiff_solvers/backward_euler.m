function [t, y] = backward_euler(f, t0, y0, t_end, h, max_iter, tol)
% BACKWARD_EULER 实现Backward Euler方法求解刚性常微分方程
%   [t, y] = backward_euler(f, t0, y0, t_end, h, max_iter, tol) 使用Backward Euler方法求解常微分方程
%   
%   输入参数：
%       f - 导数函数，形式为f(t, y)，返回y的导数
%       t0 - 初始时间
%       y0 - 初始状态向量
%       t_end - 结束时间
%       h - 时间步长
%       max_iter - 最大牛顿迭代次数（默认100）
%       tol - 迭代收敛 tolerance（默认1e-6）
%   
%   输出参数：
%       t - 时间向量
%       y - 状态向量矩阵，每行对应一个时间步的状态

    % 设置默认参数
    if nargin < 6
        max_iter = 100;
    end
    if nargin < 7
        tol = 1e-6;
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
    
    % 主循环
    for i = 1:N
        % 时间步i的时间
        t_prev = t(i);
        t_curr = t(i+1);
        y_prev = y(i, :)';
        
        % 使用牛顿迭代求解隐式方程：y_curr = y_prev + h*f(t_curr, y_curr)
        % 初始猜测：使用前向欧拉预测
        y_guess = y_prev + h*f(t_prev, y_prev);
        
        for iter = 1:max_iter
            % 计算当前函数值
            f_curr = f(t_curr, y_guess);
            F = y_guess - y_prev - h*f_curr;
            
            % 计算雅可比矩阵
            J = eye(n) - h*jacobian(@(y) f(t_curr, y), y_guess);
            
            % 求解线性方程组 J*delta_y = -F
            delta_y = J \ (-F);
            
            % 更新猜测值
            y_guess_new = y_guess + delta_y;
            
            % 检查收敛性
            if norm(delta_y) < tol
                break;
            end
            
            if iter == max_iter
                warning('Backward Euler牛顿迭代未收敛，达到最大迭代次数');
            end
            
            y_guess = y_guess_new;
        end
        
        % 保存结果
        y(i+1, :) = y_guess';
    end
end

function J = jacobian(f, y)
% JACOBIAN 数值计算雅可比矩阵
%   J = jacobian(f, y) 计算函数f在点y处的雅可比矩阵
%   
%   输入参数：
%       f - 向量值函数，f(y)返回列向量
%       y - 计算雅可比矩阵的点
%   
%   输出参数：
%       J - 雅可比矩阵，J(i,j) = df_i/dy_j

    y = y(:);
    n = length(y);
    f0 = f(y);
    m = length(f0);
    J = zeros(m, n);
    
    % 使用中心差分法计算雅可比矩阵
    h_jac = 1e-8;
    for j = 1:n
        y_plus = y;
        y_plus(j) = y_plus(j) + h_jac;
        y_minus = y;
        y_minus(j) = y_minus(j) - h_jac;
        
        f_plus = f(y_plus);
        f_minus = f(y_minus);
        
        J(:, j) = (f_plus - f_minus) / (2*h_jac);
    end
end