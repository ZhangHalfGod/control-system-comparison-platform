% 可观标准型转换
% 输入：系统矩阵A, 输入矩阵B, 输出矩阵C
% 输出：可观标准型矩阵Ao, Bo, Co，以及转换矩阵T
function [Ao, Bo, Co, T] = observable_canonical_form(A, B, C)
    % 计算可观标准型
    % 输入：系统矩阵A, 输入矩阵B, 输出矩阵C
    % 输出：可观标准型矩阵Ao, Bo, Co，以及转换矩阵T
    
    n = size(A, 1);
    
    % 计算特征多项式系数（使用companion矩阵的方法）
    poly_coeffs = poly(A);
    a = poly_coeffs(2:end);  % 去掉首项系数
    
    % 计算可观性矩阵
    Qo = observability_matrix(A, C);
    
    % 检查可观性
    if rank(Qo) < n
        warning('系统不可观，无法转换为可观标准型');
        Ao = A;
        Bo = B;
        Co = C;
        T = eye(n);
        return;
    end
    
    % 构造转换矩阵T的逆
    T_inv = zeros(n, n);
    T_inv(1, :) = C;
    
    for i = 2:n
        T_inv(i, :) = T_inv(i-1, :) * A;
    end
    
    % 计算转换矩阵T
    T = inv(T_inv);
    
    % 计算可观标准型
    Ao = T \ A * T;
    Bo = T \ B;
    Co = C * T;
    
    disp('可观标准型转换完成');
    disp('可观标准型矩阵Ao：');
    disp(Ao);
    disp('可观标准型输入矩阵Bo：');
    disp(Bo);
    disp('可观标准型输出矩阵Co：');
    disp(Co);
end

function Qo = observability_matrix(A, C)
    % 计算可观性矩阵
    n = size(A, 1);
    Qo = C;
    for i = 1:n-1
        Qo = [Qo; C*A^i];
    end
end