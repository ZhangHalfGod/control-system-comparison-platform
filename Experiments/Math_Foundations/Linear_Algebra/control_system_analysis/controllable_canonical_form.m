% 可控标准型转换
% 输入：系统矩阵A, 输入矩阵B, 输出矩阵C
% 输出：可控标准型矩阵Ac, Bc, Cc，以及转换矩阵T
function [Ac, Bc, Cc, T] = controllable_canonical_form(A, B, C)
    % 计算可控性矩阵
    n = size(A, 1);
    Qc = controllability_matrix(A, B);
    
    % 检查可控性
    if rank(Qc) < n
        warning('系统不可控，无法转换为可控标准型');
        Ac = A;
        Bc = B;
        Cc = C;
        T = eye(n);
        return;
    end
    
    % 构造转换矩阵T
    T = [Qc(:, end:-1:1) * diag(1./controllability_coefficients(A, B))];
    
    % 计算可控标准型
    Ac = T \ A * T;
    Bc = T \ B;
    Cc = C * T;
    
    disp('可控标准型转换完成');
    disp('可控标准型矩阵Ac：');
    disp(Ac);
    disp('可控标准型输入矩阵Bc：');
    disp(Bc);
    disp('可控标准型输出矩阵Cc：');
    disp(Cc);
end

function Qc = controllability_matrix(A, B)
    % 计算可控性矩阵
    n = size(A, 1);
    Qc = B;
    for i = 1:n-1
        Qc = [Qc, A^i*B];
    end
end

function coeffs = controllability_coefficients(A, B)
    % 计算可控性系数
    n = size(A, 1);
    coeffs = zeros(n, 1);
    for i = 1:n
        coeffs(i) = trace(A^(n-i));
    end
end