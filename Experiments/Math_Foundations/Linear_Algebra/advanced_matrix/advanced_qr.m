% QR分解
% 输入：矩阵A
% 输出：Q, R，满足A = Q*R
% 其中Q是正交矩阵，R是上三角矩阵
function [Q, R] = advanced_qr(A)
    % 使用MATLAB内置的qr函数实现
    [Q, R] = qr(A);
    
    disp('QR分解完成');
    disp('上三角矩阵R：');
    disp(R);
end