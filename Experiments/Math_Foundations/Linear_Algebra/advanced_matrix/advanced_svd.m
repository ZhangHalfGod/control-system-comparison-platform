% 奇异值分解（SVD）
% 输入：矩阵A
% 输出：U, S, V，满足A = U*S*V'
% 其中U和V是正交矩阵，S是对角矩阵
function [U, S, V] = advanced_svd(A)
    % 使用MATLAB内置的svd函数实现
    [U, S, V] = svd(A);
    
    disp('奇异值分解完成');
    disp('奇异值：');
    disp(diag(S));
end