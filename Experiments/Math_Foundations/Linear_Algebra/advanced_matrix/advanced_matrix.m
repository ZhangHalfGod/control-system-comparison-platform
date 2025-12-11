% 高级矩阵运算实现
% 包括奇异值分解（SVD）、QR分解和矩阵条件数计算

function [U, S, V] = advanced_svd(A)
    % 奇异值分解（SVD）
    % 输入：矩阵A
    % 输出：U, S, V，满足A = U*S*V'
    % 其中U和V是正交矩阵，S是对角矩阵
    
    % 使用MATLAB内置的svd函数实现
    [U, S, V] = svd(A);
    
    disp('奇异值分解完成');
    disp('奇异值：');
    disp(diag(S));
end

function [Q, R] = advanced_qr(A)
    % QR分解
    % 输入：矩阵A
    % 输出：Q, R，满足A = Q*R
    % 其中Q是正交矩阵，R是上三角矩阵
    
    % 使用MATLAB内置的qr函数实现
    [Q, R] = qr(A);
    
    disp('QR分解完成');
    disp('上三角矩阵R：');
    disp(R);
end

function cond_num = advanced_cond(A, p)
    % 矩阵条件数计算
    % 输入：矩阵A，范数类型p（默认2）
    % 输出：矩阵A的条件数
    
    if nargin < 2
        p = 2; % 默认使用2-范数
    end
    
    % 使用MATLAB内置的cond函数实现
    cond_num = cond(A, p);
    
    disp(['矩阵条件数（', num2str(p), '-范数）：', num2str(cond_num)]);
end

function test_advanced_matrix()
    % 测试高级矩阵运算功能
    
    % 创建测试矩阵
    A = [1 2 3; 4 5 6; 7 8 9];
    disp('测试矩阵A：');
    disp(A);
    
    % 测试奇异值分解
    disp('\n=== 测试奇异值分解 ===');
    [U, S, V] = advanced_svd(A);
    
    % 测试QR分解
    disp('\n=== 测试QR分解 ===');
    [Q, R] = advanced_qr(A);
    
    % 测试矩阵条件数
    disp('\n=== 测试矩阵条件数 ===');
    cond_2 = advanced_cond(A, 2);
    cond_1 = advanced_cond(A, 1);
    cond_inf = advanced_cond(A, inf);
    
    % 测试结果验证
    disp('\n=== 结果验证 ===');
    % SVD验证
    A_recon = U * S * V';
    disp(['SVD重构误差：', num2str(norm(A - A_recon))]);
    
    % QR验证
    A_recon_qr = Q * R;
    disp(['QR重构误差：', num2str(norm(A - A_recon_qr))]);
end
