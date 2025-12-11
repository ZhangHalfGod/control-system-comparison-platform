% 测试高级矩阵运算功能
% 这个脚本文件可以直接运行，用于测试advanced_matrix.m中的功能

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
