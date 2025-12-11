% 测试控制系统矩阵分析功能
% 这个脚本文件可以直接运行，用于测试control_system_analysis.m中的功能

% 创建测试系统
A = [-2 1; -3 0];
B = [0; 1];
C = [1 0];
disp('测试系统：');
disp('A =');
disp(A);
disp('B =');
disp(B);
disp('C =');
disp(C);

% 测试状态转移矩阵
disp('\n=== 测试状态转移矩阵 ===');
Phi = state_transition_matrix(A, 1);

% 测试可控标准型转换
disp('\n=== 测试可控标准型转换 ===');
[Ac, Bc, Cc, Tc] = controllable_canonical_form(A, B, C);

% 测试可观标准型转换
disp('\n=== 测试可观标准型转换 ===');
[Ao, Bo, Co, To] = observable_canonical_form(A, B, C);

% 测试系统稳定性
disp('\n=== 测试系统稳定性 ===');
[is_stable, eigenvalues] = system_stability(A);
