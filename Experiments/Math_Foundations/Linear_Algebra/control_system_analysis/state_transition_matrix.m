% 状态转移矩阵计算
% 输入：系统矩阵A，时间t
% 输出：状态转移矩阵Phi(t) = exp(A*t)
function Phi = state_transition_matrix(A, t)
    % 使用MATLAB内置的expm函数实现矩阵指数计算
    Phi = expm(A * t);
    
    disp(['时间t=', num2str(t), '时的状态转移矩阵：']);
    disp(Phi);
end