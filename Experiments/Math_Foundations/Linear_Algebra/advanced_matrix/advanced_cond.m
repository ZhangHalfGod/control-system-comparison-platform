% 矩阵条件数计算
% 输入：矩阵A，范数类型p（默认2）
% 输出：矩阵A的条件数
function cond_num = advanced_cond(A, p)
    if nargin < 2
        p = 2; % 默认使用2-范数
    end
    
    % 使用MATLAB内置的cond函数实现
    cond_num = cond(A, p);
    
    disp(['矩阵条件数（', num2str(p), '-范数）：', num2str(cond_num)]);
end