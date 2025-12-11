% 系统稳定性分析
% 输入：系统矩阵A
% 输出：is_stable（系统是否稳定），特征值eigenvalues
function [is_stable, eigenvalues] = system_stability(A)
    % 计算特征值
    eigenvalues = eig(A);
    
    % 检查稳定性：所有特征值的实部小于0
    is_stable = all(real(eigenvalues) < 0);
    
    disp('系统特征值：');
    disp(eigenvalues);
    
    if is_stable
        disp('系统稳定：所有特征值的实部小于0');
    else
        disp('系统不稳定：存在特征值的实部大于等于0');
    end
end