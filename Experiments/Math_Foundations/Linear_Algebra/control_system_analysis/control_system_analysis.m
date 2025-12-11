% 控制系统矩阵分析实现
% 包括状态转移矩阵计算、可控标准型和可观标准型转换、系统稳定性分析

function Phi = state_transition_matrix(A, t)
    % 状态转移矩阵计算
    % 输入：系统矩阵A，时间t
    % 输出：状态转移矩阵Phi(t) = exp(A*t)
    
    % 使用MATLAB内置的expm函数实现矩阵指数计算
    Phi = expm(A * t);
    
    disp(['时间t=', num2str(t), '时的状态转移矩阵：']);
    disp(Phi);
end

function [Ac, Bc, Cc, T] = controllable_canonical_form(A, B, C)
    % 可控标准型转换
    % 输入：系统矩阵A, 输入矩阵B, 输出矩阵C
    % 输出：可控标准型矩阵Ac, Bc, Cc，以及转换矩阵T
    
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

function [Ao, Bo, Co, T] = observable_canonical_form(A, B, C)
    % 可观标准型转换
    % 输入：系统矩阵A, 输入矩阵B, 输出矩阵C
    % 输出：可观标准型矩阵Ao, Bo, Co，以及转换矩阵T
    
    % 计算可观性矩阵
    n = size(A, 1);
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
    
    % 构造转换矩阵T
    T = [observability_coefficients(A, C) * Qo(:, end:-1:1)];
    T = inv(T);
    
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

function [is_stable, eigenvalues] = system_stability(A)
    % 系统稳定性分析
    % 输入：系统矩阵A
    % 输出：is_stable（系统是否稳定），特征值eigenvalues
    
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

function Qc = controllability_matrix(A, B)
    % 计算可控性矩阵
    n = size(A, 1);
    Qc = B;
    for i = 1:n-1
        Qc = [Qc, A^i*B];
    end
end

function Qo = observability_matrix(A, C)
    % 计算可观性矩阵
    n = size(A, 1);
    Qo = C;
    for i = 1:n-1
        Qo = [Qo; C*A^i];
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

function coeffs = observability_coefficients(A, C)
    % 计算可观性系数
    n = size(A, 1);
    coeffs = zeros(1, n);
    for i = 1:n
        coeffs(i) = trace(A^(n-i));
    end
end

function test_control_system_analysis()
    % 测试控制系统矩阵分析功能
    
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
end
