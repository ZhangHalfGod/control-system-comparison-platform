import numpy as np

def matrix_operations():
    """
    矩阵基础运算演示
    """
    print("=== 矩阵基础运算 ===")
    
    # 创建测试矩阵
    A = np.array([[1, 2], [3, 4]])
    B = np.array([[5, 6], [7, 8]])
    
    print("矩阵A:")
    print(A)
    print("\n矩阵B:")
    print(B)
    
    # 矩阵加法
    C = A + B
    print("\n矩阵加法 A + B:")
    print(C)
    
    # 矩阵乘法
    D = np.dot(A, B)
    print("\n矩阵乘法 A * B:")
    print(D)
    
    # 矩阵转置
    E = A.T
    print("\n矩阵转置 A.T:")
    print(E)
    
    # 矩阵求逆
    try:
        F = np.linalg.inv(A)
        print("\n矩阵求逆 A⁻¹:")
        print(F)
        
        # 验证逆矩阵
        print("\n验证 A * A⁻¹:")
        print(np.dot(A, F))
    except np.linalg.LinAlgError:
        print("\n矩阵A不可逆")
    
    # 矩阵行列式
    det_A = np.linalg.det(A)
    print(f"\n矩阵A的行列式: {det_A:.4f}")
    
    return {
        'A': A,
        'B': B,
        'sum': C,
        'product': D,
        'transpose': E,
        'inverse': F if 'F' in locals() else None,
        'determinant': det_A
    }

def eigenvalue_decomposition(A):
    """
    特征值分解
    返回特征值和特征向量
    """
    eigenvalues, eigenvectors = np.linalg.eig(A)
    return eigenvalues, eigenvectors

def check_stability(A):
    """
    检查系统稳定性
    如果所有特征值的实部都小于0，则系统稳定
    """
    eigenvalues, _ = eigenvalue_decomposition(A)
    real_parts = np.real(eigenvalues)
    is_stable = all(real_parts < 0)
    
    print("=== 系统稳定性分析 ===")
    print(f"系统矩阵A:")
    print(A)
    print(f"\n特征值: {eigenvalues}")
    print(f"特征值实部: {real_parts}")
    print(f"系统是否稳定: {'是' if is_stable else '否'}")
    
    return {
        'eigenvalues': eigenvalues,
        'real_parts': real_parts,
        'is_stable': is_stable
    }

def dc_motor_state_space(params):
    """
    构建直流电机的状态空间模型
    params = [J, b, Kt, Ke, R, L]
    返回状态空间矩阵 A, B, C, D
    """
    J, b, Kt, Ke, R, L = params
    
    # 状态变量：x = [ω, i]ᵀ
    # 输入：u = [V]（电压）
    # 输出：y = [ω]（转速）
    
    # 状态矩阵 A
    A = np.array([
        [-b/J, Kt/J],
        [-Ke/L, -R/L]
    ])
    
    # 输入矩阵 B
    B = np.array([
        [0],
        [1/L]
    ])
    
    # 输出矩阵 C
    C = np.array([[1, 0]])
    
    # 直接传输矩阵 D
    D = np.array([[0]])
    
    return A, B, C, D

def controllability_matrix(A, B):
    """
    计算能控性矩阵
    能控性矩阵：[B, AB, A²B, ..., A^(n-1)B]
    """
    n = A.shape[0]  # 状态维度
    C_mat = B
    
    for i in range(1, n):
        C_mat = np.hstack((C_mat, np.dot(np.linalg.matrix_power(A, i), B)))
    
    return C_mat

def observability_matrix(A, C):
    """
    计算能观性矩阵
    能观性矩阵：[C; CA; CA²; ...; CA^(n-1)]
    """
    n = A.shape[0]  # 状态维度
    O_mat = C
    
    for i in range(1, n):
        O_mat = np.vstack((O_mat, np.dot(C, np.linalg.matrix_power(A, i))))
    
    return O_mat

def check_controllability(A, B):
    """
    检查系统能控性
    如果能控性矩阵的秩等于状态维度，则系统能控
    """
    C_mat = controllability_matrix(A, B)
    rank = np.linalg.matrix_rank(C_mat)
    n = A.shape[0]
    is_controllable = (rank == n)
    
    print("=== 系统能控性分析 ===")
    print(f"能控性矩阵:")
    print(C_mat)
    print(f"能控性矩阵秩: {rank}")
    print(f"状态维度: {n}")
    print(f"系统是否能控: {'是' if is_controllable else '否'}")
    
    return {
        'controllability_matrix': C_mat,
        'rank': rank,
        'is_controllable': is_controllable
    }

def check_observability(A, C):
    """
    检查系统能观性
    如果能观性矩阵的秩等于状态维度，则系统能观
    """
    O_mat = observability_matrix(A, C)
    rank = np.linalg.matrix_rank(O_mat)
    n = A.shape[0]
    is_observable = (rank == n)
    
    print("=== 系统能观性分析 ===")
    print(f"能观性矩阵:")
    print(O_mat)
    print(f"能观性矩阵秩: {rank}")
    print(f"状态维度: {n}")
    print(f"系统是否能观: {'是' if is_observable else '否'}")
    
    return {
        'observability_matrix': O_mat,
        'rank': rank,
        'is_observable': is_observable
    }

def pole_placement(A, B, desired_poles):
    """
    极点配置法设计状态反馈增益矩阵K
    注意：此函数需要scipy库
    """
    try:
        from scipy.signal import place_poles
        
        # 使用scipy的place_poles函数进行极点配置
        result = place_poles(A, B, desired_poles)
        K = result.gain_matrix
        
        # 验证闭环系统特征值
        A_cl = A - np.dot(B, K)
        eigenvalues, _ = eigenvalue_decomposition(A_cl)
        
        print("=== 极点配置结果 ===")
        print(f"期望极点: {desired_poles}")
        print(f"状态反馈增益K: {K}")
        print(f"闭环系统特征值: {eigenvalues}")
        
        return {
            'K': K,
            'closed_loop_eigenvalues': eigenvalues,
            'A_cl': A_cl
        }
        
    except ImportError:
        print("需要安装scipy库来使用极点配置功能: pip install scipy")
        return None
    except Exception as e:
        print(f"极点配置失败: {e}")
        return None

if __name__ == "__main__":
    # 测试矩阵运算
    matrix_operations()
    
    # 直流电机参数
    J = 0.01     # 转动惯量 (kg·m²)
    b = 0.1      # 阻尼系数 (N·m·s)
    Kt = 0.1     # 转矩常数 (N·m/A)
    Ke = 0.1     # 反电动势常数 (V·s/rad)
    R = 1.0      # 电阻 (Ω)
    L = 0.1      # 电感 (H)
    
    motor_params = [J, b, Kt, Ke, R, L]
    
    # 构建直流电机状态空间模型
    A, B, C, D = dc_motor_state_space(motor_params)
    
    print("\n=== 直流电机状态空间模型 ===")
    print(f"状态矩阵 A:")
    print(A)
    print(f"\n输入矩阵 B:")
    print(B)
    print(f"\n输出矩阵 C:")
    print(C)
    print(f"\n直接传输矩阵 D:")
    print(D)
    
    # 检查系统稳定性
    check_stability(A)
    
    # 检查系统能控性
    check_controllability(A, B)
    
    # 检查系统能观性
    check_observability(A, C)
    
    # 测试极点配置（可选）
    print("\n=== 极点配置测试 ===")
    desired_poles = np.array([-10.0, -20.0])  # 期望极点（更稳定）
    pole_placement(A, B, desired_poles)
