import numpy as np
import matplotlib.pyplot as plt

def dc_motor_ode(state, t, params):
    """
    直流电机的状态空间方程
    状态变量：state = [ω, i]，其中ω是转速，i是电流
    输入：params = [J, b, Kt, Ke, R, L, u]，其中u是输入电压
    """
    ω, i = state
    J, b, Kt, Ke, R, L, u = params
    
    # 状态方程：
    # dω/dt = (1/J) * (Kt*i - b*ω)
    # di/dt = (1/L) * (u - R*i - Ke*ω)
    
    dω_dt = (Kt * i - b * ω) / J
    di_dt = (u - R * i - Ke * ω) / L
    
    return np.array([dω_dt, di_dt])

def runge_kutta4(f, state0, t_span, dt, params):
    """
    四阶龙格-库塔法求解ODE
    f: 状态方程函数
    state0: 初始状态
    t_span: 时间范围 (t0, tf)
    dt: 时间步长
    params: 模型参数
    """
    t0, tf = t_span
    t = np.arange(t0, tf, dt)
    n = len(t)
    
    # 初始化状态数组
    state = np.zeros((n, len(state0)))
    state[0] = state0
    
    # 龙格-库塔迭代
    for i in range(n-1):
        k1 = f(state[i], t[i], params) * dt
        k2 = f(state[i] + k1/2, t[i] + dt/2, params) * dt
        k3 = f(state[i] + k2/2, t[i] + dt/2, params) * dt
        k4 = f(state[i] + k3, t[i] + dt, params) * dt
        
        state[i+1] = state[i] + (k1 + 2*k2 + 2*k3 + k4) / 6
    
    return t, state

def ode_comparison():
    """
    对比MATLAB ODE求解器与RK4实现的差异
    注意：此函数需要安装scipy库来调用odeint进行对比
    """
    try:
        from scipy.integrate import odeint
        
        # 电机参数
        J = 0.01     # 转动惯量 (kg·m²)
        b = 0.1      # 阻尼系数 (N·m·s)
        Kt = 0.1     # 转矩常数 (N·m/A)
        Ke = 0.1     # 反电动势常数 (V·s/rad)
        R = 1.0      # 电阻 (Ω)
        L = 0.1      # 电感 (H)
        u = 1.0      # 输入电压 (V)
        
        params = [J, b, Kt, Ke, R, L, u]
        
        # 初始状态：转速=0，电流=0
        state0 = [0.0, 0.0]
        
        # 时间范围和步长
        t_span = (0.0, 2.0)
        dt = 0.01
        
        # 使用RK4求解
        t_rk4, state_rk4 = runge_kutta4(dc_motor_ode, state0, t_span, dt, params)
        
        # 使用scipy的odeint求解（作为参考）
        t_odeint = np.arange(t_span[0], t_span[1], dt)
        state_odeint = odeint(dc_motor_ode, state0, t_odeint, args=(params,))
        
        # 计算误差
        error_omega = np.abs(state_rk4[:, 0] - state_odeint[:, 0])
        error_current = np.abs(state_rk4[:, 1] - state_odeint[:, 1])
        
        # 绘制结果
        fig, axes = plt.subplots(3, 1, figsize=(10, 12))
        
        # 转速对比
        axes[0].plot(t_rk4, state_rk4[:, 0], label='RK4', linewidth=2)
        axes[0].plot(t_odeint, state_odeint[:, 0], label='odeint', linestyle='--')
        axes[0].set_ylabel('转速 (rad/s)')
        axes[0].legend()
        axes[0].grid(True)
        
        # 电流对比
        axes[1].plot(t_rk4, state_rk4[:, 1], label='RK4', linewidth=2)
        axes[1].plot(t_odeint, state_odeint[:, 1], label='odeint', linestyle='--')
        axes[1].set_ylabel('电流 (A)')
        axes[1].legend()
        axes[1].grid(True)
        
        # 误差分析
        axes[2].plot(t_rk4, error_omega, label='转速误差', linewidth=2)
        axes[2].plot(t_rk4, error_current, label='电流误差', linewidth=2)
        axes[2].set_xlabel('时间 (s)')
        axes[2].set_ylabel('误差')
        axes[2].legend()
        axes[2].grid(True)
        
        plt.tight_layout()
        plt.savefig('dc_motor_ode_comparison.png')
        plt.close()
        
        print("ODE求解对比完成，结果已保存到dc_motor_ode_comparison.png")
        print(f"最大转速误差: {np.max(error_omega):.6f}")
        print(f"最大电流误差: {np.max(error_current):.6f}")
        
        return {
            't_rk4': t_rk4,
            'state_rk4': state_rk4,
            't_odeint': t_odeint,
            'state_odeint': state_odeint,
            'max_error_omega': np.max(error_omega),
            'max_error_current': np.max(error_current)
        }
        
    except ImportError:
        print("需要安装scipy库来运行对比测试: pip install scipy")
        return None

if __name__ == "__main__":
    # 测试RK4求解器
    print("测试直流电机RK4求解器...")
    
    # 电机参数
    J = 0.01     # 转动惯量 (kg·m²)
    b = 0.1      # 阻尼系数 (N·m·s)
    Kt = 0.1     # 转矩常数 (N·m/A)
    Ke = 0.1     # 反电动势常数 (V·s/rad)
    R = 1.0      # 电阻 (Ω)
    L = 0.1      # 电感 (H)
    u = 1.0      # 输入电压 (V)
    
    params = [J, b, Kt, Ke, R, L, u]
    state0 = [0.0, 0.0]  # 初始状态
    t_span = (0.0, 2.0)
    dt = 0.01
    
    t, state = runge_kutta4(dc_motor_ode, state0, t_span, dt, params)
    
    # 绘制结果
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(t, state[:, 0])
    plt.ylabel('转速 (rad/s)')
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(t, state[:, 1])
    plt.xlabel('时间 (s)')
    plt.ylabel('电流 (A)')
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('dc_motor_rk4_result.png')
    plt.close()
    
    print("RK4求解完成，结果已保存到dc_motor_rk4_result.png")
    print("\n最终状态:")
    print(f"转速: {state[-1, 0]:.4f} rad/s")
    print(f"电流: {state[-1, 1]:.4f} A")
    
    # 运行对比测试
    print("\n运行ODE求解对比测试...")
    ode_comparison()
