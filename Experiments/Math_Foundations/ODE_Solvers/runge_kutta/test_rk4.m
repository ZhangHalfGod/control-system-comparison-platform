% 测试四阶龙格-库塔方法（RK4）
% 1. 与MATLAB内置ODE求解器对比
% 2. 分析不同步长对求解精度的影响

% 定义测试问题：一阶线性ODE
% dy/dt = -2y + sin(t)
% 解析解：y(t) = (2/5)e^(-2t) + (1/5)sin(t) - (2/5)cos(t)

test_1st_order_ode();

% 定义测试问题：二阶非线性ODE（范德波尔振荡器）
% d^2x/dt^2 - mu(1-x^2)dx/dt + x = 0
% 转换为一阶系统：
% dx/dt = y
% dy/dt = mu(1-x^2)y - x
test_vanderpol_oscillator();

function test_1st_order_ode()
    disp('=== 测试一阶线性ODE ===');
    
    % 定义导数函数
    f = @(t, y) -2*y + sin(t);
    
    % 解析解（正确版本）
    y_analytic = @(t) (1/5)*exp(-2*t) + (2/5)*sin(t) - (1/5)*cos(t);
    
    % 初始条件和时间范围
    t0 = 0;
    y0 = [0];
    t_end = 5;
    
    % 不同步长
    h_values = [0.5, 0.1, 0.05, 0.01];
    
    % 使用MATLAB内置ode45求解
    [t_ode45, y_ode45] = ode45(f, [t0, t_end], y0);
    
    % 计算解析解
    t_analytic = linspace(t0, t_end, 1000);
    y_exact = y_analytic(t_analytic);
    
    % 绘制结果对比
    figure('Name', '一阶线性ODE求解结果对比');
    hold on;
    plot(t_analytic, y_exact, 'k-', 'LineWidth', 2, 'DisplayName', '解析解');
    plot(t_ode45, y_ode45, 'r--', 'LineWidth', 1.5, 'DisplayName', 'ode45');
    
    % 使用不同步长的RK4求解并比较
    errors = zeros(length(h_values), 1);
    
    for i = 1:length(h_values)
        h = h_values(i);
        [t_rk4, y_rk4] = rk4(f, t0, y0, t_end, h);
        
        % 计算数值解与解析解的最大误差
        y_rk4_interp = interp1(t_rk4, y_rk4, t_analytic, 'linear');
        errors(i) = max(abs(y_rk4_interp - y_exact));
        
        % 绘制RK4结果
        plot(t_rk4, y_rk4, 'o-', 'LineWidth', 1, 'DisplayName', ['RK4 h=', num2str(h)]);
    end
    
    xlabel('时间 t');
    ylabel('y(t)');
    title('一阶线性ODE: dy/dt = -2y + sin(t) 求解结果对比');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 绘制误差随步长变化的曲线
    figure('Name', 'RK4求解精度与步长关系（一阶线性ODE）');
    loglog(h_values, errors, 'o-', 'LineWidth', 2);
    xlabel('步长 h（对数坐标）');
    ylabel('最大误差（对数坐标）');
    title('RK4求解精度与步长关系');
    grid on;
    
    % 输出误差分析结果
    disp('\n=== 一阶线性ODE误差分析 ===');
    disp('步长 h   | 最大误差');
    disp('-------------------');
    for i = 1:length(h_values)
        disp(sprintf('%6.4f | %10.8e', h_values(i), errors(i)));
    end
    
    % 计算收敛阶
    disp('\n=== 收敛阶分析 ===');
    for i = 1:length(h_values)-1
        ratio = errors(i) / errors(i+1);
        h_ratio = h_values(i) / h_values(i+1);
        order = log(ratio) / log(h_ratio);
        disp(sprintf('h=%.4f到h=%.4f: 收敛阶≈%.2f', h_values(i), h_values(i+1), order));
    end
end

function test_vanderpol_oscillator()
    disp('\n=== 测试范德波尔振荡器（二阶非线性ODE） ===');
    
    % 定义参数
    mu = 1.0;
    
    % 定义导数函数（转换为一阶系统）
    f = @(t, y) [y(2); mu*(1-y(1)^2)*y(2) - y(1)];
    
    % 初始条件和时间范围
    t0 = 0;
    y0 = [2; 0]; % 初始位置为2，初始速度为0
    t_end = 20;
    
    % 使用MATLAB内置ode45求解
    [t_ode45, y_ode45] = ode45(f, [t0, t_end], y0);
    
    % 使用不同步长的RK4求解
    h_values = [0.5, 0.1, 0.05];
    
    % 绘制结果对比
    figure('Name', '范德波尔振荡器求解结果对比（位置）');
    hold on;
    plot(t_ode45, y_ode45(:, 1), 'r--', 'LineWidth', 1.5, 'DisplayName', 'ode45');
    
    for i = 1:length(h_values)
        h = h_values(i);
        [t_rk4, y_rk4] = rk4(f, t0, y0, t_end, h);
        plot(t_rk4, y_rk4(:, 1), 'o-', 'LineWidth', 1, 'DisplayName', ['RK4 h=', num2str(h)]);
    end
    
    xlabel('时间 t');
    ylabel('位置 x(t)');
    title(['范德波尔振荡器: mu=', num2str(mu)]);
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 绘制速度对比
    figure('Name', '范德波尔振荡器求解结果对比（速度）');
    hold on;
    plot(t_ode45, y_ode45(:, 2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'ode45');
    
    for i = 1:length(h_values)
        h = h_values(i);
        [t_rk4, y_rk4] = rk4(f, t0, y0, t_end, h);
        plot(t_rk4, y_rk4(:, 2), 'o-', 'LineWidth', 1, 'DisplayName', ['RK4 h=', num2str(h)]);
    end
    
    xlabel('时间 t');
    ylabel('速度 v(t)');
    title(['范德波尔振荡器速度: mu=', num2str(mu)]);
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 绘制相图
    figure('Name', '范德波尔振荡器相图');
    hold on;
    plot(y_ode45(:, 1), y_ode45(:, 2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'ode45');
    
    for i = 1:length(h_values)
        h = h_values(i);
        [t_rk4, y_rk4] = rk4(f, t0, y0, t_end, h);
        plot(y_rk4(:, 1), y_rk4(:, 2), 'o-', 'LineWidth', 1, 'DisplayName', ['RK4 h=', num2str(h)]);
    end
    
    xlabel('位置 x');
    ylabel('速度 v');
    title(['范德波尔振荡器相图: mu=', num2str(mu)]);
    legend('Location', 'best');
    grid on;
    hold off;
end