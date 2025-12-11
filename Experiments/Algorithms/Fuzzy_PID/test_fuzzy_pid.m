% 模糊PID控制器测试脚本
% 功能：测试模糊PID控制器的性能并与传统PID控制器对比
% 测试系统：一阶惯性系统和二阶振荡系统

clc;
clear;
close all;

% 测试1：一阶惯性系统
fprintf('=== 测试1：一阶惯性系统 ===\n');
first_order_test();

% 测试2：二阶振荡系统
fprintf('\n=== 测试2：二阶振荡系统 ===\n');
second_order_test();

% 测试3：非线性系统
fprintf('\n=== 测试3：非线性系统 ===\n');
nonlinear_system_test();

% 一阶惯性系统测试函数
function first_order_test()
    % 系统模型：G(s) = 1/(s+1)
    % 离散化，采样时间Ts=0.1s
    Ts = 0.1;
    A = exp(-Ts);
    B = 1 - exp(-Ts);
    
    % 传统PID参数
    kp_pid = 2.0;
    ki_pid = 0.5;
    kd_pid = 0.1;
    
    % 模糊PID初始参数
    kp_fuzzy = 2.0;
    ki_fuzzy = 0.5;
    kd_fuzzy = 0.1;
    
    % 模拟参数
    t_end = 10; % 模拟时间10秒
    t = 0:Ts:t_end;
    n = length(t);
    
    % 参考输入（阶跃信号）
    r = ones(n, 1);
    
    % 初始化变量
    y_pid = zeros(n, 1);
    y_fuzzy = zeros(n, 1);
    u_pid = zeros(n, 1);
    u_fuzzy = zeros(n, 1);
    e_pid = zeros(n, 1);
    e_fuzzy = zeros(n, 1);
    ec_pid = zeros(n, 1);
    ec_fuzzy = zeros(n, 1);
    integral_pid = 0;
    integral_fuzzy = 0;
    
    % 系统模拟
    for i = 2:n
        % 传统PID控制
        e_pid(i) = r(i) - y_pid(i-1);
        if i > 2
            ec_pid(i) = e_pid(i) - e_pid(i-1);
        end
        integral_pid = integral_pid + e_pid(i) * Ts;
        u_pid(i) = kp_pid * e_pid(i) + ki_pid * integral_pid + kd_pid * ec_pid(i) / Ts;
        y_pid(i) = A * y_pid(i-1) + B * u_pid(i);
        
        % 模糊PID控制
        e_fuzzy(i) = r(i) - y_fuzzy(i-1);
        if i > 2
            ec_fuzzy(i) = e_fuzzy(i) - e_fuzzy(i-1);
        else
            ec_fuzzy(i) = e_fuzzy(i) - e_fuzzy(1);
        end
        integral_fuzzy = integral_fuzzy + e_fuzzy(i) * Ts;
        
        % 动态调整PID参数
        [new_kp, new_ki, new_kd] = fuzzy_pid_controller(e_fuzzy(i), ec_fuzzy(i), kp_fuzzy, ki_fuzzy, kd_fuzzy);
        
        % 打印参数调整信息（每5个采样点打印一次）
        if mod(i, 5) == 0
            fprintf('时间: %.1f s, e: %.3f, ec: %.3f, 调整前: Kp=%.3f, Ki=%.3f, Kd=%.3f, 调整后: Kp=%.3f, Ki=%.3f, Kd=%.3f\n', ...
                (i-1)*Ts, e_fuzzy(i), ec_fuzzy(i), kp_fuzzy, ki_fuzzy, kd_fuzzy, new_kp, new_ki, new_kd);
        end
        
        % 更新PID参数
        kp_fuzzy = new_kp;
        ki_fuzzy = new_ki;
        kd_fuzzy = new_kd;
        
        u_fuzzy(i) = kp_fuzzy * e_fuzzy(i) + ki_fuzzy * integral_fuzzy + kd_fuzzy * ec_fuzzy(i) / Ts;
        y_fuzzy(i) = A * y_fuzzy(i-1) + B * u_fuzzy(i);
    end
    
    % 性能指标计算
    [overshoot_pid, settling_time_pid, rise_time_pid] = calculate_performance(y_pid, r, Ts);
    [overshoot_fuzzy, settling_time_fuzzy, rise_time_fuzzy] = calculate_performance(y_fuzzy, r, Ts);
    
    fprintf('传统PID：\n');
    fprintf('  超调量：%.2f%%\n', overshoot_pid);
    fprintf('  调节时间：%.2f s\n', settling_time_pid);
    fprintf('  上升时间：%.2f s\n', rise_time_pid);
    
    fprintf('模糊PID：\n');
    fprintf('  超调量：%.2f%%\n', overshoot_fuzzy);
    fprintf('  调节时间：%.2f s\n', settling_time_fuzzy);
    fprintf('  上升时间：%.2f s\n', rise_time_fuzzy);
    
    % 绘制结果
    figure('Name', '一阶惯性系统响应对比', 'Position', [100, 100, 800, 600]);
    subplot(2, 1, 1);
    plot(t, r, 'r--', 'LineWidth', 2, 'DisplayName', '参考输入');
    hold on;
    plot(t, y_pid, 'b-', 'LineWidth', 2, 'DisplayName', '传统PID');
    plot(t, y_fuzzy, 'g-', 'LineWidth', 2, 'DisplayName', '模糊PID');
    xlabel('时间 (s)');
    ylabel('输出');
    title('一阶惯性系统响应');
    grid on;
    legend('Location', 'best');
    
    subplot(2, 1, 2);
    plot(t, u_pid, 'b-', 'LineWidth', 2, 'DisplayName', '传统PID控制量');
    hold on;
    plot(t, u_fuzzy, 'g-', 'LineWidth', 2, 'DisplayName', '模糊PID控制量');
    xlabel('时间 (s)');
    ylabel('控制量');
    title('控制量对比');
    grid on;
    legend('Location', 'best');
end

% 二阶振荡系统测试函数
function second_order_test()
    % 系统模型：G(s) = 1/(s^2 + 0.5s + 1)
    % 离散化，采样时间Ts=0.1s
    Ts = 0.1;
    A = [1, Ts; -Ts, 1 - 0.5*Ts];
    B = [0; Ts];
    C = [1, 0];
    D = 0;
    
    % 传统PID参数
    kp_pid = 3.0;
    ki_pid = 1.0;
    kd_pid = 1.0;
    
    % 模糊PID初始参数
    kp_fuzzy = 3.0;
    ki_fuzzy = 1.0;
    kd_fuzzy = 1.0;
    
    % 模拟参数
    t_end = 20; % 模拟时间20秒
    t = 0:Ts:t_end;
    n = length(t);
    
    % 参考输入（阶跃信号）
    r = ones(n, 1);
    
    % 初始化变量
    x_pid = zeros(2, n);
    x_fuzzy = zeros(2, n);
    y_pid = zeros(n, 1);
    y_fuzzy = zeros(n, 1);
    u_pid = zeros(n, 1);
    u_fuzzy = zeros(n, 1);
    e_pid = zeros(n, 1);
    e_fuzzy = zeros(n, 1);
    ec_pid = zeros(n, 1);
    ec_fuzzy = zeros(n, 1);
    integral_pid = 0;
    integral_fuzzy = 0;
    
    % 系统模拟
    for i = 2:n
        % 传统PID控制
        y_pid(i-1) = C * x_pid(:, i-1) + D * u_pid(i-1);
        e_pid(i) = r(i) - y_pid(i-1);
        if i > 2
            ec_pid(i) = e_pid(i) - e_pid(i-1);
        end
        integral_pid = integral_pid + e_pid(i) * Ts;
        u_pid(i) = kp_pid * e_pid(i) + ki_pid * integral_pid + kd_pid * ec_pid(i) / Ts;
        x_pid(:, i) = A * x_pid(:, i-1) + B * u_pid(i);
        
        % 模糊PID控制
        y_fuzzy(i-1) = C * x_fuzzy(:, i-1) + D * u_fuzzy(i-1);
        e_fuzzy(i) = r(i) - y_fuzzy(i-1);
        if i > 2
            ec_fuzzy(i) = e_fuzzy(i) - e_fuzzy(i-1);
        end
        integral_fuzzy = integral_fuzzy + e_fuzzy(i) * Ts;
        
        % 动态调整PID参数
        [kp_fuzzy, ki_fuzzy, kd_fuzzy] = fuzzy_pid_controller(e_fuzzy(i), ec_fuzzy(i), kp_fuzzy, ki_fuzzy, kd_fuzzy);
        
        u_fuzzy(i) = kp_fuzzy * e_fuzzy(i) + ki_fuzzy * integral_fuzzy + kd_fuzzy * ec_fuzzy(i) / Ts;
        x_fuzzy(:, i) = A * x_fuzzy(:, i-1) + B * u_fuzzy(i);
    end
    
    % 计算最终输出
    y_pid(n) = C * x_pid(:, n) + D * u_pid(n);
    y_fuzzy(n) = C * x_fuzzy(:, n) + D * u_fuzzy(n);
    
    % 性能指标计算
    [overshoot_pid, settling_time_pid, rise_time_pid] = calculate_performance(y_pid, r, Ts);
    [overshoot_fuzzy, settling_time_fuzzy, rise_time_fuzzy] = calculate_performance(y_fuzzy, r, Ts);
    
    fprintf('传统PID：\n');
    fprintf('  超调量：%.2f%%\n', overshoot_pid);
    fprintf('  调节时间：%.2f s\n', settling_time_pid);
    fprintf('  上升时间：%.2f s\n', rise_time_pid);
    
    fprintf('模糊PID：\n');
    fprintf('  超调量：%.2f%%\n', overshoot_fuzzy);
    fprintf('  调节时间：%.2f s\n', settling_time_fuzzy);
    fprintf('  上升时间：%.2f s\n', rise_time_fuzzy);
    
    % 绘制结果
    figure('Name', '二阶振荡系统响应对比', 'Position', [100, 100, 800, 600]);
    subplot(2, 1, 1);
    plot(t, r, 'r--', 'LineWidth', 2, 'DisplayName', '参考输入');
    hold on;
    plot(t, y_pid, 'b-', 'LineWidth', 2, 'DisplayName', '传统PID');
    plot(t, y_fuzzy, 'g-', 'LineWidth', 2, 'DisplayName', '模糊PID');
    xlabel('时间 (s)');
    ylabel('输出');
    title('二阶振荡系统响应');
    grid on;
    legend('Location', 'best');
    
    subplot(2, 1, 2);
    plot(t, u_pid, 'b-', 'LineWidth', 2, 'DisplayName', '传统PID控制量');
    hold on;
    plot(t, u_fuzzy, 'g-', 'LineWidth', 2, 'DisplayName', '模糊PID控制量');
    xlabel('时间 (s)');
    ylabel('控制量');
    title('控制量对比');
    grid on;
    legend('Location', 'best');
end

% 非线性系统测试函数
function nonlinear_system_test()
    % 非线性系统模型：y(k+1) = y(k)/(1+y(k)^2) + u(k)^3
    % 采样时间Ts=0.1s
    Ts = 0.1;
    
    % 传统PID参数
    kp_pid = 0.5;
    ki_pid = 0.1;
    kd_pid = 0.05;
    
    % 模糊PID初始参数
    kp_fuzzy = 0.5;
    ki_fuzzy = 0.1;
    kd_fuzzy = 0.05;
    
    % 模拟参数
    t_end = 15; % 模拟时间15秒
    t = 0:Ts:t_end;
    n = length(t);
    
    % 参考输入（阶跃信号，幅值0.5）
    r = 0.5 * ones(n, 1);
    
    % 初始化变量
    y_pid = zeros(n, 1);
    y_fuzzy = zeros(n, 1);
    u_pid = zeros(n, 1);
    u_fuzzy = zeros(n, 1);
    e_pid = zeros(n, 1);
    e_fuzzy = zeros(n, 1);
    ec_pid = zeros(n, 1);
    ec_fuzzy = zeros(n, 1);
    integral_pid = 0;
    integral_fuzzy = 0;
    
    % 系统模拟
    for i = 2:n
        % 传统PID控制
        e_pid(i) = r(i) - y_pid(i-1);
        if i > 2
            ec_pid(i) = e_pid(i) - e_pid(i-1);
        end
        integral_pid = integral_pid + e_pid(i) * Ts;
        u_pid(i) = kp_pid * e_pid(i) + ki_pid * integral_pid + kd_pid * ec_pid(i) / Ts;
        y_pid(i) = y_pid(i-1)/(1 + y_pid(i-1)^2) + u_pid(i)^3;
        
        % 模糊PID控制
        e_fuzzy(i) = r(i) - y_fuzzy(i-1);
        if i > 2
            ec_fuzzy(i) = e_fuzzy(i) - e_fuzzy(i-1);
        end
        integral_fuzzy = integral_fuzzy + e_fuzzy(i) * Ts;
        
        % 动态调整PID参数
        [kp_fuzzy, ki_fuzzy, kd_fuzzy] = fuzzy_pid_controller(e_fuzzy(i), ec_fuzzy(i), kp_fuzzy, ki_fuzzy, kd_fuzzy);
        
        u_fuzzy(i) = kp_fuzzy * e_fuzzy(i) + ki_fuzzy * integral_fuzzy + kd_fuzzy * ec_fuzzy(i) / Ts;
        y_fuzzy(i) = y_fuzzy(i-1)/(1 + y_fuzzy(i-1)^2) + u_fuzzy(i)^3;
    end
    
    % 性能指标计算
    [overshoot_pid, settling_time_pid, rise_time_pid] = calculate_performance(y_pid, r, Ts);
    [overshoot_fuzzy, settling_time_fuzzy, rise_time_fuzzy] = calculate_performance(y_fuzzy, r, Ts);
    
    fprintf('传统PID：\n');
    fprintf('  超调量：%.2f%%\n', overshoot_pid);
    fprintf('  调节时间：%.2f s\n', settling_time_pid);
    fprintf('  上升时间：%.2f s\n', rise_time_pid);
    
    fprintf('模糊PID：\n');
    fprintf('  超调量：%.2f%%\n', overshoot_fuzzy);
    fprintf('  调节时间：%.2f s\n', settling_time_fuzzy);
    fprintf('  上升时间：%.2f s\n', rise_time_fuzzy);
    
    % 绘制结果
    figure('Name', '非线性系统响应对比', 'Position', [100, 100, 800, 600]);
    subplot(2, 1, 1);
    plot(t, r, 'r--', 'LineWidth', 2, 'DisplayName', '参考输入');
    hold on;
    plot(t, y_pid, 'b-', 'LineWidth', 2, 'DisplayName', '传统PID');
    plot(t, y_fuzzy, 'g-', 'LineWidth', 2, 'DisplayName', '模糊PID');
    xlabel('时间 (s)');
    ylabel('输出');
    title('非线性系统响应');
    grid on;
    legend('Location', 'best');
    
    subplot(2, 1, 2);
    plot(t, u_pid, 'b-', 'LineWidth', 2, 'DisplayName', '传统PID控制量');
    hold on;
    plot(t, u_fuzzy, 'g-', 'LineWidth', 2, 'DisplayName', '模糊PID控制量');
    xlabel('时间 (s)');
    ylabel('控制量');
    title('控制量对比');
    grid on;
    legend('Location', 'best');
end

% 性能指标计算函数
function [overshoot, settling_time, rise_time] = calculate_performance(y, r, Ts)
    % 计算超调量、调节时间和上升时间
    
    % 稳态值（最后10%时间的平均值）
    n = length(y);
    steady_state = mean(y(end - floor(n*0.1) + 1:end));
    
    % 峰值和峰值时间
    [peak, peak_idx] = max(y);
    peak_time = (peak_idx - 1) * Ts;
    
    % 超调量
    overshoot = max(0, (peak - steady_state) / steady_state * 100);
    
    % 上升时间（从10%到90%稳态值的时间）
    y_min = min(y);
    rise_10 = 0.1 * steady_state + 0.9 * y_min;
    rise_90 = 0.9 * steady_state + 0.1 * y_min;
    
    rise_idx1 = find(y >= rise_10, 1);
    rise_idx2 = find(y >= rise_90, 1);
    
    if isempty(rise_idx1) || isempty(rise_idx2)
        rise_time = NaN;
    else
        rise_time = (rise_idx2 - rise_idx1) * Ts;
    end
    
    % 调节时间（进入稳态值±5%范围并保持）
    tolerance = 0.05 * steady_state;
    lower_bound = steady_state - tolerance;
    upper_bound = steady_state + tolerance;
    
    settling_time = NaN;
    for i = peak_idx:n
        if all(y(i:end) >= lower_bound) && all(y(i:end) <= upper_bound)
            settling_time = (i - 1) * Ts;
            break;
        end
    end
    
    % 如果没有找到调节时间，使用最大时间
    if isnan(settling_time)
        settling_time = (n - 1) * Ts;
    end
end