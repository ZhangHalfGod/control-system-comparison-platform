% 直流电机状态空间模型测试脚本
% MATLAB R2024a

%% 1. 定义直流电机参数
J = 0.01;     % 转动惯量 (kg·m²)
b = 0.1;      % 阻尼系数 (N·m·s)
Kt = 0.1;     % 转矩常数 (N·m/A)
Ke = 0.1;     % 反电动势常数 (V·s/rad)
R = 1.0;      % 电阻 (Ω)
L = 0.1;      % 电感 (H)

%% 2. 构建状态空间模型
% 状态变量：x = [ω; i]，其中ω是转速，i是电流
% 输入：u = V（电压）
% 输出：y = ω（转速）

A = [ -b/J,  Kt/J;
      -Ke/L, -R/L  ];

B = [ 0;
      1/L ];

C = [ 1, 0 ];

D = [ 0 ];

% 创建状态空间对象
motor_ss = ss(A, B, C, D);

%% 3. 系统稳定性分析
eigenvalues = eig(A);
real_parts = real(eigenvalues);
is_stable = all(real_parts < 0);

fprintf('=== 直流电机系统稳定性分析 ===\n');
fprintf('系统矩阵 A:\n');
disp(A);
fprintf('\n特征值:');
disp(eigenvalues);
fprintf('特征值实部:');
disp(real_parts);
% 使用MATLAB兼容的条件判断语法
if is_stable
    stable_str = '是';
else
    stable_str = '否';
end
fprintf('系统是否稳定: %s\n\n', stable_str);

%% 4. 能控性分析
controllability_mat = ctrb(A, B);
controllability_rank = rank(controllability_mat);
is_controllable = (controllability_rank == size(A, 1));

fprintf('=== 能控性分析 ===\n');
fprintf('能控性矩阵:\n');
disp(controllability_mat);
fprintf('能控性矩阵秩: %d\n', controllability_rank);
if is_controllable
    controllable_str = '是';
else
    controllable_str = '否';
end
fprintf('系统是否能控: %s\n\n', controllable_str);

%% 5. 能观性分析
observability_mat = obsv(A, C);
observability_rank = rank(observability_mat);
is_observable = (observability_rank == size(A, 1));

fprintf('=== 能观性分析 ===\n');
fprintf('能观性矩阵:\n');
disp(observability_mat);
fprintf('能观性矩阵秩: %d\n', observability_rank);
if is_observable
    observable_str = '是';
else
    observable_str = '否';
end
fprintf('系统是否能观: %s\n\n', observable_str);

%% 6. 阶跃响应分析
fprintf('=== 阶跃响应分析 ===\n');
t = 0:0.01:2;  % 时间范围
u = 1;         % 阶跃输入电压

% 连续系统阶跃响应
[y_cont, t_cont] = step(motor_ss, t);

%% 7. 离散化方法对比
Ts = 0.1;  % 采样时间

% 零阶保持离散化
motor_d_zoh = c2d(motor_ss, Ts, 'zoh');
[y_d_zoh, t_d_zoh] = step(motor_d_zoh);  % 让step函数自动生成合适的时间向量

% 一阶保持离散化
motor_d_foh = c2d(motor_ss, Ts, 'foh');
[y_d_foh, t_d_foh] = step(motor_d_foh);  % 让step函数自动生成合适的时间向量

% 双线性变换离散化
motor_d_tustin = c2d(motor_ss, Ts, 'tustin');
[y_d_tustin, t_d_tustin] = step(motor_d_tustin);  % 让step函数自动生成合适的时间向量

%% 8. 绘制响应曲线
figure('Position', [100, 100, 1000, 600]);

subplot(2, 2, 1);
plot(t_cont, y_cont, 'LineWidth', 2);
title('连续系统阶跃响应');
xlabel('时间 (s)');
ylabel('转速 (rad/s)');
grid on;

subplot(2, 2, 2);
plot(t_d_zoh, y_d_zoh, 'LineWidth', 2);
title('离散系统阶跃响应 (ZOH)');
xlabel('时间 (s)');
ylabel('转速 (rad/s)');
grid on;

subplot(2, 2, 3);
plot(t_d_foh, y_d_foh, 'LineWidth', 2);
title('离散系统阶跃响应 (FOH)');
xlabel('时间 (s)');
ylabel('转速 (rad/s)');
grid on;

subplot(2, 2, 4);
plot(t_d_tustin, y_d_tustin, 'LineWidth', 2);
title('离散系统阶跃响应 (Tustin)');
xlabel('时间 (s)');
ylabel('转速 (rad/s)');
grid on;

%% 9. 极点配置设计
fprintf('=== 极点配置设计 ===\n');

% 期望极点（更稳定的闭环系统）
desired_poles = [-10, -20];

% 计算状态反馈增益 K
K = place(A, B, desired_poles);
fprintf('状态反馈增益 K: ');
disp(K);

% 闭环系统矩阵
A_cl = A - B * K;
closed_loop_eigenvalues = eig(A_cl);
fprintf('闭环系统特征值: ');
disp(closed_loop_eigenvalues);

%% 10. 闭环系统响应
motor_cl_ss = ss(A_cl, B, C, D);
[y_cl, t_cl] = step(motor_cl_ss, t);

% 绘制闭环响应
figure('Position', [100, 100, 800, 600]);
plot(t_cont, y_cont, 'b--', 'LineWidth', 2, 'DisplayName', '开环系统');
hold on;
plot(t_cl, y_cl, 'r-', 'LineWidth', 2, 'DisplayName', '闭环系统（极点配置）');
title('直流电机系统响应对比');
xlabel('时间 (s)');
ylabel('转速 (rad/s)');
legend('Location', 'best');
grid on;
hold off;

fprintf('\n=== 实验完成 ===\n');
fprintf('所有结果已绘制在图形窗口中。\n');
