% MATLAB Control Toolbox 深度使用演示脚本
% 演示控制系统的建模、分析和设计功能
% 版本：1.0
% 日期：2025-12-23

%% 1. 控制系统建模
clear;
clc;
close all;

% 直流电机模型参数
J = 0.01;      % 转动惯量 (kg·m²)
B = 0.1;       % 阻尼系数 (N·m·s/rad)
K = 0.01;      % 电机常数 (V·s/rad)
R = 1;         % 电阻 (Ω)
L = 0.5;       % 电感 (H)

% 连续状态空间模型
A = [ -B/J   K/J     ;
      -K/L  -R/L     ];
B = [ 0      ;
      1/L    ];
C = [ 1  0   ];
D = 0;

ss_cont = ss(A, B, C, D);

% 离散化模型 (Ts = 0.1s)
Ts = 0.1;
ss_disc = c2d(ss_cont, Ts, 'zoh');

%% 2. 控制系统分析
figure;

% 阶跃响应分析
subplot(2, 2, 1);
step(ss_cont, ss_disc);
title('阶跃响应对比');
legend('连续系统', '离散系统');
grid on;

% 伯德图分析
subplot(2, 2, 2);
bode(ss_cont, ss_disc);
title('伯德图对比');
legend('连续系统', '离散系统');
grid on;

% 根轨迹分析
subplot(2, 2, 3);
rltitle('连续系统根轨迹');
rgrid;
grid on;

% 奈奎斯特图分析
subplot(2, 2, 4);
nyquist(ss_cont);
title('连续系统奈奎斯特图');
grid on;

%% 3. PID控制器设计与整定
% 使用Ziegler-Nichols方法整定PID参数
[Ku, Pu] = margin(ss_cont); % 获取临界增益和周期

% 计算PID参数
Kp = 0.6 * Ku;
Ti = Pu / 2;
Td = Pu / 8;

% 创建PID控制器
pid_controller = pid(Kp, Ti, Td);

% 闭环系统
cl_system = feedback(pid_controller * ss_cont, 1);

% 仿真结果
figure;
step(cl_system);
title('PID控制闭环阶跃响应');
grid on;

%% 4. 系统性能指标计算
[y, t] = step(cl_system);

% 上升时间 (从10%到90%的时间)
[y_max, idx_max] = max(y);
y_ss = y(end);
t_r = t(find(y >= 0.9*y_ss, 1)) - t(find(y >= 0.1*y_ss, 1));

% 超调量
overshoot = (y_max - y_ss) / y_ss * 100;

% 调节时间 (2%误差带)
t_settle = t(find(abs(y - y_ss) <= 0.02*y_ss, 1, 'last'));

% 稳态误差
e_ss = 1 - y_ss;

% 显示性能指标
fprintf('系统性能指标：\n');
fprintf('上升时间：%.4f s\n', t_r);
fprintf('超调量：%.2f%%\n', overshoot);
fprintf('调节时间：%.4f s\n', t_settle);
fprintf('稳态误差：%.4f\n', e_ss);

%% 5. 控制系统优化
% 使用sisotool进行交互式设计
sisotool(ss_cont);

%% 6. 结果保存
save('control_toolbox_demo_results.mat', 'ss_cont', 'ss_disc', 'pid_controller', 'cl_system', 't_r', 'overshoot', 't_settle', 'e_ss');

fprintf('\nMATLAB Control Toolbox 演示完成！\n');
fprintf('结果已保存到 control_toolbox_demo_results.mat\n');