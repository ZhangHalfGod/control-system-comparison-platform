% MATLAB Fuzzy Logic Toolbox 深度使用演示脚本
% 演示模糊控制器的设计、仿真和应用
% 版本：1.0
% 日期：2025-12-23

%% 1. 创建模糊推理系统
clear;
clc;
close all;

% 创建FIS结构
fis = newfis('fuzzy_pid_controller', 'mamdani', 'min', 'max', 'min', 'max', 'centroid');

%% 2. 添加输入变量：误差(e)和误差变化率(de)

% 误差(e) - 范围：[-10, 10]
fis = addvar(fis, 'input', 'e', [-10 10]);
fis = addmf(fis, 'input', 1, 'NB', 'gaussmf', [-10 3]);
fis = addmf(fis, 'input', 1, 'NS', 'gaussmf', [-5 3]);
fis = addmf(fis, 'input', 1, 'ZE', 'gaussmf', [0 3]);
fis = addmf(fis, 'input', 1, 'PS', 'gaussmf', [5 3]);
fis = addmf(fis, 'input', 1, 'PB', 'gaussmf', [10 3]);

% 误差变化率(de) - 范围：[-5, 5]
fis = addvar(fis, 'input', 'de', [-5 5]);
fis = addmf(fis, 'input', 2, 'NB', 'gaussmf', [-5 2]);
fis = addmf(fis, 'input', 2, 'NS', 'gaussmf', [-2.5 2]);
fis = addmf(fis, 'input', 2, 'ZE', 'gaussmf', [0 2]);
fis = addmf(fis, 'input', 2, 'PS', 'gaussmf', [2.5 2]);
fis = addmf(fis, 'input', 2, 'PB', 'gaussmf', [5 2]);

%% 3. 添加输出变量：控制量(u)

% 控制量(u) - 范围：[-100, 100]
fis = addvar(fis, 'output', 'u', [-100 100]);
fis = addmf(fis, 'output', 1, 'NB', 'gaussmf', [-100 30]);
fis = addmf(fis, 'output', 1, 'NS', 'gaussmf', [-50 30]);
fis = addmf(fis, 'output', 1, 'ZE', 'gaussmf', [0 30]);
fis = addmf(fis, 'output', 1, 'PS', 'gaussmf', [50 30]);
fis = addmf(fis, 'output', 1, 'PB', 'gaussmf', [100 30]);

%% 4. 设计模糊规则库（49条规则）

% 规则矩阵：e(de, e) -> u
% 行：de的模糊集（NB, NS, ZE, PS, PB）
% 列：e的模糊集（NB, NS, ZE, PS, PB）
rule_matrix = [
    % NB       NS       ZE       PS       PB
    'NB',    'NB',    'NB',    'NS',    'ZE'; % de=NB
    'NB',    'NB',    'NS',    'ZE',    'PS'; % de=NS
    'NB',    'NS',    'ZE',    'PS',    'PB'; % de=ZE
    'NS',    'ZE',    'PS',    'PB',    'PB'; % de=PS
    'ZE',    'PS',    'PB',    'PB',    'PB'; % de=PB
];

% 添加规则
for i = 1:5
    for j = 1:5
        fis = addrule(fis, [i, j, str2double(cell2mat(rule_matrix(i,j))), 1]);
    end
end

% 修复规则编号
for i = 1:25
    fis.Rules(i).Connection = 1;
    fis.Rules(i).Weight = 1;
end

%% 5. 可视化模糊系统

% 输入输出隶属度函数图
figure;
subplot(2, 2, 1);
plotmf(fis, 'input', 1);
title('误差(e)隶属度函数');
grid on;

subplot(2, 2, 2);
plotmf(fis, 'input', 2);
title('误差变化率(de)隶属度函数');
grid on;

subplot(2, 2, 3);
plotmf(fis, 'output', 1);
title('控制量(u)隶属度函数');
grid on;

% 模糊规则查看器
figure;
ruleview(fis);

% 模糊推理查看器
figure;
surfview(fis);
title('模糊推理曲面');

%% 6. 模糊控制器仿真测试

% 测试输入
e_test = 5;
de_test = -2;

% 模糊推理
u_output = evalfis([e_test, de_test], fis);

fprintf('模糊控制器测试结果：\n');
fprintf('误差(e) = %.2f\n', e_test);
fprintf('误差变化率(de) = %.2f\n', de_test);
fprintf('控制量(u) = %.2f\n', u_output);

%% 7. 模糊控制器与传统PID对比

% 直流电机模型
J = 0.01;
B = 0.1;
K = 0.01;
R = 1;
L = 0.5;

A = [ -B/J   K/J     ;
      -K/L  -R/L     ];
B = [ 0      ;
      1/L    ];
C = [ 1  0   ];
D = 0;

ss_cont = ss(A, B, C, D);

% 传统PID控制器
Kp = 10;
Ti = 1;
Td = 0.1;
pid_controller = pid(Kp, Ti, Td);
cl_pid = feedback(pid_controller * ss_cont, 1);

% 模糊控制器封装为SISO系统
fuzzy_controller = @(t, x, u) evalfis([u(1), x(2)], fis);

%% 8. 保存模糊系统
writefis(fis, 'fuzzy_pid_controller.fis');

fprintf('\nMATLAB Fuzzy Logic Toolbox 演示完成！\n');
fprintf('模糊系统已保存到 fuzzy_pid_controller.fis\n');