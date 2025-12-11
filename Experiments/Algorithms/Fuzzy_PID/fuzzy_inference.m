% 模糊推理引擎实现
% 功能：实现模糊PID控制器的模糊推理过程
% 输入：e - 误差
%       ec - 误差变化率
% 输出：delta_kp - Kp调整量
%       delta_ki - Ki调整量
%       delta_kd - Kd调整量

function [delta_kp, delta_ki, delta_kd] = fuzzy_inference(e, ec)
    % 定义模糊子集中心和标准差（高斯隶属度函数参数）
    % 输入变量：误差e和误差变化率ec
    % 模糊子集：NB(负大), NM(负中), NS(负小), ZO(零), PS(正小), PM(正中), PB(正大)
    e_centers = [-3, -2, -1, 0, 1, 2, 3];
    ec_centers = [-3, -2, -1, 0, 1, 2, 3];
    sigma = 1.0; % 高斯函数标准差
    
    % 输出变量：Kp、Ki、Kd调整量
    % 模糊子集：NB(负大), NM(负中), NS(负小), ZO(零), PS(正小), PM(正中), PB(正大)
    delta_centers = [-3, -2, -1, 0, 1, 2, 3];
    
    % 计算输入变量的隶属度
    mu_e = zeros(7, 1);
    mu_ec = zeros(7, 1);
    
    for i = 1:7
        mu_e(i) = gaussian_mf(e, e_centers(i), sigma);
        mu_ec(i) = gaussian_mf(ec, ec_centers(i), sigma);
    end
    
    % 定义49条模糊规则（7×7）
    % 规则格式：如果e是A且ec是B，则delta_kp是C，delta_ki是D，delta_kd是E
    % 规则矩阵：[e_subset, ec_subset, delta_kp_subset, delta_ki_subset, delta_kd_subset]
    rules = [
        1, 1, 7, 1, 7; % 如果e是NB且ec是NB，则delta_kp是PB，delta_ki是NB，delta_kd是PB
        1, 2, 7, 1, 6; % 如果e是NB且ec是NM，则delta_kp是PB，delta_ki是NB，delta_kd是PM
        1, 3, 6, 1, 5; % 如果e是NB且ec是NS，则delta_kp是PM，delta_ki是NB，delta_kd是PS
        1, 4, 6, 2, 4; % 如果e是NB且ec是ZO，则delta_kp是PM，delta_ki是NM，delta_kd是ZO
        1, 5, 5, 2, 3; % 如果e是NB且ec是PS，则delta_kp是PS，delta_ki是NM，delta_kd是NS
        1, 6, 5, 3, 2; % 如果e是NB且ec是PM，则delta_kp是PS，delta_ki是NS，delta_kd是NM
        1, 7, 4, 3, 1; % 如果e是NB且ec是PB，则delta_kp是ZO，delta_ki是NS，delta_kd是NB
        
        2, 1, 7, 1, 7; % 如果e是NM且ec是NB，则delta_kp是PB，delta_ki是NB，delta_kd是PB
        2, 2, 7, 1, 6; % 如果e是NM且ec是NM，则delta_kp是PB，delta_ki是NB，delta_kd是PM
        2, 3, 6, 1, 6; % 如果e是NM且ec是NS，则delta_kp是PM，delta_ki是NB，delta_kd是PM
        2, 4, 6, 2, 5; % 如果e是NM且ec是ZO，则delta_kp是PM，delta_ki是NM，delta_kd是PS
        2, 5, 5, 2, 4; % 如果e是NM且ec是PS，则delta_kp是PS，delta_ki是NM，delta_kd是ZO
        2, 6, 4, 3, 3; % 如果e是NM且ec是PM，则delta_kp是ZO，delta_ki是NS，delta_kd是NS
        2, 7, 3, 3, 2; % 如果e是NM且ec是PB，则delta_kp是NS，delta_ki是NS，delta_kd是NM
        
        3, 1, 6, 1, 7; % 如果e是NS且ec是NB，则delta_kp是PM，delta_ki是NB，delta_kd是PB
        3, 2, 6, 1, 6; % 如果e是NS且ec是NM，则delta_kp是PM，delta_ki是NB，delta_kd是PM
        3, 3, 6, 2, 6; % 如果e是NS且ec是NS，则delta_kp是PM，delta_ki是NM，delta_kd是PM
        3, 4, 5, 2, 5; % 如果e是NS且ec是ZO，则delta_kp是PS，delta_ki是NM，delta_kd是PS
        3, 5, 4, 3, 4; % 如果e是NS且ec是PS，则delta_kp是ZO，delta_ki是NS，delta_kd是ZO
        3, 6, 3, 4, 3; % 如果e是NS且ec是PM，则delta_kp是NS，delta_ki是ZO，delta_kd是NS
        3, 7, 2, 4, 2; % 如果e是NS且ec是PB，则delta_kp是NM，delta_ki是ZO，delta_kd是NM
        
        4, 1, 6, 2, 6; % 如果e是ZO且ec是NB，则delta_kp是PM，delta_ki是NM，delta_kd是PM
        4, 2, 6, 2, 5; % 如果e是ZO且ec是NM，则delta_kp是PM，delta_ki是NM，delta_kd是PS
        4, 3, 5, 3, 5; % 如果e是ZO且ec是NS，则delta_kp是PS，delta_ki是NS，delta_kd是PS
        4, 4, 4, 4, 4; % 如果e是ZO且ec是ZO，则delta_kp是ZO，delta_ki是ZO，delta_kd是ZO
        4, 5, 3, 5, 3; % 如果e是ZO且ec是PS，则delta_kp是NS，delta_ki是PS，delta_kd是NS
        4, 6, 2, 6, 2; % 如果e是ZO且ec是PM，则delta_kp是NM，delta_ki是PM，delta_kd是NM
        4, 7, 2, 6, 1; % 如果e是ZO且ec是PB，则delta_kp是NM，delta_ki是PM，delta_kd是NB
        
        5, 1, 5, 2, 5; % 如果e是PS且ec是NB，则delta_kp是PS，delta_ki是NM，delta_kd是PS
        5, 2, 5, 3, 4; % 如果e是PS且ec是NM，则delta_kp是PS，delta_ki是NS，delta_kd是ZO
        5, 3, 4, 4, 4; % 如果e是PS且ec是NS，则delta_kp是ZO，delta_ki是ZO，delta_kd是ZO
        5, 4, 3, 5, 3; % 如果e是PS且ec是ZO，则delta_kp是NS，delta_ki是PS，delta_kd是NS
        5, 5, 2, 6, 3; % 如果e是PS且ec是PS，则delta_kp是NM，delta_ki是PM，delta_kd是NS
        5, 6, 2, 6, 2; % 如果e是PS且ec是PM，则delta_kp是NM，delta_ki是PM，delta_kd是NM
        5, 7, 1, 7, 1; % 如果e是PS且ec是PB，则delta_kp是NB，delta_ki是PB，delta_kd是NB
        
        6, 1, 5, 3, 4; % 如果e是PM且ec是NB，则delta_kp是PS，delta_ki是NS，delta_kd是ZO
        6, 2, 4, 4, 3; % 如果e是PM且ec是NM，则delta_kp是ZO，delta_ki是ZO，delta_kd是NS
        6, 3, 3, 5, 3; % 如果e是PM且ec是NS，则delta_kp是NS，delta_ki是PS，delta_kd是NS
        6, 4, 2, 6, 2; % 如果e是PM且ec是ZO，则delta_kp是NM，delta_ki是PM，delta_kd是NM
        6, 5, 2, 6, 1; % 如果e是PM且ec是PS，则delta_kp是NM，delta_ki是PM，delta_kd是NB
        6, 6, 1, 7, 1; % 如果e是PM且ec是PM，则delta_kp是NB，delta_ki是PB，delta_kd是NB
        6, 7, 1, 7, 1; % 如果e是PM且ec是PB，则delta_kp是NB，delta_ki是PB，delta_kd是NB
        
        7, 1, 4, 3, 3; % 如果e是PB且ec是NB，则delta_kp是ZO，delta_ki是NS，delta_kd是NS
        7, 2, 3, 4, 3; % 如果e是PB且ec是NM，则delta_kp是NS，delta_ki是ZO，delta_kd是NS
        7, 3, 2, 5, 2; % 如果e是PB且ec是NS，则delta_kp是NM，delta_ki是PS，delta_kd是NM
        7, 4, 2, 6, 1; % 如果e是PB且ec是ZO，则delta_kp是NM，delta_ki是PM，delta_kd是NB
        7, 5, 1, 6, 1; % 如果e是PB且ec是PS，则delta_kp是NB，delta_ki是PM，delta_kd是NB
        7, 6, 1, 7, 1; % 如果e是PB且ec是PM，则delta_kp是NB，delta_ki是PB，delta_kd是NB
        7, 7, 1, 7, 1; % 如果e是PB且ec是PB，则delta_kp是NB，delta_ki是PB，delta_kd是NB
    ];
    
    % 模糊推理过程
    % 初始化输出变量的模糊隶属度
    mu_delta_kp = zeros(7, 1);
    mu_delta_ki = zeros(7, 1);
    mu_delta_kd = zeros(7, 1);
    
    % 遍历49条规则
    for i = 1:49
        e_rule = rules(i, 1);
        ec_rule = rules(i, 2);
        delta_kp_rule = rules(i, 3);
        delta_ki_rule = rules(i, 4);
        delta_kd_rule = rules(i, 5);
        
        % 计算规则触发强度（取小运算）
        rule_strength = min(mu_e(e_rule), mu_ec(ec_rule));
        
        % 累积输出变量的隶属度（取大运算）
        mu_delta_kp(delta_kp_rule) = max(mu_delta_kp(delta_kp_rule), rule_strength);
        mu_delta_ki(delta_ki_rule) = max(mu_delta_ki(delta_ki_rule), rule_strength);
        mu_delta_kd(delta_kd_rule) = max(mu_delta_kd(delta_kd_rule), rule_strength);
    end
    
    % 去模糊化（重心法）
    delta_kp = defuzzification(delta_centers, mu_delta_kp);
    delta_ki = defuzzification(delta_centers, mu_delta_ki);
    delta_kd = defuzzification(delta_centers, mu_delta_kd);
end