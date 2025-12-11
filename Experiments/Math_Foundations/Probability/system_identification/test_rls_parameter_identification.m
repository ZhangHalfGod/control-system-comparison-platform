% 测试递归最小二乘（RLS）参数辨识
% 分析不同遗忘因子对辨识结果的影响

% 定义测试系统：一阶线性系统
% y(k) = a1*y(k-1) + b1*u(k-1) + v(k)
% 真实参数：a1=0.8, b1=0.5

% 生成测试数据
generate_test_data();

function generate_test_data()
    disp('=== 生成测试数据 ===');
    
    % 系统参数
    a1_true = 0.8;
    b1_true = 0.5;
    true_params = [a1_true, b1_true];
    
    % 模拟参数
    n_steps = 100;
    noise_var = 0.1;  % 观测噪声方差
    
    % 生成输入信号（随机信号）
    rng(456);  % 设置随机种子，确保结果可复现
    u = randn(n_steps, 1);  % 零均值、单位方差的高斯随机输入
    
    % 生成输出信号
    y = zeros(n_steps, 1);
    for k = 2:n_steps
        y(k) = a1_true * y(k-1) + b1_true * u(k-1) + sqrt(noise_var) * randn();
    end
    
    % 构建回归矩阵phi
    % phi(k, :) = [y(k-1), u(k-1)]
    phi = zeros(n_steps-1, 2);
    for k = 2:n_steps
        phi(k-1, :) = [y(k-1), u(k-1)];
    end
    
    % 输出序列（从第2步开始）
    y_obs = y(2:end);
    
    disp(['真实系统参数：a1=', num2str(a1_true), ', b1=', num2str(b1_true)]);
    disp(['观测噪声方差：', num2str(noise_var)]);
    disp(['数据长度：', num2str(n_steps)]);
    
    % 测试不同遗忘因子
    test_different_lambdas(phi, y_obs, true_params);
end

function test_different_lambdas(phi, y_obs, true_params)
    disp('\n=== 测试不同遗忘因子 ===');
    
    % 定义不同的遗忘因子
    lambda_values = [0.8, 0.9, 0.95, 0.99, 1.0];
    n_lambdas = length(lambda_values);
    
    % 存储不同遗忘因子的结果
    all_theta_est = cell(n_lambdas, 1);
    all_e = cell(n_lambdas, 1);
    final_errors = zeros(n_lambdas, 1);
    
    % 对每个遗忘因子运行RLS算法
    for i = 1:n_lambdas
        lambda = lambda_values(i);
        disp(['\n--- 测试遗忘因子 lambda = ', num2str(lambda), ' ---']);
        
        % 运行RLS参数辨识
        [theta_est, ~, e] = rls_parameter_identification(phi, y_obs, lambda);
        
        % 存储结果
        all_theta_est{i} = theta_est;
        all_e{i} = e;
        
        % 计算最终参数误差
        final_theta = theta_est(end, :)';
        final_errors(i) = norm(final_theta - true_params');
        
        disp(['最终参数估计：', num2str(final_theta')]);
        disp(['最终参数误差：', num2str(final_errors(i))]);
    end
    
    % 可视化结果
    visualize_results(lambda_values, all_theta_est, all_e, true_params, final_errors);
end

function visualize_results(lambda_values, all_theta_est, all_e, true_params, final_errors)
    n_lambdas = length(lambda_values);
    n_steps = size(all_theta_est{1}, 1);
    
    % 1. 绘制参数估计随时间变化的曲线
    figure('Name', '不同遗忘因子下的参数估计结果');
    
    % 绘制a1参数估计
    subplot(2, 1, 1);
    for i = 1:n_lambdas
        plot(1:n_steps, all_theta_est{i}(:, 1), 'LineWidth', 1.5, 'DisplayName', ['lambda=', num2str(lambda_values(i))]);
        hold on;
    end
    plot([1, n_steps], [true_params(1), true_params(1)], 'k--', 'LineWidth', 2, 'DisplayName', '真实值');
    xlabel('时间步 k');
    ylabel('参数a1估计');
    title('不同遗忘因子下的参数a1估计');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 绘制b1参数估计
    subplot(2, 1, 2);
    for i = 1:n_lambdas
        plot(1:n_steps, all_theta_est{i}(:, 2), 'LineWidth', 1.5, 'DisplayName', ['lambda=', num2str(lambda_values(i))]);
        hold on;
    end
    plot([1, n_steps], [true_params(2), true_params(2)], 'k--', 'LineWidth', 2, 'DisplayName', '真实值');
    xlabel('时间步 k');
    ylabel('参数b1估计');
    title('不同遗忘因子下的参数b1估计');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 2. 绘制估计误差曲线
    figure('Name', '不同遗忘因子下的估计误差');
    for i = 1:n_lambdas
        plot(1:n_steps, all_e{i}, 'LineWidth', 1.5, 'DisplayName', ['lambda=', num2str(lambda_values(i))]);
        hold on;
    end
    xlabel('时间步 k');
    ylabel('估计误差 e(k)');
    title('不同遗忘因子下的估计误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 3. 绘制最终参数误差与遗忘因子的关系
    figure('Name', '最终参数误差与遗忘因子的关系');
    plot(lambda_values, final_errors, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
    xlabel('遗忘因子 lambda');
    ylabel('最终参数误差（2-范数）');
    title('最终参数误差与遗忘因子的关系');
    grid on;
    
    % 4. 绘制最终参数估计值
    figure('Name', '不同遗忘因子下的最终参数估计');
    bar_width = 0.15;
    x = 1:2;
    
    for i = 1:n_lambdas
        final_theta = all_theta_est{i}(end, :);
        bar(x + (i-1)*bar_width, final_theta, bar_width, 'DisplayName', ['lambda=', num2str(lambda_values(i))]);
        hold on;
    end
    
    % 绘制真实参数
    plot(x + (n_lambdas/2-0.5)*bar_width, true_params, 'k--', 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 10, 'DisplayName', '真实值');
    
    xlabel('参数');
    ylabel('参数估计值');
    title('不同遗忘因子下的最终参数估计');
    set(gca, 'XTick', x + (n_lambdas/2-0.5)*bar_width);
    set(gca, 'XTickLabel', {'a1', 'b1'});
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 5. 绘制参数估计的收敛过程（误差平方和）
    figure('Name', '参数估计的收敛过程');
    for i = 1:n_lambdas
        theta_est = all_theta_est{i};
        err_sum = zeros(size(theta_est, 1), 1);
        for k = 1:size(theta_est, 1)
            err_sum(k) = norm(theta_est(k, :) - true_params, 2);
        end
        semilogy(1:size(theta_est, 1), err_sum, 'LineWidth', 1.5, 'DisplayName', ['lambda=', num2str(lambda_values(i))]);
        hold on;
    end
    xlabel('时间步 k');
    ylabel('参数估计误差（2-范数，对数坐标）');
    title('参数估计的收敛过程');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 输出分析结果
    disp('\n=== 不同遗忘因子性能分析 ===');
    disp('遗忘因子 | 最终a1估计 | 最终b1估计 | 最终参数误差');
    disp('--------------------------------------------');
    for i = 1:n_lambdas
        final_theta = all_theta_est{i}(end, :);
        disp(sprintf('  %.2f    |   %.4f    |   %.4f    |   %.6f', ...
            lambda_values(i), final_theta(1), final_theta(2), final_errors(i)));
    end
    
    % 找出最优遗忘因子
    [min_err, min_idx] = min(final_errors);
    disp(sprintf('\n最优遗忘因子：lambda=%.2f，最小参数误差=%.6f', lambda_values(min_idx), min_err));
    
    % 分析不同遗忘因子的特点
    disp('\n=== 遗忘因子对辨识结果的影响分析 ===');
    disp('1. 较小的遗忘因子（lambda<0.95）：');
    disp('   - 跟踪能力强，对新数据反应快');
    disp('   - 估计波动较大，收敛后仍有波动');
    disp('   - 适合时变系统');
    
    disp('2. 较大的遗忘因子（lambda>0.95）：');
    disp('   - 平滑性好，估计波动小');
    disp('   - 对新数据反应慢，收敛速度较慢');
    disp('   - 适合定常系统');
    
    disp('3. 遗忘因子lambda=1.0：');
    disp('   - 标准最小二乘算法');
    disp('   - 所有数据权重相同');
    disp('   - 收敛最慢，但最终精度最高（当系统定常时）');
end