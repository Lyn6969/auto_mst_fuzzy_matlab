%% 运动显著性参数分析 - 快速测试版本
% 
% 这是一个简化版本，用于快速验证分析脚本的功能
% 使用较少的参数点和较短的仿真时间
%
% 运行时间：约1-2分钟

clc;
clear;
close all;

fprintf('=== 运动显著性参数分析 - 快速测试 ===\n');

%% 简化的实验参数设置
% 仅测试3个G.weight_cj值进行快速验证
weight_cj_values = [50, 100, 150];
num_simulations = 2;        % 减少仿真次数
max_sim_steps = 100;        % 减少仿真步数
robot_count = 20;           % 减少机器人数量

fprintf('快速测试参数:\n');
fprintf('- G.weight_cj值: %s\n', mat2str(weight_cj_values));
fprintf('- 仿真次数: %d\n', num_simulations);
fprintf('- 仿真步数: %d\n', max_sim_steps);
fprintf('- 机器人数量: %d\n', robot_count);

%% 创建结果存储结构
test_results = struct();
test_results.weight_cj_values = weight_cj_values;

%% 运行快速测试
fprintf('\n开始快速测试...\n');

for i = 1:length(weight_cj_values)
    current_weight_cj = weight_cj_values(i);
    fprintf('\n--- 测试 G.weight_cj = %d ---\n', current_weight_cj);
    
    all_cj_values = [];
    total_activations = 0;
    
    for sim_idx = 1:num_simulations
        fprintf('运行仿真 %d/%d...\n', sim_idx, num_simulations);
        
        % 运行单次仿真
        [sim_cj_values, sim_activations] = run_quick_simulation(current_weight_cj, max_sim_steps, robot_count);
        
        all_cj_values = [all_cj_values; sim_cj_values];
        total_activations = total_activations + sim_activations;
    end
    
    % 计算统计量
    if ~isempty(all_cj_values)
        cj_min = min(all_cj_values);
        cj_max = max(all_cj_values);
        cj_mean = mean(all_cj_values);
        cj_std = std(all_cj_values);
        
        fprintf('结果: cj值范围[%.3f, %.3f], 均值%.3f±%.3f, 激活次数%d\n', ...
            cj_min, cj_max, cj_mean, cj_std, total_activations);
        
        % 存储结果
        test_results.(['weight_' num2str(current_weight_cj)]).cj_values = all_cj_values;
        test_results.(['weight_' num2str(current_weight_cj)]).stats = [cj_min, cj_max, cj_mean, cj_std];
        test_results.(['weight_' num2str(current_weight_cj)]).activations = total_activations;
    else
        fprintf('警告: 未收集到有效的cj值数据\n');
    end
end

%% 生成简化的分析图表
fprintf('\n生成测试图表...\n');
generate_quick_plots(test_results);

%% 验证线性关系
fprintf('\n=== 线性关系验证 ===\n');
verify_linear_relationship(test_results);

fprintf('\n=== 快速测试完成 ===\n');
fprintf('如果结果符合预期，可以运行完整版本：analyze_motion_saliency_parameters\n');

%% 辅助函数

function [all_cj_values, total_activations] = run_quick_simulation(weight_cj, max_steps, robot_count)
    % 运行快速仿真
    
    % 初始化仿真环境
    G = struct();
    G.simStep = 0;
    G.maxID = robot_count;
    G.maxSimSteps = max_steps;
    G.cycTime = 0.2;
    G.v0 = 12;
    G.maxRotRate = 12*1.91;
    G.r_sense = 1000;
    G.R_escape = 300 + zeros(G.maxID,1);
    G.R_escape(2,1) = 1000;
    
    % 协同参数
    G.weight_align = 10;
    G.Drep = 300;
    G.Dsen = 1000;
    G.weight_rep = 8;
    G.weight_att = 0.01;
    G.cj_threshold = 0;  % 无FLC机制
    G.weight_cj = weight_cj;
    G.deac_threshold = 0.2;
    G.noise_mov = 0;
    G.max_neighbors = 7;
    
    % 追逃参数
    G.weight_esc = 1;
    G.hawkID = 1;
    G.attackStep = 30;
    G.R_dead = 120;
    G.v0_hawk = 24;
    G.maxcj = zeros(G.maxSimSteps,1);
    
    % 初始化机器人
    G.actor{G.hawkID}.pose = [2000,2000];
    G.actor{G.hawkID}.vel = [-1,-1];
    G = initSimRobots(G,[0,0],100);
    
    % 数据收集
    all_cj_values = [];
    total_activations = 0;
    
    try
        % 仿真循环
        for t = 1:G.maxSimSteps
            G.simStep = t;
            
            % 运行算法
            [desTurnAngle, desSpeed, G] = algo_mst_ce(G);
            
            % 收集数据（跳过前5步）
            if t > 5
                % 收集cj值
                preyList = setdiff(G.robotsList, G.hawkID);
                for i = preyList
                    [neighbors, ~, ~] = get_topology_neighbors(G, G.actor{i}.id);
                    if ~isempty(neighbors)
                        cj_values = get_candidate_neighbors(G.actor{i}.id, neighbors, G);
                        if ~isempty(cj_values)
                            all_cj_values = [all_cj_values; cj_values];
                        end
                    end
                    
                    % 统计激活
                    if G.actor{i}.is_activated
                        total_activations = total_activations + 1;
                    end
                end
            end
            
            % 更新状态
            G = parallelSimRobots_v2(G, desTurnAngle, desSpeed);
            
            % 检查终止条件
            if G.simStep >= G.attackStep && G.target_dist(end) <= G.R_dead
                break;
            end
        end
    catch ME
        fprintf('仿真出错: %s\n', ME.message);
    end
end

function generate_quick_plots(test_results)
    % 生成快速测试图表
    
    weight_cj_values = test_results.weight_cj_values;
    
    % 提取数据
    means = [];
    stds = [];
    mins = [];
    maxs = [];
    
    for i = 1:length(weight_cj_values)
        weight_key = ['weight_' num2str(weight_cj_values(i))];
        if isfield(test_results, weight_key)
            stats = test_results.(weight_key).stats;
            mins = [mins, stats(1)];
            maxs = [maxs, stats(2)];
            means = [means, stats(3)];
            stds = [stds, stats(4)];
        end
    end
    
    % 创建图表
    figure('Position', [200, 200, 1200, 400], 'Name', 'Quick Test Results');
    
    % 子图1：cj值统计
    subplot(1, 3, 1);
    errorbar(weight_cj_values, means, stds, 'bo-', 'LineWidth', 2, 'MarkerSize', 8);
    xlabel('G.weight\_cj');
    ylabel('cj值均值');
    title('cj值均值 vs G.weight\_cj');
    grid on;
    
    % 子图2：cj值范围
    subplot(1, 3, 2);
    plot(weight_cj_values, mins, 'ro-', 'LineWidth', 2, 'DisplayName', '最小值');
    hold on;
    plot(weight_cj_values, maxs, 'go-', 'LineWidth', 2, 'DisplayName', '最大值');
    xlabel('G.weight\_cj');
    ylabel('cj值');
    title('cj值范围 vs G.weight\_cj');
    legend();
    grid on;
    
    % 子图3：线性拟合
    subplot(1, 3, 3);
    scatter(weight_cj_values, means, 100, 'filled');
    hold on;
    
    if length(weight_cj_values) >= 2
        p = polyfit(weight_cj_values, means, 1);
        fitted_line = polyval(p, weight_cj_values);
        plot(weight_cj_values, fitted_line, 'r-', 'LineWidth', 2);
        
        % 计算R²
        if var(means) > 0
            R_squared = 1 - sum((means - fitted_line).^2) / sum((means - mean(means)).^2);
        else
            R_squared = NaN;
        end
        
        title(sprintf('线性拟合 (R² = %.4f)', R_squared));
        text(0.05, 0.95, sprintf('y = %.4fx + %.4f', p(1), p(2)), ...
            'Units', 'normalized', 'FontSize', 10, 'BackgroundColor', 'white');
    else
        title('线性拟合（数据点不足）');
    end
    
    xlabel('G.weight\_cj');
    ylabel('cj值均值');
    grid on;
    
    sgtitle('快速测试结果', 'FontSize', 14, 'FontWeight', 'bold');
end

function verify_linear_relationship(test_results)
    % 验证线性关系
    
    weight_cj_values = test_results.weight_cj_values;
    means = [];
    
    for i = 1:length(weight_cj_values)
        weight_key = ['weight_' num2str(weight_cj_values(i))];
        if isfield(test_results, weight_key)
            stats = test_results.(weight_key).stats;
            means = [means, stats(3)];
        end
    end
    
    if length(means) >= 2
        % 计算线性相关系数
        correlation = corrcoef(weight_cj_values, means);
        r_value = correlation(1,2);
        
        % 计算理论比值
        if length(means) >= 2
            theoretical_ratio = means(2) / weight_cj_values(2) * weight_cj_values(1);
            actual_ratio = means(1);
            ratio_error = abs(theoretical_ratio - actual_ratio) / actual_ratio * 100;
            
            fprintf('线性关系验证：\n');
            fprintf('- 相关系数 r = %.4f\n', r_value);
            fprintf('- 理论值 vs 实际值比较：\n');
            fprintf('  weight_cj=%d: 理论=%.3f, 实际=%.3f, 误差=%.1f%%\n', ...
                weight_cj_values(1), theoretical_ratio, actual_ratio, ratio_error);
            
            if abs(r_value) > 0.95
                fprintf('✓ 线性关系良好\n');
            else
                fprintf('⚠ 线性关系需要改进\n');
            end
        end
    else
        fprintf('数据点不足，无法验证线性关系\n');
    end
end