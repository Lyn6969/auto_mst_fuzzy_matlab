%% motion_saliency_time_factor 参数敏感性分析脚本
% 
% 目的：专门分析 motion_saliency_time_factor 参数对运动显著性 cj 值的定量影响
% 
% 分析内容：
% 1. 测试 motion_saliency_time_factor 在 [2,3,4,5,6,7,8,9,10] 范围内的影响
% 2. 固定 G.weight_cj = 300，分析不同时间因子下的 cj 值分布
% 3. 计算激活率变化和参数敏感性
% 4. 生成可视化图表和数值对比表格
% 5. 提供参数调优建议
%
% 作者：LYN
% 日期：2025-08-05

function analyze_motion_saliency_time_factor()
    clc;
    clear;
    close all;

    % 添加必要的路径
    addpath('src/swarm');
    addpath('src/algorithms');
    addpath('src/utils/positioning');
    addpath('src/utils');
    addpath('src/utils/geometry');
    addpath('src/utils/visualization');

    fprintf('=== motion_saliency_time_factor 参数敏感性分析开始 ===\n');
    
    %% 实验参数设置
    % 测试 motion_saliency_time_factor 的不同值
    time_factor_values = [2, 3, 4, 5, 6, 7, 8, 9, 10];
    num_factor_tests = length(time_factor_values);
    
    % 固定其他参数
    fixed_weight_cj = 300;  % 固定为300而不是默认的100
    fixed_cj_threshold = 35; % 使用合理的阈值
    
    % 仿真基本设置
    num_simulations = 5;        % 每个参数组合运行的仿真次数
    max_sim_steps = 800;        % 仿真步数
    robot_count = 50;           % 机器人数量
    
    fprintf('测试参数设置：\n');
    fprintf('- motion_saliency_time_factor 范围: [%d, %d]\n', min(time_factor_values), max(time_factor_values));
    fprintf('- 固定 G.weight_cj = %d\n', fixed_weight_cj);
    fprintf('- 固定 G.cj_threshold = %d\n', fixed_cj_threshold);
    fprintf('- 仿真次数: %d\n', num_simulations);
    fprintf('- 机器人数量: %d\n', robot_count);
    fprintf('- 仿真步数: %d\n', max_sim_steps);
    
    %% 数据收集
    fprintf('\n=== 开始数据收集 ===\n');
    
    % 预分配结果存储
    all_sim_data = cell(num_factor_tests, num_simulations);
    
    % 进度跟踪
    total_simulations = num_factor_tests * num_simulations;
    current_sim = 0;
    
    for factor_idx = 1:num_factor_tests
        current_time_factor = time_factor_values(factor_idx);
        
        fprintf('--- 测试 motion_saliency_time_factor = %d (%d/%d) ---\n', ...
            current_time_factor, factor_idx, num_factor_tests);
        
        for sim_idx = 1:num_simulations
            current_sim = current_sim + 1;
            fprintf('  仿真 %d/%d (总进度: %d/%d)\n', ...
                sim_idx, num_simulations, current_sim, total_simulations);
            
            % 运行单次仿真
            sim_data = run_single_simulation_with_time_factor(...
                current_time_factor, fixed_weight_cj, fixed_cj_threshold, ...
                max_sim_steps, robot_count);
            
            all_sim_data{factor_idx, sim_idx} = sim_data;
        end
    end
    
    fprintf('数据收集完成！\n');
    
    %% 统计分析
    fprintf('\n=== 开始统计分析 ===\n');
    
    analysis_results = compute_time_factor_statistics(all_sim_data, time_factor_values, ...
        num_simulations, fixed_weight_cj, fixed_cj_threshold);
    
    %% 生成分析图表
    fprintf('\n=== 生成分析图表 ===\n');
    generate_time_factor_plots(analysis_results);
    
    %% 输出数值对比表格
    fprintf('\n=== 数值对比表格 ===\n');
    print_comparison_table(analysis_results);
    
    %% 参数调优建议
    fprintf('\n=== 参数调优建议 ===\n');
    provide_optimization_recommendations(analysis_results);
    
    %% 保存结果
    timestamp = datetime('now','Format','yyyyMMdd_HHmmss');
    save_filename = sprintf('time_factor_analysis_results_%s.mat', timestamp);
    save(save_filename, 'analysis_results', 'time_factor_values', 'fixed_weight_cj', 'fixed_cj_threshold');
    fprintf('\n分析结果已保存到: %s\n', save_filename);
    
    fprintf('\n=== motion_saliency_time_factor 参数敏感性分析完成 ===\n');
end

function sim_data = run_single_simulation_with_time_factor(time_factor, weight_cj, cj_threshold, max_steps, robot_count)
    % 运行单次仿真并收集运动显著性数据（修改版本支持自定义时间因子）
    
    % 初始化仿真环境
    G = struct();
    G.simStep = 0;
    
    % 基本参数设置（使用CE场景参数）
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
    G.cj_threshold = cj_threshold;
    G.weight_cj = weight_cj;
    G.deac_threshold = 0.2;
    G.noise_mov = 0;
    G.max_neighbors = 7;
    
    % 追逃参数
    G.weight_esc = 1;
    G.hawkID = 1;
    G.attackStep = 1;
    G.R_dead = 120;
    G.v0_hawk = 24;
    
    % 统计变量
    G.maxcj = zeros(G.maxSimSteps,1);
    G.target_dist = zeros(G.maxSimSteps,1);  % 初始化target_dist字段
    G.warnIDs = cell(G.maxSimSteps,1);       % 初始化warnIDs字段
    G.warnNum = zeros(G.maxSimSteps,1);      % 初始化warnNum字段
    
    % 初始化机器人
    G.actor{G.hawkID}.pose = [2000,2000];
    G.actor{G.hawkID}.vel = [-1,-1];
    G = initSimRobots(G,[0,0],100);
    
    % 数据收集变量
    all_cj_values = [];
    total_activations = 0;
    step_count = 0;
    
    % 仿真循环
    for t = 1:G.maxSimSteps
        G.simStep = t;
        
        % 运行算法（使用修改版本支持自定义时间因子）
        [desTurnAngle, desSpeed, G] = algo_mst_ce_with_time_factor(G, time_factor);
        
        % 收集运动显著性数据（跳过前10步让系统稳定）
        if t > 10
            step_cj_values = collect_step_cj_values_with_time_factor(G, time_factor);
            all_cj_values = [all_cj_values; step_cj_values];
            
            % 统计激活次数
            step_activations = count_step_activations(G);
            total_activations = total_activations + step_activations;
            step_count = step_count + 1;
        end
        
        % 更新机器人状态
        G = parallelSimRobots_v2(G, desTurnAngle, desSpeed);
        
        % 简化的终止条件检查
        if G.simStep >= G.attackStep && G.simStep > 100  % 简单的步数限制
            % 计算捕食者到最近猎物的距离
            hawk_pos = G.actor{G.hawkID}.pose;
            min_dist = inf;
            for prey_id = 2:G.maxID
                if prey_id ~= G.hawkID
                    dist = norm(G.actor{prey_id}.pose - hawk_pos);
                    if dist < min_dist
                        min_dist = dist;
                    end
                end
            end
            G.target_dist(G.simStep) = min_dist;

            if min_dist <= G.R_dead
                break;
            end
        end
    end
    
    % 返回仿真数据
    sim_data = struct();
    sim_data.all_cj_values = all_cj_values;
    sim_data.total_activations = total_activations;
    sim_data.step_count = step_count;
    sim_data.time_factor = time_factor;
    sim_data.weight_cj = weight_cj;
    sim_data.cj_threshold = cj_threshold;
    sim_data.final_step = t;
end

function step_cj_values = collect_step_cj_values_with_time_factor(G, time_factor)
    % 收集当前步骤的所有cj值（使用指定的时间因子）
    step_cj_values = [];
    
    % 遍历所有猎物（排除捕食者）
    for i = 2:G.maxID  % 假设1是捕食者
        if G.actor{i}.id ~= G.hawkID
            [neighbors, ~, ~] = get_topology_neighbors(G, i);
            if ~isempty(neighbors)
                cj_values = get_candidate_neighbors_with_time_factor(i, neighbors, G, time_factor);
                if ~isempty(cj_values)
                    step_cj_values = [step_cj_values; cj_values];
                end
            end
        end
    end
end

function count = count_step_activations(G)
    % 统计当前步骤的激活个体数量
    count = 0;
    for i = 2:G.maxID  % 排除捕食者
        if G.actor{i}.id ~= G.hawkID && G.actor{i}.is_activated
            count = count + 1;
        end
    end
end

function cj = get_candidate_neighbors_with_time_factor(focal_agent_id, neighbors, G, motion_saliency_time_factor)
    % 修改版本的get_candidate_neighbors，支持自定义时间因子

    num_neis = numel(neighbors);
    cj = zeros(num_neis, 1);

    % 获取当前代理和邻居的位置
    focal_agent = G.actor{focal_agent_id};
    my_pos = focal_agent.pose;

    for j = 1:num_neis
        nei_pos = neighbors{j}.pose;
        current_diff = nei_pos - my_pos;
        current_diff = current_diff ./ vecnorm(current_diff, 2, 2);

        if ~isfield(G.actor{focal_agent_id}, 'memory')
            cj = zeros(num_neis, 1);
            return;
        end

        history_length = size(neighbors{j}.memory, 1);

        if history_length >= motion_saliency_time_factor
            my_past_pos = G.actor{focal_agent_id}.memory(end-(motion_saliency_time_factor-1), 1:2);
            nei_past_pos = neighbors{j}.memory(end-(motion_saliency_time_factor-1), 1:2);
        else
            my_past_pos = G.actor{focal_agent_id}.memory(end, 1:2);
            nei_past_pos = neighbors{j}.memory(end, 1:2);
        end

        past_diff = nei_past_pos - my_past_pos;
        norm_past_diff = vecnorm(past_diff, 2, 2);
        norm_past_diff(norm_past_diff == 0) = 1e-4;
        past_diff = past_diff ./ norm_past_diff;

        angle_cos = max((min(dot(past_diff,current_diff,2), 1)), -1);

        % 关键修改：使用传入的motion_saliency_time_factor
        cj(j) = (acos(angle_cos)/ (G.cycTime * motion_saliency_time_factor)) * G.weight_cj;
    end
end

function [desTurnAngle, desSpeed, G] = algo_mst_ce_with_time_factor(G, time_factor)
    % 修改版本的algo_mst_ce，支持自定义时间因子
    % 这里简化实现，主要是为了支持自定义时间因子的cj计算

    % 初始化输出
    desTurnAngle = zeros(G.maxID, 1);
    desSpeed = G.v0 * ones(G.maxID, 1);

    % 简化的算法实现，重点是cj计算
    for i = 2:G.maxID  % 排除捕食者
        if G.actor{i}.id ~= G.hawkID
            [neighbors, ~, neighbors_id_list] = get_topology_neighbors(G, i);

            if ~isempty(neighbors)
                % 使用自定义时间因子计算cj
                cj = get_candidate_neighbors_with_time_factor(i, neighbors, G, time_factor);

                if ~isempty(cj) && (max(cj) > G.cj_threshold)
                    G.actor{i}.is_activated = true;
                    G.maxcj(G.simStep) = max(cj);
                else
                    G.actor{i}.is_activated = false;
                end
            end
        end
    end

    % 简化的运动控制（保持基本的群体行为）
    for i = 1:G.maxID
        if G.actor{i}.id ~= G.hawkID
            % 基本的随机运动
            desTurnAngle(i) = (rand - 0.5) * 30; % 随机转向
            desSpeed(i) = G.v0;
        else
            % 捕食者行为
            desTurnAngle(i) = 0;
            desSpeed(i) = G.v0_hawk;
        end
    end
end

function analysis_results = compute_time_factor_statistics(all_sim_data, time_factor_values, num_simulations, weight_cj, cj_threshold)
    % 计算时间因子分析的统计结果

    num_factors = length(time_factor_values);

    % 初始化结果结构
    analysis_results = struct();
    analysis_results.time_factor_values = time_factor_values;
    analysis_results.weight_cj = weight_cj;
    analysis_results.cj_threshold = cj_threshold;
    analysis_results.num_simulations = num_simulations;

    % 预分配统计数组
    stats_matrix = zeros(num_factors, 8); % [min, max, mean, std, median, count, activation_rate, relative_change]

    for factor_idx = 1:num_factors
        current_factor = time_factor_values(factor_idx);

        % 收集所有仿真的cj值
        all_factor_cj_values = [];
        total_activations = 0;
        total_steps = 0;

        for sim_idx = 1:num_simulations
            sim_data = all_sim_data{factor_idx, sim_idx};
            if ~isempty(sim_data.all_cj_values)
                all_factor_cj_values = [all_factor_cj_values; sim_data.all_cj_values];
            end
            total_activations = total_activations + sim_data.total_activations;
            total_steps = total_steps + sim_data.step_count;
        end

        % 计算统计量
        if ~isempty(all_factor_cj_values)
            stats_matrix(factor_idx, 1) = min(all_factor_cj_values);      % min
            stats_matrix(factor_idx, 2) = max(all_factor_cj_values);      % max
            stats_matrix(factor_idx, 3) = mean(all_factor_cj_values);     % mean
            stats_matrix(factor_idx, 4) = std(all_factor_cj_values);      % std
            stats_matrix(factor_idx, 5) = median(all_factor_cj_values);   % median
            stats_matrix(factor_idx, 6) = length(all_factor_cj_values);   % count
        end

        % 计算激活率
        if total_steps > 0
            stats_matrix(factor_idx, 7) = total_activations / total_steps; % activation_rate
        end

        % 计算相对于默认值5的变化
        if current_factor == 5
            baseline_idx = factor_idx;
        end
    end

    % 计算相对变化（相对于默认值5）
    if exist('baseline_idx', 'var')
        baseline_mean = stats_matrix(baseline_idx, 3);
        for factor_idx = 1:num_factors
            if baseline_mean > 0
                stats_matrix(factor_idx, 8) = (stats_matrix(factor_idx, 3) - baseline_mean) / baseline_mean * 100;
            end
        end
    end

    analysis_results.stats_matrix = stats_matrix;
    analysis_results.column_names = {'min', 'max', 'mean', 'std', 'median', 'count', 'activation_rate', 'relative_change_pct'};
end

function generate_time_factor_plots(analysis_results)
    % 生成时间因子分析的可视化图表

    time_factor_values = analysis_results.time_factor_values;
    stats_matrix = analysis_results.stats_matrix;

    % 创建图表窗口
    figure('Position', [100, 100, 1200, 800]);

    % 子图1：cj值均值随时间因子的变化
    subplot(2, 3, 1);
    plot(time_factor_values, stats_matrix(:, 3), 'bo-', 'LineWidth', 2, 'MarkerSize', 8);
    xlabel('motion\_saliency\_time\_factor');
    ylabel('cj值均值');
    title('cj值均值 vs 时间因子');
    grid on;

    % 添加理论曲线（反比关系）
    hold on;
    theoretical_values = stats_matrix(time_factor_values == 5, 3) * 5 ./ time_factor_values;
    plot(time_factor_values, theoretical_values, 'r--', 'LineWidth', 1.5);
    legend('实际值', '理论值 (∝1/factor)', 'Location', 'best');

    % 子图2：激活率随时间因子的变化
    subplot(2, 3, 2);
    plot(time_factor_values, stats_matrix(:, 7) * 100, 'go-', 'LineWidth', 2, 'MarkerSize', 8);
    xlabel('motion\_saliency\_time\_factor');
    ylabel('激活率 (%)');
    title('激活率 vs 时间因子');
    grid on;

    % 子图3：相对变化百分比
    subplot(2, 3, 3);
    bar(time_factor_values, stats_matrix(:, 8));
    xlabel('motion\_saliency\_time\_factor');
    ylabel('相对变化 (%)');
    title('相对于默认值5的变化');
    grid on;

    % 添加零线
    hold on;
    plot([min(time_factor_values), max(time_factor_values)], [0, 0], 'r--', 'LineWidth', 1);

    % 子图4：cj值分布的箱线图（需要原始数据，这里用均值±标准差近似）
    subplot(2, 3, 4);
    errorbar(time_factor_values, stats_matrix(:, 3), stats_matrix(:, 4), 'mo-', 'LineWidth', 2, 'MarkerSize', 8);
    xlabel('motion\_saliency\_time\_factor');
    ylabel('cj值 (均值±标准差)');
    title('cj值分布');
    grid on;

    % 子图5：最大值和最小值范围
    subplot(2, 3, 5);
    plot(time_factor_values, stats_matrix(:, 2), 'r^-', 'LineWidth', 2, 'MarkerSize', 8);
    hold on;
    plot(time_factor_values, stats_matrix(:, 1), 'bv-', 'LineWidth', 2, 'MarkerSize', 8);
    xlabel('motion\_saliency\_time\_factor');
    ylabel('cj值');
    title('cj值范围');
    legend('最大值', '最小值', 'Location', 'best');
    grid on;

    % 子图6：数据点数量
    subplot(2, 3, 6);
    bar(time_factor_values, stats_matrix(:, 6));
    xlabel('motion\_saliency\_time\_factor');
    ylabel('数据点数量');
    title('收集的数据点数量');
    grid on;

    % 调整布局
    sgtitle(sprintf('motion\\_saliency\\_time\\_factor 参数敏感性分析 (weight\\_cj=%d)', ...
        analysis_results.weight_cj), 'FontSize', 14, 'FontWeight', 'bold');

    % 保存图表
    timestamp = datetime('now','Format','yyyyMMdd_HHmmss');
    saveas(gcf, sprintf('time_factor_analysis_%s.png', timestamp));
    saveas(gcf, sprintf('time_factor_analysis_%s.fig', timestamp));
end

function print_comparison_table(analysis_results)
    % 打印数值对比表格

    time_factor_values = analysis_results.time_factor_values;
    stats_matrix = analysis_results.stats_matrix;

    fprintf('\n========== motion_saliency_time_factor 参数影响对比表格 ==========\n');
    fprintf('固定参数: G.weight_cj = %d, G.cj_threshold = %d\n', ...
        analysis_results.weight_cj, analysis_results.cj_threshold);
    fprintf('================================================================\n');
    fprintf('时间因子 | cj均值    | cj标准差  | 激活率(%%) | 相对变化(%%) | 数据点数\n');
    fprintf('---------|----------|----------|----------|------------|--------\n');

    for i = 1:length(time_factor_values)
        fprintf('%8d | %8.3f | %8.3f | %8.2f | %10.1f | %8d\n', ...
            time_factor_values(i), ...
            stats_matrix(i, 3), ...  % mean
            stats_matrix(i, 4), ...  % std
            stats_matrix(i, 7) * 100, ... % activation_rate
            stats_matrix(i, 8), ...  % relative_change
            stats_matrix(i, 6));     % count
    end

    fprintf('================================================================\n');

    % 计算理论值对比
    fprintf('\n========== 理论值 vs 实际值对比 ==========\n');
    fprintf('时间因子 | 理论倍数 | 实际倍数 | 误差(%%)\n');
    fprintf('---------|----------|----------|--------\n');

    baseline_idx = find(time_factor_values == 5);
    if ~isempty(baseline_idx)
        baseline_mean = stats_matrix(baseline_idx, 3);
        for i = 1:length(time_factor_values)
            theoretical_ratio = 5 / time_factor_values(i);
            actual_ratio = stats_matrix(i, 3) / baseline_mean;
            error_pct = abs(actual_ratio - theoretical_ratio) / theoretical_ratio * 100;

            fprintf('%8d | %8.3f | %8.3f | %6.1f\n', ...
                time_factor_values(i), theoretical_ratio, actual_ratio, error_pct);
        end
    end
    fprintf('==========================================\n');
end

function provide_optimization_recommendations(analysis_results)
    % 提供参数调优建议

    time_factor_values = analysis_results.time_factor_values;
    stats_matrix = analysis_results.stats_matrix;

    fprintf('\n========== 参数调优建议 ==========\n');

    % 1. 分析激活率
    activation_rates = stats_matrix(:, 7);
    optimal_activation_range = [0.1, 0.3]; % 理想激活率范围10%-30%

    fprintf('1. 激活率分析:\n');
    fprintf('   - 当前激活率范围: %.1f%% - %.1f%%\n', min(activation_rates)*100, max(activation_rates)*100);
    fprintf('   - 建议激活率范围: %.0f%% - %.0f%%\n', optimal_activation_range(1)*100, optimal_activation_range(2)*100);

    % 找到激活率在理想范围内的时间因子
    optimal_indices = find(activation_rates >= optimal_activation_range(1) & ...
                          activation_rates <= optimal_activation_range(2));

    if ~isempty(optimal_indices)
        optimal_factors = time_factor_values(optimal_indices);
        fprintf('   - 激活率在理想范围内的时间因子: %s\n', mat2str(optimal_factors));
    else
        fprintf('   - 警告: 没有时间因子的激活率在理想范围内\n');
    end

    % 2. 分析cj值稳定性
    cv_values = stats_matrix(:, 4) ./ stats_matrix(:, 3); % 变异系数
    fprintf('\n2. cj值稳定性分析:\n');
    fprintf('   - 变异系数范围: %.3f - %.3f\n', min(cv_values), max(cv_values));

    [~, most_stable_idx] = min(cv_values);
    fprintf('   - 最稳定的时间因子: %d (变异系数=%.3f)\n', ...
        time_factor_values(most_stable_idx), cv_values(most_stable_idx));

    % 3. 分析敏感性
    relative_changes = abs(stats_matrix(:, 8));
    fprintf('\n3. 参数敏感性分析:\n');
    fprintf('   - 最大相对变化: %.1f%% (时间因子=%d)\n', ...
        max(relative_changes), time_factor_values(relative_changes == max(relative_changes)));

    % 4. 综合建议
    fprintf('\n4. 综合建议:\n');

    % 计算综合评分（激活率权重0.4，稳定性权重0.3，敏感性权重0.3）
    activation_score = zeros(size(activation_rates));
    for i = 1:length(activation_rates)
        if activation_rates(i) >= optimal_activation_range(1) && activation_rates(i) <= optimal_activation_range(2)
            activation_score(i) = 1 - abs(activation_rates(i) - mean(optimal_activation_range)) / (optimal_activation_range(2) - optimal_activation_range(1));
        else
            activation_score(i) = 0;
        end
    end

    stability_score = 1 - (cv_values - min(cv_values)) / (max(cv_values) - min(cv_values));
    sensitivity_score = 1 - (relative_changes - min(relative_changes)) / (max(relative_changes) - min(relative_changes));

    composite_score = 0.4 * activation_score + 0.3 * stability_score + 0.3 * sensitivity_score;

    [~, best_idx] = max(composite_score);
    recommended_factor = time_factor_values(best_idx);

    fprintf('   - 推荐的motion_saliency_time_factor值: %d\n', recommended_factor);
    fprintf('   - 综合评分: %.3f (满分1.0)\n', composite_score(best_idx));

    % 5. 应用场景建议
    fprintf('\n5. 应用场景建议:\n');

    % 快速响应场景
    high_sensitivity_indices = find(relative_changes > 30);
    if ~isempty(high_sensitivity_indices)
        fast_response_factors = time_factor_values(high_sensitivity_indices);
        fprintf('   - 快速响应场景 (高敏感性): 时间因子 %s\n', mat2str(fast_response_factors));
    end

    % 稳定环境场景
    low_cv_indices = find(cv_values < median(cv_values));
    if ~isempty(low_cv_indices)
        stable_factors = time_factor_values(low_cv_indices);
        fprintf('   - 稳定环境场景 (低变异): 时间因子 %s\n', mat2str(stable_factors));
    end

    % 平衡场景
    fprintf('   - 平衡场景 (推荐): 时间因子 %d\n', recommended_factor);

    % 6. 注意事项
    fprintf('\n6. 注意事项:\n');
    fprintf('   - 时间因子越小，系统对运动变化越敏感，但可能增加噪声影响\n');
    fprintf('   - 时间因子越大，系统越稳定，但可能降低响应速度\n');
    fprintf('   - 建议根据具体应用场景的要求选择合适的时间因子\n');
    fprintf('   - 当前分析基于固定的weight_cj=%d，实际应用中可能需要联合调优\n', analysis_results.weight_cj);

    fprintf('\n================================\n');
end
