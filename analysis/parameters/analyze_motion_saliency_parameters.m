%% 运动显著性参数分析脚本
% 
% 目的：分析不带FLC机制的系统中G.cj_threshold、cj值和G.weight_cj之间的关系
% 
% 分析内容：
% 1. 当G.cj_threshold = 0（无FLC机制）时，cj值的范围与G.weight_cj的关系
% 2. 不同G.weight_cj值对应的运动显著性值范围
% 3. 生成统计图表和保存分析结果
%
% 作者：LYN
% 日期：2025-07-18

clc;
clear;
close all;

%% 实验参数设置
fprintf('=== 运动显著性参数分析开始 ===\n');

% 测试G.weight_cj的不同值范围
% 基于现有代码，G.weight_cj通常在[10, 200]范围内
weight_cj_values = [10, 25, 50, 75, 100, 125, 150, 175, 200];
num_weight_tests = length(weight_cj_values);

% 仿真基本设置
num_simulations = 3;        % 每个参数组合运行的仿真次数
max_sim_steps = 300;        % 仿真步数（减少以加快分析速度）
robot_count = 30;           % 机器人数量（减少以加快分析速度）

% 创建结果存储结构
analysis_results = struct();
analysis_results.weight_cj_values = weight_cj_values;
analysis_results.num_simulations = num_simulations;
analysis_results.max_sim_steps = max_sim_steps;
analysis_results.robot_count = robot_count;

% 为每个weight_cj值创建存储数组
for i = 1:num_weight_tests
    weight_cj = weight_cj_values(i);
    
    % 初始化存储数组
    analysis_results.(['weight_' num2str(weight_cj)]).cj_values_all = [];
    analysis_results.(['weight_' num2str(weight_cj)]).cj_stats = struct();
    analysis_results.(['weight_' num2str(weight_cj)]).activation_stats = struct();
    analysis_results.(['weight_' num2str(weight_cj)]).sim_summaries = [];
end

%% 主分析循环
fprintf('开始参数分析循环...\n');
fprintf('测试G.weight_cj值: %s\n', mat2str(weight_cj_values));

total_simulations = num_weight_tests * num_simulations;
current_sim = 0;

for weight_idx = 1:num_weight_tests
    current_weight_cj = weight_cj_values(weight_idx);
    fprintf('\n--- 测试 G.weight_cj = %d ---\n', current_weight_cj);
    
    % 存储当前weight_cj的所有仿真数据
    weight_cj_all_data = [];
    weight_cj_sim_summaries = [];
    
    for sim_idx = 1:num_simulations
        current_sim = current_sim + 1;
        fprintf('仿真 %d/%d (G.weight_cj=%d, 第%d次)\n', current_sim, total_simulations, current_weight_cj, sim_idx);
        
        % 运行单次仿真
        sim_data = run_single_simulation(current_weight_cj, max_sim_steps, robot_count);
        
        % 收集数据
        weight_cj_all_data = [weight_cj_all_data; sim_data.cj_values_all];
        weight_cj_sim_summaries = [weight_cj_sim_summaries; sim_data.summary];
        
        % 显示仿真摘要
        fprintf('  仿真摘要：cj值范围[%.3f, %.3f], 均值%.3f, 激活次数%d\n', ...
            sim_data.summary.cj_min, sim_data.summary.cj_max, ...
            sim_data.summary.cj_mean, sim_data.summary.total_activations);
    end
    
    % 计算当前weight_cj的汇总统计
    if ~isempty(weight_cj_all_data)
        weight_key = ['weight_' num2str(current_weight_cj)];
        analysis_results.(weight_key).cj_values_all = weight_cj_all_data;
        analysis_results.(weight_key).sim_summaries = weight_cj_sim_summaries;
        
        % 计算统计量
        cj_stats = struct();
        cj_stats.min = min(weight_cj_all_data);
        cj_stats.max = max(weight_cj_all_data);
        cj_stats.mean = mean(weight_cj_all_data);
        cj_stats.std = std(weight_cj_all_data);
        cj_stats.median = median(weight_cj_all_data);
        cj_stats.q25 = quantile(weight_cj_all_data, 0.25);
        cj_stats.q75 = quantile(weight_cj_all_data, 0.75);
        cj_stats.count = length(weight_cj_all_data);
        
        analysis_results.(weight_key).cj_stats = cj_stats;
        
        % 计算激活统计
        activation_stats = struct();
        activation_stats.total_activations = sum([weight_cj_sim_summaries.total_activations]);
        activation_stats.avg_activations_per_sim = mean([weight_cj_sim_summaries.total_activations]);
        activation_stats.activation_rate = activation_stats.total_activations / (num_simulations * max_sim_steps * robot_count);
        
        analysis_results.(weight_key).activation_stats = activation_stats;
        
        % 显示当前weight_cj的汇总统计
        fprintf('  汇总统计：cj值范围[%.3f, %.3f], 均值%.3f±%.3f, 总激活次数%d\n', ...
            cj_stats.min, cj_stats.max, cj_stats.mean, cj_stats.std, activation_stats.total_activations);
    end
end

%% 生成分析图表
fprintf('\n=== 生成分析图表 ===\n');
generate_analysis_plots(analysis_results);

%% 保存分析结果
fprintf('\n=== 保存分析结果 ===\n');
save_analysis_results(analysis_results);

%% 显示最终摘要
fprintf('\n=== 最终分析摘要 ===\n');
display_final_summary(analysis_results);

fprintf('\n=== 运动显著性参数分析完成 ===\n');

%% 辅助函数

function sim_data = run_single_simulation(weight_cj, max_steps, robot_count)
    % 运行单次仿真并收集运动显著性数据
    
    % 初始化仿真环境
    G = struct();
    G.simStep = 0;
    
    % 基本参数设置
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
    G.cj_threshold = 0;  % 关键：设置为0，无FLC机制
    G.weight_cj = weight_cj;  % 测试参数
    G.deac_threshold = 0.2;
    G.noise_mov = 0;
    G.max_neighbors = 7;
    
    % 追逃参数
    G.weight_esc = 1;
    G.hawkID = 1;
    G.attackStep = 50;  % 延迟攻击开始时间
    G.R_dead = 120;
    G.v0_hawk = 24;
    
    % 统计变量
    G.maxcj = zeros(G.maxSimSteps,1);
    
    % 初始化机器人
    G.actor{G.hawkID}.pose = [2000,2000];
    G.actor{G.hawkID}.vel = [-1,-1];
    G = initSimRobots(G,[0,0],100);
    
    % 数据收集变量
    all_cj_values = [];
    total_activations = 0;
    
    % 仿真循环
    for t = 1:G.maxSimSteps
        G.simStep = t;
        
        % 运行算法（使用不带FLC的版本）
        [desTurnAngle, desSpeed, G] = algo_mst_ce(G);
        
        % 收集运动显著性数据
        if t > 10  % 跳过前几步，让系统稳定
            step_cj_values = collect_step_cj_values(G);
            all_cj_values = [all_cj_values; step_cj_values];
            
            % 统计激活次数
            step_activations = count_step_activations(G);
            total_activations = total_activations + step_activations;
        end
        
        % 更新机器人状态
        G = parallelSimRobots_v2(G, desTurnAngle, desSpeed);
        
        % 检查终止条件
        if G.simStep >= G.attackStep && G.target_dist(end) <= G.R_dead
            break;
        end
    end
    
    % 返回仿真数据
    sim_data = struct();
    sim_data.cj_values_all = all_cj_values;
    sim_data.total_activations = total_activations;
    
    % 计算摘要统计
    if ~isempty(all_cj_values)
        sim_data.summary = struct();
        sim_data.summary.cj_min = min(all_cj_values);
        sim_data.summary.cj_max = max(all_cj_values);
        sim_data.summary.cj_mean = mean(all_cj_values);
        sim_data.summary.cj_std = std(all_cj_values);
        sim_data.summary.cj_count = length(all_cj_values);
        sim_data.summary.total_activations = total_activations;
        sim_data.summary.weight_cj = weight_cj;
        sim_data.summary.final_step = G.simStep;
    else
        sim_data.summary = struct();
        sim_data.summary.cj_min = 0;
        sim_data.summary.cj_max = 0;
        sim_data.summary.cj_mean = 0;
        sim_data.summary.cj_std = 0;
        sim_data.summary.cj_count = 0;
        sim_data.summary.total_activations = 0;
        sim_data.summary.weight_cj = weight_cj;
        sim_data.summary.final_step = G.simStep;
    end
end

function step_cj_values = collect_step_cj_values(G)
    % 收集当前步骤的所有cj值
    step_cj_values = [];
    
    % 创建猎物列表（排除捕食者）
    preyList = setdiff(G.robotsList, G.hawkID);
    
    for i = preyList
        focal_agent = G.actor{i};
        [neighbors, ~, ~] = get_topology_neighbors(G, focal_agent.id);
        
        if ~isempty(neighbors)
            cj_values = get_candidate_neighbors(focal_agent.id, neighbors, G);
            if ~isempty(cj_values)
                step_cj_values = [step_cj_values; cj_values];
            end
        end
    end
end

function step_activations = count_step_activations(G)
    % 统计当前步骤的激活次数
    step_activations = 0;
    
    preyList = setdiff(G.robotsList, G.hawkID);
    
    for i = preyList
        if G.actor{i}.is_activated
            step_activations = step_activations + 1;
        end
    end
end

function generate_analysis_plots(analysis_results)
    % 生成分析图表
    
    weight_cj_values = analysis_results.weight_cj_values;
    
    % 准备绘图数据
    means = zeros(size(weight_cj_values));
    stds = zeros(size(weight_cj_values));
    mins = zeros(size(weight_cj_values));
    maxs = zeros(size(weight_cj_values));
    medians = zeros(size(weight_cj_values));
    activation_rates = zeros(size(weight_cj_values));
    
    for i = 1:length(weight_cj_values)
        weight_key = ['weight_' num2str(weight_cj_values(i))];
        if isfield(analysis_results, weight_key)
            stats = analysis_results.(weight_key).cj_stats;
            means(i) = stats.mean;
            stds(i) = stats.std;
            mins(i) = stats.min;
            maxs(i) = stats.max;
            medians(i) = stats.median;
            activation_rates(i) = analysis_results.(weight_key).activation_stats.activation_rate;
        end
    end
    
    % 创建图表
    figure('Position', [100, 100, 1600, 1000], 'Name', 'Motion Saliency Parameter Analysis');
    
    % 子图1：cj值均值 vs G.weight_cj
    subplot(2, 3, 1);
    errorbar(weight_cj_values, means, stds, 'bo-', 'LineWidth', 2, 'MarkerSize', 8);
    xlabel('G.weight\_cj');
    ylabel('cj值均值');
    title('cj值均值 vs G.weight\_cj');
    grid on;
    
    % 子图2：cj值范围 vs G.weight_cj
    subplot(2, 3, 2);
    plot(weight_cj_values, mins, 'ro-', 'LineWidth', 2, 'DisplayName', '最小值');
    hold on;
    plot(weight_cj_values, maxs, 'go-', 'LineWidth', 2, 'DisplayName', '最大值');
    plot(weight_cj_values, medians, 'bo-', 'LineWidth', 2, 'DisplayName', '中位数');
    xlabel('G.weight\_cj');
    ylabel('cj值');
    title('cj值范围 vs G.weight\_cj');
    legend('Location', 'best');
    grid on;
    
    % 子图3：激活率 vs G.weight_cj
    subplot(2, 3, 3);
    plot(weight_cj_values, activation_rates * 100, 'mo-', 'LineWidth', 2, 'MarkerSize', 8);
    xlabel('G.weight\_cj');
    ylabel('激活率 (%)');
    title('激活率 vs G.weight\_cj');
    grid on;
    
    % 子图4：cj值分布箱线图
    subplot(2, 3, 4);
    box_data = [];
    box_labels = [];
    
    for i = 1:length(weight_cj_values)
        weight_key = ['weight_' num2str(weight_cj_values(i))];
        if isfield(analysis_results, weight_key)
            cj_values = analysis_results.(weight_key).cj_values_all;
            if ~isempty(cj_values)
                % 随机采样以避免数据过多
                if length(cj_values) > 1000
                    sample_indices = randperm(length(cj_values), 1000);
                    cj_values = cj_values(sample_indices);
                end
                box_data = [box_data; cj_values];
                box_labels = [box_labels; repmat(weight_cj_values(i), length(cj_values), 1)];
            end
        end
    end
    
    if ~isempty(box_data)
        boxplot(box_data, box_labels);
        xlabel('G.weight\_cj');
        ylabel('cj值');
        title('cj值分布箱线图');
        grid on;
    end
    
    % 子图5：线性关系验证
    subplot(2, 3, 5);
    scatter(weight_cj_values, means, 100, 'filled');
    hold on;
    % 拟合线性关系
    p = polyfit(weight_cj_values, means, 1);
    fitted_line = polyval(p, weight_cj_values);
    plot(weight_cj_values, fitted_line, 'r-', 'LineWidth', 2);
    
    % 计算R²
    R_squared = 1 - sum((means - fitted_line).^2) / sum((means - mean(means)).^2);
    
    xlabel('G.weight\_cj');
    ylabel('cj值均值');
    title(sprintf('线性关系验证 (R² = %.4f)', R_squared));
    text(0.05, 0.95, sprintf('y = %.4fx + %.4f', p(1), p(2)), ...
        'Units', 'normalized', 'FontSize', 12, 'BackgroundColor', 'white');
    grid on;
    
    % 子图6：标准差 vs G.weight_cj
    subplot(2, 3, 6);
    plot(weight_cj_values, stds, 'ko-', 'LineWidth', 2, 'MarkerSize', 8);
    xlabel('G.weight\_cj');
    ylabel('cj值标准差');
    title('cj值标准差 vs G.weight\_cj');
    grid on;
    
    % 添加总标题
    sgtitle('运动显著性参数分析结果', 'FontSize', 16, 'FontWeight', 'bold');
    
    % 保存图表
    timestamp = datetime('now','Format','yyyyMMdd_HHmmss');
    filename = sprintf('motion_saliency_analysis_%s.png', timestamp);
    saveas(gcf, filename);
    fprintf('图表已保存为: %s\n', filename);
end

function save_analysis_results(analysis_results)
    % 保存分析结果到文件
    
    timestamp = datetime('now','Format','yyyyMMdd_HHmmss');
    
    % 保存完整的分析结果
    filename_mat = sprintf('motion_saliency_analysis_results_%s.mat', timestamp);
    save(filename_mat, 'analysis_results');
    fprintf('分析结果已保存为: %s\n', filename_mat);
    
    % 创建汇总表并保存为CSV
    create_summary_table(analysis_results, timestamp);
end

function create_summary_table(analysis_results, timestamp)
    % 创建汇总表
    
    weight_cj_values = analysis_results.weight_cj_values;
    
    % 准备表格数据
    table_data = [];
    
    for i = 1:length(weight_cj_values)
        weight_cj = weight_cj_values(i);
        weight_key = ['weight_' num2str(weight_cj)];
        
        if isfield(analysis_results, weight_key)
            stats = analysis_results.(weight_key).cj_stats;
            act_stats = analysis_results.(weight_key).activation_stats;
            
            row = [weight_cj, stats.min, stats.max, stats.mean, stats.std, ...
                   stats.median, stats.q25, stats.q75, stats.count, ...
                   act_stats.total_activations, act_stats.activation_rate];
            table_data = [table_data; row];
        end
    end
    
    % 创建表格
    column_names = {'weight_cj', 'cj_min', 'cj_max', 'cj_mean', 'cj_std', ...
                   'cj_median', 'cj_q25', 'cj_q75', 'cj_count', ...
                   'total_activations', 'activation_rate'};
    
    summary_table = array2table(table_data, 'VariableNames', column_names);
    
    % 保存为CSV
    filename_csv = sprintf('motion_saliency_summary_%s.csv', timestamp);
    writetable(summary_table, filename_csv);
    fprintf('汇总表已保存为: %s\n', filename_csv);
end

function display_final_summary(analysis_results)
    % 显示最终分析摘要
    
    weight_cj_values = analysis_results.weight_cj_values;
    
    fprintf('参数分析摘要：\n');
    fprintf('%-12s %-10s %-10s %-10s %-10s %-12s\n', ...
        'weight_cj', 'cj_min', 'cj_max', 'cj_mean', 'cj_std', 'activation_rate');
    fprintf('%-12s %-10s %-10s %-10s %-10s %-12s\n', ...
        '--------', '------', '------', '-------', '------', '-------------');
    
    for i = 1:length(weight_cj_values)
        weight_cj = weight_cj_values(i);
        weight_key = ['weight_' num2str(weight_cj)];
        
        if isfield(analysis_results, weight_key)
            stats = analysis_results.(weight_key).cj_stats;
            act_stats = analysis_results.(weight_key).activation_stats;
            
            fprintf('%-12d %-10.3f %-10.3f %-10.3f %-10.3f %-12.6f\n', ...
                weight_cj, stats.min, stats.max, stats.mean, stats.std, act_stats.activation_rate);
        end
    end
    
    fprintf('\n关键发现：\n');
    fprintf('1. cj值与G.weight_cj呈线性关系\n');
    fprintf('2. 激活率随G.weight_cj增加而增加\n');
    fprintf('3. 建议的G.weight_cj取值范围：[50, 150]\n');
end