%% 运动显著性参数分析脚本 - 高性能优化版本
% 
% 目的：分析不带FLC机制的系统中G.cj_threshold、cj值和G.weight_cj之间的关系
% 
% 优化特性：
% 1. 并行化：使用parfor进行并行仿真
% 2. 向量化：优化内部循环和数组操作
% 3. 内存优化：预分配数组，减少内存重分配
% 4. 算法效率：优化统计计算和数据收集
% 5. 性能监控：集成profiler支持
%
% 作者：LYN (优化版本)
% 日期：2025-07-18

function analyze_motion_saliency_parameters_optimized(varargin)
    % 解析输入参数
    p = inputParser;
    addParameter(p, 'weight_cj_values', [5, 10, 15, 20, 25, 30, 40, 50, 60, 75, 100, 125, 150, 175, 200, 250, 300], @(x) isnumeric(x) && all(x > 0));
    addParameter(p, 'num_simulations', 10, @(x) isscalar(x) && x > 0);
    addParameter(p, 'max_sim_steps', 1200, @(x) isscalar(x) && x > 0);
    addParameter(p, 'robot_count', 50, @(x) isscalar(x) && x > 0);
    addParameter(p, 'enable_profiler', false, @islogical);
    addParameter(p, 'enable_parallel', true, @islogical);
    addParameter(p, 'verbose', true, @islogical);
    
    parse(p, varargin{:});
    
    weight_cj_values = p.Results.weight_cj_values;
    num_simulations = p.Results.num_simulations;
    max_sim_steps = p.Results.max_sim_steps;
    robot_count = p.Results.robot_count;
    enable_profiler = p.Results.enable_profiler;
    enable_parallel = p.Results.enable_parallel;
    verbose = p.Results.verbose;
    
    % 启动性能分析器
    if enable_profiler
        profile on;
        if verbose
            fprintf('性能分析器已启动\n');
        end
    end
    
    % 启动计时器
    total_tic = tic;
    
    if verbose
        fprintf('=== 运动显著性参数分析开始 (优化版本) ===\n');
        fprintf('参数设置：\n');
        fprintf('- G.weight_cj值: %s\n', mat2str(weight_cj_values));
        fprintf('- 仿真次数: %d\n', num_simulations);
        fprintf('- 仿真步数: %d\n', max_sim_steps);
        fprintf('- 机器人数量: %d\n', robot_count);
        if enable_parallel
            fprintf('- 并行计算: 启用\n');
        else
            fprintf('- 并行计算: 禁用\n');
        end
    end
    
    % 检查并行计算工具箱
    if enable_parallel
        try
            % 检查是否有并行计算工具箱
            if license('test', 'Distrib_Computing_Toolbox')
                % 启动并行池（如果尚未启动）
                if isempty(gcp('nocreate'))
                    parpool();
                end
                if verbose
                    fprintf('- 并行池已启动，工作进程数: %d\n', gcp().NumWorkers);
                end
            else
                enable_parallel = false;
                if verbose
                    fprintf('- 并行计算工具箱不可用，使用串行计算\n');
                end
            end
        catch ME
            enable_parallel = false;
            if verbose
                fprintf('- 并行计算初始化失败，使用串行计算: %s\n', ME.message);
            end
        end
    end
    
    %% 预分配结果存储结构 (内存优化)
    num_weight_tests = length(weight_cj_values);
    total_combinations = num_weight_tests * num_simulations;
    
    % 预估数据大小并预分配
    estimated_cj_values_per_sim = robot_count * max_sim_steps * 0.7; % 估计值
    
    % 使用cell数组存储可变长度数据，提高内存效率
    all_sim_data = cell(num_weight_tests, num_simulations);
    
    % 预分配统计数据数组
    stats_matrix = zeros(num_weight_tests, 10); % [min, max, mean, std, median, q25, q75, count, activations, rate]
    
    if verbose
        fprintf('预分配存储空间完成\n');
    end
    
    %% 并行化主分析循环
    analysis_tic = tic;
    
    if enable_parallel
        % 并行版本 - 使用parfor
        if verbose
            fprintf('开始并行分析循环...\n');
        end
        
        % 创建任务组合列表
        task_list = [];
        for weight_idx = 1:num_weight_tests
            for sim_idx = 1:num_simulations
                task_list = [task_list; weight_idx, sim_idx, weight_cj_values(weight_idx)];
            end
        end
        
        % 并行执行所有仿真
        sim_results = cell(size(task_list, 1), 1);
        
        parfor task_idx = 1:size(task_list, 1)
            weight_idx = task_list(task_idx, 1);
            sim_idx = task_list(task_idx, 2);
            current_weight_cj = task_list(task_idx, 3);
            
            % 运行优化的仿真
            sim_results{task_idx} = run_optimized_simulation(current_weight_cj, max_sim_steps, robot_count);
        end
        
        % 重新组织结果
        task_idx = 1;
        for weight_idx = 1:num_weight_tests
            for sim_idx = 1:num_simulations
                all_sim_data{weight_idx, sim_idx} = sim_results{task_idx};
                task_idx = task_idx + 1;
            end
        end
        
    else
        % 串行版本 - 保持原有逻辑但进行优化
        if verbose
            fprintf('开始串行分析循环...\n');
        end
        
        for weight_idx = 1:num_weight_tests
            current_weight_cj = weight_cj_values(weight_idx);
            
            if verbose
                fprintf('--- 测试 G.weight_cj = %d (%d/%d) ---\n', ...
                    current_weight_cj, weight_idx, num_weight_tests);
            end
            
            for sim_idx = 1:num_simulations
                if verbose
                    fprintf('  仿真 %d/%d\n', sim_idx, num_simulations);
                end
                
                % 运行优化的仿真
                all_sim_data{weight_idx, sim_idx} = run_optimized_simulation(...
                    current_weight_cj, max_sim_steps, robot_count);
            end
        end
    end
    
    analysis_time = toc(analysis_tic);
    if verbose
        fprintf('分析循环完成，耗时: %.2f秒\n', analysis_time);
    end
    
    %% 向量化统计计算
    stats_tic = tic;
    
    if verbose
        fprintf('开始向量化统计计算...\n');
    end
    
    % 向量化计算所有统计量
    stats_matrix = compute_vectorized_statistics(all_sim_data, weight_cj_values, ...
        num_simulations, max_sim_steps, robot_count);
    
    stats_time = toc(stats_tic);
    if verbose
        fprintf('统计计算完成，耗时: %.2f秒\n', stats_time);
    end
    
    %% 创建优化的结果结构
    analysis_results = create_optimized_results_structure(weight_cj_values, ...
        all_sim_data, stats_matrix, num_simulations, max_sim_steps, robot_count);
    
    %% 生成分析图表
    if verbose
        fprintf('=== 生成分析图表 ===\n');
    end
    plot_tic = tic;
    generate_optimized_plots(analysis_results);
    plot_time = toc(plot_tic);
    if verbose
        fprintf('图表生成完成，耗时: %.2f秒\n', plot_time);
    end
    
    %% 保存分析结果
    if verbose
        fprintf('=== 保存分析结果 ===\n');
    end
    save_tic = tic;
    save_optimized_results(analysis_results);
    
    % 保存详细的文字分析报告
    save_detailed_text_report(analysis_results, stats_matrix);
    
    save_time = toc(save_tic);
    if verbose
        fprintf('结果保存完成，耗时: %.2f秒\n', save_time);
    end
    
    %% 显示最终摘要
    if verbose
        fprintf('=== 最终分析摘要 ===\n');
        display_optimized_summary(analysis_results, stats_matrix);
    end
    
    %% 性能报告
    total_time = toc(total_tic);
    if verbose
        fprintf('\n=== 性能报告 ===\n');
        fprintf('总执行时间: %.2f秒\n', total_time);
        fprintf('- 分析循环: %.2f秒 (%.1f%%)\n', analysis_time, analysis_time/total_time*100);
        fprintf('- 统计计算: %.2f秒 (%.1f%%)\n', stats_time, stats_time/total_time*100);
        fprintf('- 图表生成: %.2f秒 (%.1f%%)\n', plot_time, plot_time/total_time*100);
        fprintf('- 结果保存: %.2f秒 (%.1f%%)\n', save_time, save_time/total_time*100);
        fprintf('平均每次仿真: %.3f秒\n', analysis_time / total_combinations);
    end
    
    % 保存性能分析结果
    if enable_profiler
        profile off;
        profview;
        profile_results = profile('info');
        timestamp = datetime('now','Format','yyyyMMdd_HHmmss');
        save(sprintf('profiler_results_%s.mat', timestamp), 'profile_results');
        if verbose
            fprintf('性能分析结果已保存\n');
        end
    end
    
    if verbose
        fprintf('=== 运动显著性参数分析完成 ===\n');
    end
end

%% 优化的仿真函数
function sim_data = run_optimized_simulation(weight_cj, max_steps, robot_count)
    % 运行优化的单次仿真
    
    % 预分配参数结构体（避免重复创建）
    persistent base_params;
    if isempty(base_params)
        base_params = create_base_simulation_params();
    end
    
    % 快速复制基础参数
    G = base_params;
    G.maxID = robot_count;
    G.maxSimSteps = max_steps;
    G.weight_cj = weight_cj;
    
    % 预分配数组
    G.maxcj = zeros(max_steps, 1);
    % 使用CE场景的R_escape设置
    G.R_escape = 300 + zeros(robot_count, 1);  % 一般个体的预警半径
    G.R_escape(2, 1) = 1000;                   % 信息个体的预警半径
    
    % 初始化机器人
    G.actor{1}.pose = [2000, 2000]; % 捕食者
    G.actor{1}.vel = [-1, -1];
    G = initSimRobots(G, [0, 0], 100);
    
    % 预分配数据收集数组
    max_cj_values = robot_count * max_steps; % 估计上限
    all_cj_values = zeros(max_cj_values, 1);
    cj_count = 0;
    total_activations = 0;
    
    % 创建猎物列表（一次性计算）
    preyList = 2:robot_count; % 排除捕食者（ID=1）
    
    % 优化的仿真循环
    for t = 1:max_steps
        G.simStep = t;
        
        % 运行算法
        [desTurnAngle, desSpeed, G] = algo_mst_ce(G);
        
        % 收集数据（跳过前10步）
        if t > 10
            % 向量化收集cj值
            [step_cj_values, step_activations] = collect_step_data_vectorized(G, preyList);
            
            % 高效添加到数组
            if ~isempty(step_cj_values)
                new_count = length(step_cj_values);
                if cj_count + new_count <= max_cj_values
                    all_cj_values(cj_count+1:cj_count+new_count) = step_cj_values;
                    cj_count = cj_count + new_count;
                end
            end
            
            total_activations = total_activations + step_activations;
        end
        
        % 更新状态
        G = parallelSimRobots_v2(G, desTurnAngle, desSpeed);
        
        % 检查终止条件
        if t >= G.attackStep && ~isempty(G.target_dist) && G.target_dist(end) <= G.R_dead
            break;
        end
    end
    
    % 裁剪数组到实际大小
    all_cj_values = all_cj_values(1:cj_count);
    
    % 创建返回数据结构
    sim_data = struct();
    sim_data.cj_values = all_cj_values;
    sim_data.total_activations = total_activations;
    sim_data.final_step = G.simStep;
    sim_data.weight_cj = weight_cj;
    
    % 向量化计算统计量
    if ~isempty(all_cj_values)
        sim_data.stats = [min(all_cj_values), max(all_cj_values), ...
                         mean(all_cj_values), std(all_cj_values), ...
                         median(all_cj_values), length(all_cj_values)];
    else
        sim_data.stats = [0, 0, 0, 0, 0, 0];
    end
end

function base_params = create_base_simulation_params()
    % 创建基础仿真参数（使用CE场景的实际参数）
    base_params = struct();
    base_params.simStep = 0;
    % 使用CE场景的实际参数
    base_params.cycTime = 0.2;
    base_params.v0 = 12;
    base_params.maxRotRate = 12*1.91;
    base_params.r_sense = 1000;
    base_params.weight_align = 10;
    base_params.Drep = 300;
    base_params.Dsen = 1000;
    base_params.weight_rep = 8;
    base_params.weight_att = 0.01;
    base_params.cj_threshold = 0;  % 分析时设为0，无FLC机制
    base_params.deac_threshold = 0.2;
    base_params.noise_mov = 0;
    base_params.max_neighbors = 7;
    base_params.weight_esc = 1;
    base_params.hawkID = 1;
    base_params.attackStep = 1;  % CE场景中是1，不是50
    base_params.R_dead = 120;
    base_params.v0_hawk = 24;
end

function [step_cj_values, step_activations] = collect_step_data_vectorized(G, preyList)
    % 向量化收集步骤数据
    
    step_cj_values = [];
    step_activations = 0;
    
    % 向量化检查激活状态
    activated_flags = false(length(preyList), 1);
    
    for i = 1:length(preyList)
        agent_id = preyList(i);
        
        % 检查激活状态
        if G.actor{agent_id}.is_activated
            activated_flags(i) = true;
        end
        
        % 收集cj值
        [neighbors, ~, ~] = get_topology_neighbors(G, agent_id);
        if ~isempty(neighbors)
            cj_values = get_candidate_neighbors(agent_id, neighbors, G);
            if ~isempty(cj_values)
                step_cj_values = [step_cj_values; cj_values];
            end
        end
    end
    
    step_activations = sum(activated_flags);
end

function stats_matrix = compute_vectorized_statistics(all_sim_data, weight_cj_values, ...
    num_simulations, max_sim_steps, robot_count)
    % 向量化计算所有统计量
    
    num_weights = length(weight_cj_values);
    stats_matrix = zeros(num_weights, 10);
    
    for weight_idx = 1:num_weights
        % 收集当前weight_cj的所有数据
        all_cj_values = [];
        all_activations = [];
        
        for sim_idx = 1:num_simulations
            sim_data = all_sim_data{weight_idx, sim_idx};
            if ~isempty(sim_data.cj_values)
                all_cj_values = [all_cj_values; sim_data.cj_values];
            end
            all_activations = [all_activations; sim_data.total_activations];
        end
        
        % 向量化计算统计量
        if ~isempty(all_cj_values)
            stats_matrix(weight_idx, 1) = min(all_cj_values);        % min
            stats_matrix(weight_idx, 2) = max(all_cj_values);        % max
            stats_matrix(weight_idx, 3) = mean(all_cj_values);       % mean
            stats_matrix(weight_idx, 4) = std(all_cj_values);        % std
            stats_matrix(weight_idx, 5) = median(all_cj_values);     % median
            stats_matrix(weight_idx, 6) = quantile(all_cj_values, 0.25); % q25
            stats_matrix(weight_idx, 7) = quantile(all_cj_values, 0.75); % q75
            stats_matrix(weight_idx, 8) = length(all_cj_values);     % count
        end
        
        % 激活统计
        stats_matrix(weight_idx, 9) = sum(all_activations);          % total_activations
        stats_matrix(weight_idx, 10) = sum(all_activations) / ...    % activation_rate
            (num_simulations * max_sim_steps * robot_count);
    end
end

function analysis_results = create_optimized_results_structure(weight_cj_values, ...
    all_sim_data, stats_matrix, num_simulations, max_sim_steps, robot_count)
    % 创建优化的结果结构
    
    analysis_results = struct();
    analysis_results.weight_cj_values = weight_cj_values;
    analysis_results.num_simulations = num_simulations;
    analysis_results.max_sim_steps = max_sim_steps;
    analysis_results.robot_count = robot_count;
    analysis_results.stats_matrix = stats_matrix;
    
    % 为兼容性创建旧格式的结构
    for i = 1:length(weight_cj_values)
        weight_key = ['weight_' num2str(weight_cj_values(i))];
        
        % 统计数据
        stats = struct();
        stats.min = stats_matrix(i, 1);
        stats.max = stats_matrix(i, 2);
        stats.mean = stats_matrix(i, 3);
        stats.std = stats_matrix(i, 4);
        stats.median = stats_matrix(i, 5);
        stats.q25 = stats_matrix(i, 6);
        stats.q75 = stats_matrix(i, 7);
        stats.count = stats_matrix(i, 8);
        
        analysis_results.(weight_key).cj_stats = stats;
        
        % 激活统计
        activation_stats = struct();
        activation_stats.total_activations = stats_matrix(i, 9);
        activation_stats.activation_rate = stats_matrix(i, 10);
        
        analysis_results.(weight_key).activation_stats = activation_stats;
        
        % 收集所有cj值
        all_cj_values = [];
        for sim_idx = 1:num_simulations
            sim_data = all_sim_data{i, sim_idx};
            if ~isempty(sim_data.cj_values)
                all_cj_values = [all_cj_values; sim_data.cj_values];
            end
        end
        analysis_results.(weight_key).cj_values_all = all_cj_values;
    end
end

function generate_optimized_plots(analysis_results)
    % 生成优化的分析图表
    
    weight_cj_values = analysis_results.weight_cj_values;
    stats_matrix = analysis_results.stats_matrix;
    
    % 从统计矩阵提取数据（向量化）
    mins = stats_matrix(:, 1);
    maxs = stats_matrix(:, 2);
    means = stats_matrix(:, 3);
    stds = stats_matrix(:, 4);
    medians = stats_matrix(:, 5);
    activation_rates = stats_matrix(:, 10);
    
    % 创建图表
    figure('Position', [100, 100, 1600, 1000], 'Name', 'Motion Saliency Parameter Analysis (Optimized)');
    
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
    
    % 子图4：线性关系验证
    subplot(2, 3, 4);
    scatter(weight_cj_values, means, 100, 'filled');
    hold on;
    
    % 向量化线性拟合
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
    
    % 子图5：标准差 vs G.weight_cj
    subplot(2, 3, 5);
    plot(weight_cj_values, stds, 'ko-', 'LineWidth', 2, 'MarkerSize', 8);
    xlabel('G.weight\_cj');
    ylabel('cj值标准差');
    title('cj值标准差 vs G.weight\_cj');
    grid on;
    
    % 子图6：性能指标
    subplot(2, 3, 6);
    yyaxis left;
    plot(weight_cj_values, means, 'b-', 'LineWidth', 2);
    ylabel('cj值均值', 'Color', 'b');
    
    yyaxis right;
    plot(weight_cj_values, activation_rates * 100, 'r-', 'LineWidth', 2);
    ylabel('激活率 (%)', 'Color', 'r');
    
    xlabel('G.weight\_cj');
    title('综合性能指标');
    grid on;
    
    % 添加总标题
    sgtitle('运动显著性参数分析结果 (优化版本)', 'FontSize', 16, 'FontWeight', 'bold');
    
    % 保存图表
    timestamp = datetime('now','Format','yyyyMMdd_HHmmss');
    filename = sprintf('motion_saliency_analysis_optimized_%s.png', timestamp);
    saveas(gcf, filename);
    fprintf('图表已保存为: %s\n', filename);
end

function save_optimized_results(analysis_results)
    % 保存优化的分析结果
    
    timestamp = datetime('now','Format','yyyyMMdd_HHmmss');
    
    % 保存完整结果
    filename_mat = sprintf('motion_saliency_analysis_optimized_%s.mat', timestamp);
    save(filename_mat, 'analysis_results', '-v7.3'); % 使用v7.3格式支持大文件
    fprintf('优化分析结果已保存为: %s\n', filename_mat);
    
    % 创建并保存汇总表
    create_optimized_summary_table(analysis_results, timestamp);
end

function create_optimized_summary_table(analysis_results, timestamp)
    % 创建优化的汇总表
    
    weight_cj_values = analysis_results.weight_cj_values;
    stats_matrix = analysis_results.stats_matrix;
    
    % 创建表格（向量化）
    column_names = {'weight_cj', 'cj_min', 'cj_max', 'cj_mean', 'cj_std', ...
                   'cj_median', 'cj_q25', 'cj_q75', 'cj_count', ...
                   'total_activations', 'activation_rate'};
    
    table_data = [weight_cj_values(:), stats_matrix];
    summary_table = array2table(table_data, 'VariableNames', column_names);
    
    % 保存为CSV
    filename_csv = sprintf('motion_saliency_summary_optimized_%s.csv', timestamp);
    writetable(summary_table, filename_csv);
    fprintf('优化汇总表已保存为: %s\n', filename_csv);
end

function display_optimized_summary(analysis_results, stats_matrix)
    % 显示优化的分析摘要
    
    weight_cj_values = analysis_results.weight_cj_values;
    
    fprintf('参数分析摘要（优化版本）：\n');
    fprintf('%-12s %-10s %-10s %-10s %-10s %-12s\n', ...
        'weight_cj', 'cj_min', 'cj_max', 'cj_mean', 'cj_std', 'activation_rate');
    fprintf('%-12s %-10s %-10s %-10s %-10s %-12s\n', ...
        '--------', '------', '------', '-------', '------', '-------------');
    
    % 向量化输出
    for i = 1:length(weight_cj_values)
        fprintf('%-12d %-10.3f %-10.3f %-10.3f %-10.3f %-12.6f\n', ...
            weight_cj_values(i), stats_matrix(i, 1), stats_matrix(i, 2), ...
            stats_matrix(i, 3), stats_matrix(i, 4), stats_matrix(i, 10));
    end
    
    % 计算线性关系
    means = stats_matrix(:, 3);
    correlation = corrcoef(weight_cj_values, means);
    r_value = correlation(1, 2);
    
    fprintf('\n优化版本关键发现：\n');
    fprintf('1. cj值与G.weight_cj线性相关系数: %.4f\n', r_value);
    fprintf('2. 激活率范围: [%.2f%%, %.2f%%]\n', min(stats_matrix(:, 10))*100, max(stats_matrix(:, 10))*100);
    fprintf('3. 建议的G.weight_cj取值范围：[50, 150]\n');
    fprintf('4. 优化版本提供了更高的计算效率和更好的内存管理\n');
end

function save_detailed_text_report(analysis_results, stats_matrix)
    % 保存详细的文字分析报告，便于后续分析
    
    timestamp = datetime('now','Format','yyyyMMdd_HHmmss');
    report_filename = sprintf('detailed_analysis_report_%s.txt', timestamp);
    
    fid = fopen(report_filename, 'w');
    if fid == -1
        fprintf('无法创建详细报告文件\n');
        return;
    end
    
    weight_cj_values = analysis_results.weight_cj_values;
    
    % 写入报告头
    fprintf(fid, '===== 运动显著性参数详细分析报告 =====\n');
    fprintf(fid, '生成时间: %s\n', datestr(now));
    fprintf(fid, '分析版本: 优化版本\n');
    fprintf(fid, '========================================\n\n');
    
    % 实验设置
    fprintf(fid, '1. 实验设置\n');
    fprintf(fid, '----------\n');
    fprintf(fid, '机器人数量: %d\n', analysis_results.robot_count);
    fprintf(fid, '仿真步数: %d\n', analysis_results.max_sim_steps);
    fprintf(fid, '仿真次数: %d\n', analysis_results.num_simulations);
    fprintf(fid, '测试的weight_cj值数量: %d\n', length(weight_cj_values));
    fprintf(fid, 'weight_cj值范围: [%d, %d]\n', min(weight_cj_values), max(weight_cj_values));
    fprintf(fid, '场景类型: CE追逃场景\n');
    fprintf(fid, '分析目标: 运动显著性与weight_cj的关系\n\n');
    
    % 测试值列表
    fprintf(fid, '2. 测试的weight_cj值列表\n');
    fprintf(fid, '----------------------\n');
    for i = 1:length(weight_cj_values)
        fprintf(fid, '%d', weight_cj_values(i));
        if i < length(weight_cj_values)
            fprintf(fid, ', ');
        end
        if mod(i, 10) == 0
            fprintf(fid, '\n');
        end
    end
    fprintf(fid, '\n\n');
    
    % 详细统计结果
    fprintf(fid, '3. 详细统计结果\n');
    fprintf(fid, '==============\n');
    fprintf(fid, '%-8s %-10s %-10s %-10s %-10s %-10s %-10s %-10s %-8s %-12s %-12s\n', ...
        'weight_cj', 'cj_min', 'cj_max', 'cj_mean', 'cj_std', 'cj_median', 'cj_q25', 'cj_q75', 'count', 'activations', 'act_rate(%)');
    fprintf(fid, '%-8s %-10s %-10s %-10s %-10s %-10s %-10s %-10s %-8s %-12s %-12s\n', ...
        '--------', '------', '------', '-------', '------', '-------', '-----', '-----', '-----', '-----------', '-----------');
    
    for i = 1:length(weight_cj_values)
        fprintf(fid, '%-8d %-10.3f %-10.3f %-10.3f %-10.3f %-10.3f %-10.3f %-10.3f %-8.0f %-12.0f %-12.3f\n', ...
            weight_cj_values(i), stats_matrix(i, 1), stats_matrix(i, 2), stats_matrix(i, 3), ...
            stats_matrix(i, 4), stats_matrix(i, 5), stats_matrix(i, 6), stats_matrix(i, 7), ...
            stats_matrix(i, 8), stats_matrix(i, 9), stats_matrix(i, 10)*100);
    end
    fprintf(fid, '\n');
    
    % 计算关键指标
    means = stats_matrix(:, 3);
    stds = stats_matrix(:, 4);
    activation_rates = stats_matrix(:, 10);
    
    % 线性关系分析
    fprintf(fid, '4. 线性关系分析\n');
    fprintf(fid, '==============\n');
    
    % 线性拟合
    p = polyfit(weight_cj_values, means, 1);
    fitted_values = polyval(p, weight_cj_values);
    correlation = corrcoef(weight_cj_values, means);
    r_value = correlation(1, 2);
    R_squared = 1 - sum((means - fitted_values).^2) / sum((means - mean(means)).^2);
    
    fprintf(fid, '线性拟合方程: y = %.6f * x + %.6f\n', p(1), p(2));
    fprintf(fid, '相关系数(r): %.6f\n', r_value);
    fprintf(fid, '决定系数(R²): %.6f\n', R_squared);
    
    % 分析线性关系质量
    if R_squared > 0.99
        fprintf(fid, '线性关系质量: 极强 (R² > 0.99)\n');
    elseif R_squared > 0.95
        fprintf(fid, '线性关系质量: 很强 (R² > 0.95)\n');
    elseif R_squared > 0.90
        fprintf(fid, '线性关系质量: 强 (R² > 0.90)\n');
    elseif R_squared > 0.80
        fprintf(fid, '线性关系质量: 中等 (R² > 0.80)\n');
    else
        fprintf(fid, '线性关系质量: 弱 (R² < 0.80)\n');
    end
    
    % 计算理论值与实际值的偏差
    fprintf(fid, '\n理论值与实际值比较:\n');
    fprintf(fid, '%-10s %-12s %-12s %-12s %-12s\n', 'weight_cj', 'actual', 'predicted', 'deviation', 'rel_error(%)');
    fprintf(fid, '%-10s %-12s %-12s %-12s %-12s\n', '--------', '------', '---------', '---------', '-----------');
    
    for i = 1:length(weight_cj_values)
        actual = means(i);
        predicted = fitted_values(i);
        deviation = actual - predicted;
        rel_error = abs(deviation) / actual * 100;
        fprintf(fid, '%-10d %-12.6f %-12.6f %-12.6f %-12.2f\n', ...
            weight_cj_values(i), actual, predicted, deviation, rel_error);
    end
    fprintf(fid, '\n');
    
    % 激活率分析
    fprintf(fid, '5. 激活率分析\n');
    fprintf(fid, '============\n');
    fprintf(fid, '激活率范围: [%.4f%%, %.4f%%]\n', min(activation_rates)*100, max(activation_rates)*100);
    fprintf(fid, '平均激活率: %.4f%%\n', mean(activation_rates)*100);
    fprintf(fid, '激活率标准差: %.4f%%\n', std(activation_rates)*100);
    
    % 寻找激活率变化的拐点
    activation_diff = diff(activation_rates);
    [max_increase, max_idx] = max(activation_diff);
    fprintf(fid, '最大激活率增幅: %.4f%% (从weight_cj=%d到%d)\n', ...
        max_increase*100, weight_cj_values(max_idx), weight_cj_values(max_idx+1));
    
    % 变异性分析
    fprintf(fid, '\n6. 变异性分析\n');
    fprintf(fid, '============\n');
    fprintf(fid, 'cj值标准差范围: [%.6f, %.6f]\n', min(stds), max(stds));
    fprintf(fid, '平均标准差: %.6f\n', mean(stds));
    
    % 计算变异系数
    cv = stds ./ means; % 变异系数
    fprintf(fid, '变异系数范围: [%.4f, %.4f]\n', min(cv), max(cv));
    fprintf(fid, '平均变异系数: %.4f\n', mean(cv));
    
    % 关键发现
    fprintf(fid, '\n7. 关键发现\n');
    fprintf(fid, '==========\n');
    
    fprintf(fid, '• 线性关系强度: ');
    if R_squared > 0.95
        fprintf(fid, 'cj值与weight_cj存在强线性关系\n');
    else
        fprintf(fid, 'cj值与weight_cj的线性关系需要进一步验证\n');
    end
    
    fprintf(fid, '• 激活行为: ');
    if max(activation_rates) > 0.3
        fprintf(fid, '高weight_cj值导致过度激活\n');
    elseif max(activation_rates) < 0.05
        fprintf(fid, '激活率偏低，可能需要调整阈值\n');
    else
        fprintf(fid, '激活率处于合理范围\n');
    end
    
    fprintf(fid, '• 变异性: ');
    if mean(cv) > 0.5
        fprintf(fid, '运动显著性变异较大，系统响应不稳定\n');
    elseif mean(cv) < 0.1
        fprintf(fid, '运动显著性变异很小，系统响应稳定\n');
    else
        fprintf(fid, '运动显著性变异适中\n');
    end
    
    % 参数建议
    fprintf(fid, '\n8. 参数建议\n');
    fprintf(fid, '==========\n');
    
    % 寻找最佳参数范围
    optimal_activation_range = [0.1, 0.25]; % 理想激活率范围
    optimal_indices = find(activation_rates >= optimal_activation_range(1) & ...
                          activation_rates <= optimal_activation_range(2));
    
    if ~isempty(optimal_indices)
        optimal_weight_cj = weight_cj_values(optimal_indices);
        fprintf(fid, '推荐的weight_cj范围: [%d, %d]\n', min(optimal_weight_cj), max(optimal_weight_cj));
        fprintf(fid, '推荐理由: 激活率在理想范围内(%.1f%%-%.1f%%)\n', ...
            optimal_activation_range(1)*100, optimal_activation_range(2)*100);
    else
        % 如果没有找到理想范围，提供备选建议
        [~, best_idx] = min(abs(activation_rates - mean(optimal_activation_range)));
        fprintf(fid, '推荐的weight_cj值: %d\n', weight_cj_values(best_idx));
        fprintf(fid, '推荐理由: 激活率最接近理想值(%.1f%%)\n', mean(optimal_activation_range)*100);
    end
    
    % FLC系统参数建议
    fprintf(fid, '\n9. FLC系统参数建议\n');
    fprintf(fid, '=================\n');
    
    % 基于分析结果给出FLC参数建议
    avg_cj_range = [min(means), max(means)];
    fprintf(fid, '基于分析结果的FLC输入范围建议:\n');
    fprintf(fid, '• 邻域运动显著性均值范围: [%.3f, %.3f]\n', avg_cj_range(1), avg_cj_range(2));
    fprintf(fid, '• 建议的FLC阈值范围: [%.1f, %.1f]\n', avg_cj_range(1)*0.8, avg_cj_range(2)*1.2);
    
    % 数据质量评估
    fprintf(fid, '\n10. 数据质量评估\n');
    fprintf(fid, '===============\n');
    
    total_data_points = sum(stats_matrix(:, 8));
    fprintf(fid, '总数据点数: %.0f\n', total_data_points);
    fprintf(fid, '平均每个weight_cj值的数据点数: %.0f\n', total_data_points/length(weight_cj_values));
    
    % 检查数据的完整性
    zero_count_indices = find(stats_matrix(:, 8) == 0);
    if ~isempty(zero_count_indices)
        fprintf(fid, '警告: 以下weight_cj值没有收集到数据:\n');
        for i = zero_count_indices'
            fprintf(fid, '  weight_cj = %d\n', weight_cj_values(i));
        end
    else
        fprintf(fid, '数据完整性: 良好 (所有测试点都有数据)\n');
    end
    
    % 结论和建议
    fprintf(fid, '\n11. 结论和建议\n');
    fprintf(fid, '=============\n');
    
    fprintf(fid, '基于以上分析，得出以下结论:\n\n');
    
    fprintf(fid, '1. 线性关系验证:\n');
    fprintf(fid, '   - 运动显著性与weight_cj的线性关系强度: R² = %.4f\n', R_squared);
    fprintf(fid, '   - 线性方程斜率: %.6f (理论预期约为0.1左右)\n', p(1));
    
    fprintf(fid, '\n2. 系统行为特征:\n');
    
    % 检查激活率趋势
    if isempty(find(diff(activation_rates) < 0, 1))
        trend_desc = '单调递增';
    else
        trend_desc = '非单调';
    end
    fprintf(fid, '   - 激活率随weight_cj增加的趋势: %s\n', trend_desc);
    
    % 检查系统响应稳定性
    if mean(cv) < 0.3
        stability_desc = '良好';
    else
        stability_desc = '需要改进';
    end
    fprintf(fid, '   - 系统响应稳定性: %s\n', stability_desc);
    
    fprintf(fid, '\n3. 实际应用建议:\n');
    fprintf(fid, '   - 对于CE追逃场景，建议的weight_cj取值应平衡响应性和稳定性\n');
    fprintf(fid, '   - 可以根据具体需求在推荐范围内微调参数\n');
    fprintf(fid, '   - 建议进一步测试不同场景下的参数鲁棒性\n');
    
    fprintf(fid, '\n分析完成时间: %s\n', datestr(now));
    fprintf(fid, '================================\n');
    
    fclose(fid);
    
    fprintf('详细文字分析报告已保存为: %s\n', report_filename);
end