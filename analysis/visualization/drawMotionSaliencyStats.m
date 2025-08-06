function drawMotionSaliencyStats(G)
    % 绘制运动显著性相关统计数据
    % 输入: G - 包含FLC数据的全局结构体
    
    % 创建新的图形窗口
    figure('Position', [150, 150, 1400, 800], 'Name', 'Motion Saliency Statistics');
    
    % 获取有效的仿真步数
    valid_steps = 1:G.simStep;
    
    % 创建猎物列表（排除捕食者）
    preyList = setdiff(G.robotsList, G.hawkID);
    
    %% 子图1: 运动显著性最小值时间序列
    subplot(2, 3, 1);
    hold on;
    for i = preyList
        plot(valid_steps, G.flc_data.ms_min(valid_steps, i), 'LineWidth', 0.8);
    end
    % 绘制平均值
    mean_ms_min = mean(G.flc_data.ms_min(valid_steps, preyList), 2);
    plot(valid_steps, mean_ms_min, 'k-', 'LineWidth', 2, 'DisplayName', 'Mean');
    
    xlabel('仿真步数');
    ylabel('运动显著性最小值');
    title('个体观察到的运动显著性最小值');
    grid on;
    
    %% 子图2: 运动显著性最大值时间序列
    subplot(2, 3, 2);
    hold on;
    for i = preyList
        plot(valid_steps, G.flc_data.ms_max(valid_steps, i), 'LineWidth', 0.8);
    end
    % 绘制平均值
    mean_ms_max = mean(G.flc_data.ms_max(valid_steps, preyList), 2);
    plot(valid_steps, mean_ms_max, 'k-', 'LineWidth', 2, 'DisplayName', 'Mean');
    
    xlabel('仿真步数');
    ylabel('运动显著性最大值');
    title('个体观察到的运动显著性最大值');
    grid on;
    
    %% 子图3: 运动显著性范围时间序列
    subplot(2, 3, 3);
    hold on;
    for i = preyList
        plot(valid_steps, G.flc_data.ms_range(valid_steps, i), 'LineWidth', 0.8);
    end
    % 绘制平均值
    mean_ms_range = mean(G.flc_data.ms_range(valid_steps, preyList), 2);
    plot(valid_steps, mean_ms_range, 'k-', 'LineWidth', 2, 'DisplayName', 'Mean');
    
    xlabel('仿真步数');
    ylabel('运动显著性范围 (最大值-最小值)');
    title('个体观察到的运动显著性范围');
    grid on;
    
    %% 子图4: 邻居数量时间序列
    subplot(2, 3, 4);
    hold on;
    for i = preyList
        plot(valid_steps, G.flc_data.ms_count(valid_steps, i), 'LineWidth', 0.8);
    end
    % 绘制平均值
    mean_ms_count = mean(G.flc_data.ms_count(valid_steps, preyList), 2);
    plot(valid_steps, mean_ms_count, 'k-', 'LineWidth', 2, 'DisplayName', 'Mean');
    
    xlabel('仿真步数');
    ylabel('邻居数量');
    title('个体邻居数量变化');
    grid on;
    
    %% 子图5: 运动显著性分布统计（箱线图）
    subplot(2, 3, 5);
    
    % 收集所有非零的运动显著性值用于箱线图
    all_ms_values = [];
    time_labels = [];
    
    % 每隔一定步数采样，避免数据过多
    sample_interval = max(1, floor(G.simStep / 20));  % 最多采样20个时间点
    sample_steps = 1:sample_interval:G.simStep;
    
    for t_idx = 1:length(sample_steps)
        t = sample_steps(t_idx);
        step_values = [];
        for i = preyList
            if G.flc_data.ms_count(t, i) > 0  % 只考虑有邻居的情况
                % 这里我们使用均值作为代表值，实际上可以收集所有原始值
                step_values = [step_values, G.flc_data.avg_mj(t, i)];
            end
        end
        if ~isempty(step_values)
            all_ms_values = [all_ms_values, step_values];
            time_labels = [time_labels, repmat(t, 1, length(step_values))];
        end
    end
    
    if ~isempty(all_ms_values)
        % 创建分组箱线图
        boxplot(all_ms_values, time_labels);
        xlabel('仿真步数');
        ylabel('运动显著性值');
        title('运动显著性分布（箱线图）');
        grid on;
    else
        text(0.5, 0.5, '无有效数据', 'HorizontalAlignment', 'center', 'Units', 'normalized');
        title('运动显著性分布（箱线图）');
    end
    
    %% 子图6: 运动显著性与阈值的关系
    subplot(2, 3, 6);
    
    % 收集所有有效的运动显著性最大值和对应的阈值
    valid_ms_max = [];
    valid_thresholds = [];
    
    for t = valid_steps
        for i = preyList
            if G.flc_data.ms_count(t, i) > 0  % 只考虑有邻居的情况
                valid_ms_max = [valid_ms_max, G.flc_data.ms_max(t, i)];
                valid_thresholds = [valid_thresholds, G.flc_data.thresholds(t, i)];
            end
        end
    end
    
    if ~isempty(valid_ms_max)
        scatter(valid_ms_max, valid_thresholds, 'filled', 'Alpha', 0.6);
        xlabel('运动显著性最大值');
        ylabel('当前阈值');
        title('运动显著性最大值 vs 当前阈值');
        grid on;
        
        % 添加激活线（y=x线）
        hold on;
        max_val = max([max(valid_ms_max), max(valid_thresholds)]);
        plot([0, max_val], [0, max_val], 'r--', 'LineWidth', 2, 'DisplayName', '激活线 (MS=阈值)');
        legend('数据点', '激活线', 'Location', 'best');
    else
        text(0.5, 0.5, '无有效数据', 'HorizontalAlignment', 'center', 'Units', 'normalized');
        title('运动显著性最大值 vs 当前阈值');
    end
    
    %% 添加总体信息文本
    sgtitle(sprintf('运动显著性统计分析 (仿真步数: %d, 个体数量: %d)', ...
        G.simStep, length(preyList)), 'FontSize', 14, 'FontWeight', 'bold');
    
    % 计算并显示统计摘要
    fprintf('\n=== 运动显著性统计摘要 ===\n');
    fprintf('仿真步数: %d\n', G.simStep);
    fprintf('个体数量: %d\n', length(preyList));
    
    if ~isempty(valid_ms_max)
        fprintf('运动显著性最大值范围: [%.2f, %.2f]\n', min(valid_ms_max), max(valid_ms_max));
        fprintf('运动显著性最大值均值: %.3f\n', mean(valid_ms_max));
        fprintf('运动显著性最大值标准差: %.3f\n', std(valid_ms_max));
    end
    
    fprintf('平均邻居数量: %.2f\n', mean(mean_ms_count));
    fprintf('平均运动显著性范围: %.3f\n', mean(mean_ms_range));
    
    % 计算激活率（运动显著性超过阈值的比例）
    activation_ratio = sum(valid_ms_max > valid_thresholds) / length(valid_ms_max) * 100;
    fprintf('激活率: %.1f%% (运动显著性超过阈值的比例)\n', activation_ratio);
    fprintf('========================\n\n');
end