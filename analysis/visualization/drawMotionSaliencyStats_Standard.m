function drawMotionSaliencyStats_Standard(G)
    % 绘制标准算法的运动显著性统计数据
    % 输入: G - 包含标准算法数据的全局结构体
    
    % 创建新的图形窗口
    figure('Position', [200, 100, 1400, 900], 'Name', 'Motion Saliency Statistics - Standard Algorithm');
    
    % 获取有效的仿真步数
    valid_steps = 1:G.simStep;
    
    %% 子图1: 最大运动显著性时间序列
    subplot(2, 3, 1);
    plot(valid_steps, G.maxcj(valid_steps), 'b-', 'LineWidth', 2);
    hold on;
    % 添加阈值参考线
    yline(G.cj_threshold, 'r--', 'LineWidth', 2, 'DisplayName', sprintf('阈值 = %.1f', G.cj_threshold));
    
    xlabel('仿真步数');
    ylabel('最大运动显著性 (maxcj)');
    title('群体最大运动显著性变化');
    legend('maxcj', '阈值线', 'Location', 'best');
    grid on;
    
    %% 子图2: 激活个体数量时间序列
    subplot(2, 3, 2);
    if isfield(G, 'activatedCount')
        plot(valid_steps, G.activatedCount(valid_steps), 'g-', 'LineWidth', 2);
        xlabel('仿真步数');
        ylabel('激活个体数量');
        title('激活个体数量变化');
        grid on;
    else
        text(0.5, 0.5, '无激活数据', 'HorizontalAlignment', 'center', 'Units', 'normalized');
        title('激活个体数量变化');
    end
    
    %% 子图3: 预警个体数量时间序列  
    subplot(2, 3, 3);
    plot(valid_steps, G.warnNum(valid_steps), 'r-', 'LineWidth', 2);
    xlabel('仿真步数');
    ylabel('预警个体数量');
    title('预警个体数量变化');
    grid on;
    
    %% 子图4: 运动显著性超阈值比例
    subplot(2, 3, 4);
    % 计算超阈值的步数比例
    above_threshold = G.maxcj(valid_steps) > G.cj_threshold;
    above_threshold_ratio = cumsum(above_threshold) ./ (1:length(valid_steps));
    
    plot(valid_steps, above_threshold_ratio * 100, 'm-', 'LineWidth', 2);
    xlabel('仿真步数');
    ylabel('超阈值比例 (%)');
    title('累积超阈值比例');
    grid on;
    ylim([0, 100]);
    
    %% 子图5: 运动显著性直方图分布
    subplot(2, 3, 5);
    % 只考虑非零值的运动显著性
    nonzero_maxcj = G.maxcj(G.maxcj > 0);
    
    if ~isempty(nonzero_maxcj)
        histogram(nonzero_maxcj, 'Normalization', 'probability', 'FaceAlpha', 0.7);
        hold on;
        xline(G.cj_threshold, 'r--', 'LineWidth', 2, 'DisplayName', sprintf('阈值 = %.1f', G.cj_threshold));
        xlabel('运动显著性值');
        ylabel('概率密度');
        title('运动显著性分布');
        legend('分布', '阈值线', 'Location', 'best');
        grid on;
    else
        text(0.5, 0.5, '无有效数据', 'HorizontalAlignment', 'center', 'Units', 'normalized');
        title('运动显著性分布');
    end
    
    %% 子图6: 综合状态分析
    subplot(2, 3, 6);
    
    % 创建三条线：运动显著性（标准化）、激活数量、预警数量
    yyaxis left;
    % 标准化运动显著性到0-1范围用于显示
    if max(G.maxcj) > 0
        normalized_maxcj = G.maxcj(valid_steps) / max(G.maxcj);
        plot(valid_steps, normalized_maxcj, 'b-', 'LineWidth', 1.5, 'DisplayName', '运动显著性(标准化)');
    end
    ylabel('标准化运动显著性', 'Color', 'b');
    
    yyaxis right;
    hold on;
    if isfield(G, 'activatedCount')
        plot(valid_steps, G.activatedCount(valid_steps), 'g-', 'LineWidth', 1.5, 'DisplayName', '激活数量');
    end
    plot(valid_steps, G.warnNum(valid_steps), 'r-', 'LineWidth', 1.5, 'DisplayName', '预警数量');
    ylabel('个体数量', 'Color', 'k');
    
    xlabel('仿真步数');
    title('综合状态分析');
    legend('Location', 'best');
    grid on;
    
    %% 添加总体信息
    sgtitle(sprintf('运动显著性统计分析 - 标准算法\n(仿真步数: %d, 阈值: %.1f, 总个体: %d)', ...
        G.simStep, G.cj_threshold, G.maxID), 'FontSize', 14, 'FontWeight', 'bold');
    
    %% 计算并显示统计摘要
    fprintf('\n=== 运动显著性统计摘要 (标准算法) ===\n');
    fprintf('仿真步数: %d\n', G.simStep);
    fprintf('个体总数: %d\n', G.maxID);
    fprintf('运动显著性阈值: %.1f\n', G.cj_threshold);
    fprintf('运动显著性权重系数: %.1f\n', G.weight_cj);
    
    % 运动显著性统计
    valid_maxcj = G.maxcj(G.maxcj > 0);
    if ~isempty(valid_maxcj)
        fprintf('运动显著性范围: [%.3f, %.3f]\n', min(valid_maxcj), max(valid_maxcj));
        fprintf('运动显著性均值: %.3f\n', mean(valid_maxcj));
        fprintf('运动显著性标准差: %.3f\n', std(valid_maxcj));
        
        % 超阈值统计
        above_count = sum(G.maxcj > G.cj_threshold);
        above_ratio = above_count / G.simStep * 100;
        fprintf('超阈值步数: %d / %d (%.1f%%)\n', above_count, G.simStep, above_ratio);
    end
    
    % 激活统计
    if isfield(G, 'activatedCount')
        total_activations = sum(G.activatedCount);
        avg_activations = mean(G.activatedCount);
        fprintf('总激活次数: %d\n', total_activations);
        fprintf('平均激活个体数: %.2f\n', avg_activations);
    end
    
    % 预警统计
    total_warnings = sum(G.warnNum);
    avg_warnings = mean(G.warnNum);
    max_warnings = max(G.warnNum);
    fprintf('总预警次数: %d\n', total_warnings);
    fprintf('平均预警个体数: %.2f\n', avg_warnings);
    fprintf('最大同时预警数: %d\n', max_warnings);
    
    % 生存时间
    if isfield(G, 'sur_time')
        fprintf('群体生存时间: %d 步\n', G.sur_time);
    end
    
    fprintf('========================================\n\n');
end