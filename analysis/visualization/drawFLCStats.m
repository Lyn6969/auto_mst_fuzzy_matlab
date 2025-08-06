function drawFLCStats(G)
    % 绘制FLC相关统计数据
    % 输入: G - 包含FLC数据的全局结构体
    
    % 创建新的图形窗口
    figure('Position', [100, 100, 1600, 1200], 'Name', 'FLC Statistics');
    
    % 获取有效的仿真步数
    valid_steps = 1:G.simStep;
    
    % 创建猎物列表（排除捕食者）
    preyList = setdiff(G.robotsList, G.hawkID);
    
    %% 子图1: 所有个体的阈值变化曲线
    subplot(3, 3, 1);
    hold on;
    for i = preyList
        plot(valid_steps, G.flc_data.thresholds(valid_steps, i), 'LineWidth', 1);
    end
    % 绘制平均阈值
    mean_threshold = mean(G.flc_data.thresholds(valid_steps, preyList), 2);
    plot(valid_steps, mean_threshold, 'k-', 'LineWidth', 2, 'DisplayName', 'Mean');
    
    xlabel('仿真步数');
    ylabel('阈值 C_i');
    title('个体阈值变化曲线');
    grid on;
    
    %% 子图2: 局部有序度p_i的时间序列
    subplot(3, 3, 2);
    hold on;
    for i = preyList
        plot(valid_steps, G.flc_data.p_i(valid_steps, i), 'LineWidth', 0.8);
    end
    % 绘制平均值
    mean_p_i = mean(G.flc_data.p_i(valid_steps, preyList), 2);
    plot(valid_steps, mean_p_i, 'k-', 'LineWidth', 2, 'DisplayName', 'Mean');
    
    xlabel('仿真步数');
    ylabel('局部有序度 p_i');
    title('局部有序度时间序列');
    grid on;
    ylim([0, 1]);
    
    %% 子图3: 邻域运动显著性均值的时间序列
    subplot(3, 3, 3);
    hold on;
    for i = preyList
        plot(valid_steps, G.flc_data.avg_mj(valid_steps, i), 'LineWidth', 0.8);
    end
    % 绘制平均值
    mean_avg_mj = mean(G.flc_data.avg_mj(valid_steps, preyList), 2);
    plot(valid_steps, mean_avg_mj, 'k-', 'LineWidth', 2, 'DisplayName', 'Mean');
    
    xlabel('仿真步数');
    ylabel('邻域运动显著性均值 avg(M_j)');
    title('邻域运动显著性均值时间序列');
    grid on;
    
    %% 子图4: 邻域运动显著性方差的时间序列
    subplot(3, 3, 4);
    hold on;
    for i = preyList
        plot(valid_steps, G.flc_data.var_mj(valid_steps, i), 'LineWidth', 0.8);
    end
    % 绘制平均值
    mean_var_mj = mean(G.flc_data.var_mj(valid_steps, preyList), 2);
    plot(valid_steps, mean_var_mj, 'k-', 'LineWidth', 2, 'DisplayName', 'Mean');
    
    xlabel('仿真步数');
    ylabel('邻域运动显著性方差 var(M_j)');
    title('邻域运动显著性方差时间序列');
    grid on;
    
    %% 子图5: 阈值变化量delta_C的时间序列
    subplot(3, 3, 5);
    hold on;
    for i = preyList
        plot(valid_steps, G.flc_data.delta_c(valid_steps, i), 'LineWidth', 0.8);
    end
    % 绘制平均值
    mean_delta_c = mean(G.flc_data.delta_c(valid_steps, preyList), 2);
    plot(valid_steps, mean_delta_c, 'k-', 'LineWidth', 2, 'DisplayName', 'Mean');
    
    xlabel('仿真步数');
    ylabel('阈值变化量 ΔC');
    title('阈值变化量时间序列');
    grid on;
    ylim([-5, 5]);
    
    %% 子图6: 激活个体数量与平均阈值的关系
    subplot(3, 3, 6);
    
    % 计算每个时间步的激活个体数量和平均阈值
    activation_count = zeros(length(valid_steps), 1);
    avg_threshold = zeros(length(valid_steps), 1);
    
    for t = 1:length(valid_steps)
        step = valid_steps(t);
        if step <= length(G.activatedCount)
            activation_count(t) = G.activatedCount(step);
        end
        avg_threshold(t) = mean(G.flc_data.thresholds(step, preyList));
    end
    
    % 创建双y轴图
    yyaxis left;
    plot(valid_steps, activation_count, 'b-', 'LineWidth', 2);
    ylabel('激活个体数量', 'Color', 'b');
    ylim([0, max(activation_count) + 1]);
    
    yyaxis right;
    plot(valid_steps, avg_threshold, 'r-', 'LineWidth', 2);
    ylabel('平均阈值', 'Color', 'r');
    
    xlabel('仿真步数');
    title('激活个体数量与平均阈值');
    grid on;
    
    %% 添加总体信息文本
    sgtitle(sprintf('FLC自适应阈值控制统计 (仿真步数: %d, 个体数量: %d)', ...
        G.simStep, length(preyList)), 'FontSize', 14, 'FontWeight', 'bold');
    
    % 计算并显示统计摘要
    fprintf('\n=== FLC统计摘要 ===\n');
    fprintf('仿真步数: %d\n', G.simStep);
    fprintf('个体数量: %d\n', length(preyList));
    fprintf('平均阈值范围: [%.2f, %.2f]\n', min(mean_threshold), max(mean_threshold));
    fprintf('平均局部有序度: %.3f\n', mean(mean_p_i));
    fprintf('平均运动显著性均值: %.3f\n', mean(mean_avg_mj));
    fprintf('平均运动显著性方差: %.3f\n', mean(mean_var_mj));
    fprintf('平均阈值变化量: %.3f\n', mean(abs(mean_delta_c)));
    fprintf('总激活次数: %d\n', sum(activation_count));
    fprintf('==================\n\n');
end
