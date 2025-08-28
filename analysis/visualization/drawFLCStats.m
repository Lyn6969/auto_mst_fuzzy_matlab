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
    
    % 保存FLC统计数据为CSV文件
    save_flc_data_to_csv(G, valid_steps, preyList);
end

function save_flc_data_to_csv(G, valid_steps, preyList)
    % 保存FLC相关数据到CSV文件
    
    % 创建保存目录
    csv_dir = './Data_Sim/csv_exports/';
    if ~exist(csv_dir, 'dir')
        mkdir(csv_dir);
    end
    
    % 生成带时间戳的文件名
    timeStamp = datetime('now','Format','yyyyMMdd''T''HHmmss');
    base_filename = sprintf('FLC_Stats_%s', string(timeStamp));
    
    % 1. 保存整体统计数据（时间序列）
    overall_data = [];
    overall_data(:,1) = valid_steps';  % 时间步
    overall_data(:,2) = mean(G.flc_data.thresholds(valid_steps, preyList), 2);  % 平均阈值
    overall_data(:,3) = mean(G.flc_data.p_i(valid_steps, preyList), 2);        % 平均局部有序度
    overall_data(:,4) = mean(G.flc_data.avg_mj(valid_steps, preyList), 2);     % 平均运动显著性均值
    overall_data(:,5) = mean(G.flc_data.var_mj(valid_steps, preyList), 2);     % 平均运动显著性方差
    overall_data(:,6) = mean(G.flc_data.delta_c(valid_steps, preyList), 2);    % 平均阈值变化量
    
    % 添加激活个体数量（如果存在）
    if isfield(G, 'activatedCount') && length(G.activatedCount) >= max(valid_steps)
        overall_data(:,7) = G.activatedCount(valid_steps)';
    else
        overall_data(:,7) = zeros(length(valid_steps), 1);
    end
    
    % 写入整体统计CSV
    overall_filename = [csv_dir base_filename '_overall.csv'];
    overall_headers = {'TimeStep', 'MeanThreshold', 'MeanLocalPolarization', ...
                      'MeanMotionSaliencyAvg', 'MeanMotionSaliencyVar', ...
                      'MeanDeltaC', 'ActivatedCount'};
    
    writecell([overall_headers; num2cell(overall_data)], overall_filename);
    fprintf('已保存整体FLC统计数据到: %s\n', overall_filename);
    
    % 2. 保存每个个体的详细数据（仅保存关键时间点以控制文件大小）
    sample_interval = max(1, floor(G.simStep / 100));  % 最多采样100个时间点
    sample_steps = 1:sample_interval:G.simStep;
    
    individual_data = [];
    row_idx = 1;
    
    for t_idx = 1:length(sample_steps)
        t = sample_steps(t_idx);
        for robot_idx = 1:length(preyList)
            robot_id = preyList(robot_idx);
            
            individual_data(row_idx, 1) = t;                                    % 时间步
            individual_data(row_idx, 2) = robot_id;                             % 机器人ID
            individual_data(row_idx, 3) = G.flc_data.thresholds(t, robot_id);   % 阈值
            individual_data(row_idx, 4) = G.flc_data.p_i(t, robot_id);          % 局部有序度
            individual_data(row_idx, 5) = G.flc_data.avg_mj(t, robot_id);       % 运动显著性均值
            individual_data(row_idx, 6) = G.flc_data.var_mj(t, robot_id);       % 运动显著性方差
            individual_data(row_idx, 7) = G.flc_data.delta_c(t, robot_id);      % 阈值变化量
            individual_data(row_idx, 8) = G.flc_data.ms_min(t, robot_id);       % 运动显著性最小值
            individual_data(row_idx, 9) = G.flc_data.ms_max(t, robot_id);       % 运动显著性最大值
            individual_data(row_idx, 10) = G.flc_data.ms_range(t, robot_id);    % 运动显著性范围
            individual_data(row_idx, 11) = G.flc_data.ms_count(t, robot_id);    % 邻居数量
            
            row_idx = row_idx + 1;
        end
    end
    
    % 写入个体详细数据CSV
    individual_filename = [csv_dir base_filename '_individual.csv'];
    individual_headers = {'TimeStep', 'RobotID', 'Threshold', 'LocalPolarization', ...
                         'MotionSaliencyAvg', 'MotionSaliencyVar', 'DeltaC', ...
                         'MSMin', 'MSMax', 'MSRange', 'NeighborCount'};
    
    writecell([individual_headers; num2cell(individual_data)], individual_filename);
    fprintf('已保存个体详细FLC数据到: %s\n', individual_filename);
    
    % 3. 保存实验参数摘要
    params_filename = [csv_dir base_filename '_parameters.csv'];
    params_data = {
        'Parameter', 'Value';
        'SimulationSteps', G.simStep;
        'RobotCount', length(preyList);
        'InitialThreshold', G.cj_threshold;
        'WeightAlign', G.weight_align;
        'WeightRep', G.weight_rep;
        'WeightAtt', G.weight_att;
        'WeightCJ', G.weight_cj;
        'DeactivationThreshold', G.deac_threshold;
        'MaxNeighbors', G.max_neighbors;
        'WeightEscape', G.weight_esc;
        'HawkID', G.hawkID;
        'DeadRadius', G.R_dead;
        'HawkSpeed', G.v0_hawk;
        'CycleTime', G.cycTime;
        'RobotSpeed', G.v0;
        'MaxRotationRate', G.maxRotRate;
        'SenseRadius', G.r_sense
    };
    
    writecell(params_data, params_filename);
    fprintf('已保存实验参数到: %s\n', params_filename);
    
    fprintf('\n所有FLC数据已成功导出到CSV格式！\n');
end
