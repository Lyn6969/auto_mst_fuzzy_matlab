function saveActivationAnalysisToCSV(activationAnalysis, csvFileName)
    % 将激活分析结果保存为CSV格式，便于后续分析
    % 输入: 
    %   activationAnalysis - 分析结果结构体
    %   csvFileName - 保存的CSV文件名
    
    fprintf('保存激活分析数据到CSV文件: %s\n', csvFileName);
    
    try
        % 创建时间序列数据表
        timeSeriesTable = table();
        
        % 确保所有数组都是列向量（不需要转置）
        steps_col = activationAnalysis.time_series.steps(:);
        activated_count_col = activationAnalysis.time_series.activated_count(:);  
        max_motion_saliency_col = activationAnalysis.time_series.max_motion_saliency(:);
        warning_count_col = activationAnalysis.time_series.warning_count(:);
        
        timeSeriesTable.Step = steps_col;
        timeSeriesTable.ActivatedCount = activated_count_col;
        timeSeriesTable.MaxMotionSaliency = max_motion_saliency_col;
        timeSeriesTable.WarningCount = warning_count_col;
        
        % 添加派生指标
        timeSeriesTable.IsAboveThreshold = double(timeSeriesTable.MaxMotionSaliency > activationAnalysis.basic_info.threshold);
        timeSeriesTable.HasActivation = double(timeSeriesTable.ActivatedCount > 0);
        
        % 保存时间序列数据
        writetable(timeSeriesTable, csvFileName);
        
        % 创建摘要统计文件
        [pathstr, name, ext] = fileparts(char(csvFileName));
        summaryFileName = fullfile(pathstr, [name '_Summary.txt']);
        
        % 确保目录存在
        if ~exist(pathstr, 'dir')
            mkdir(pathstr);
        end
        
        % 保存摘要统计到文本文件
        fid = fopen(summaryFileName, 'w');
        if fid == -1
            % 如果失败，尝试使用绝对路径
            abs_summaryFileName = fullfile(pwd, summaryFileName);
            fid = fopen(abs_summaryFileName, 'w');
            if fid == -1
                error('无法创建摘要文件: %s', summaryFileName);
            end
        end
        
        fprintf(fid, '=== 激活数据分析摘要 ===\n');
        fprintf(fid, '生成时间: %s\n', datestr(now));
        fprintf(fid, '\n--- 基本信息 ---\n');
        fprintf(fid, '总仿真步数: %d\n', activationAnalysis.basic_info.total_steps);
        fprintf(fid, '总个体数: %d\n', activationAnalysis.basic_info.total_agents);
        fprintf(fid, '捕食者ID: %d\n', activationAnalysis.basic_info.predator_id);
        fprintf(fid, '运动显著性阈值: %.6f\n', activationAnalysis.basic_info.threshold);
        fprintf(fid, '运动显著性权重: %.1f\n', activationAnalysis.basic_info.weight_cj);
        
        fprintf(fid, '\n--- 激活统计 ---\n');
        fprintf(fid, '总激活次数: %d\n', activationAnalysis.activation_stats.total_activations);
        fprintf(fid, '平均每步激活数: %.3f\n', activationAnalysis.activation_stats.average_per_step);
        fprintf(fid, '最大同时激活数: %d\n', activationAnalysis.activation_stats.max_simultaneous);
        fprintf(fid, '激活数标准差: %.3f\n', activationAnalysis.activation_stats.standard_deviation);
        fprintf(fid, '有激活的步数: %d\n', activationAnalysis.activation_stats.steps_with_activation);
        fprintf(fid, '激活率: %.2f%%\n', activationAnalysis.activation_stats.activation_rate_percent);
        
        fprintf(fid, '\n--- 运动显著性统计 ---\n');
        fprintf(fid, '超阈值步数: %d\n', activationAnalysis.motion_saliency_stats.above_threshold_steps);
        fprintf(fid, '超阈值率: %.2f%%\n', activationAnalysis.motion_saliency_stats.above_threshold_rate_percent);
        fprintf(fid, '运动显著性最小值: %.6f\n', activationAnalysis.motion_saliency_stats.min_value);
        fprintf(fid, '运动显著性最大值: %.6f\n', activationAnalysis.motion_saliency_stats.max_value);
        fprintf(fid, '运动显著性均值: %.6f\n', activationAnalysis.motion_saliency_stats.mean_value);
        fprintf(fid, '运动显著性标准差: %.6f\n', activationAnalysis.motion_saliency_stats.std_value);
        
        % 个体统计
        fprintf(fid, '\n--- 个体激活统计 ---\n');
        fprintf(fid, '个体ID, 激活次数\n');
        for i = 1:length(activationAnalysis.individual_stats.agent_ids)
            agent_id = activationAnalysis.individual_stats.agent_ids(i);
            count = activationAnalysis.individual_stats.activation_counts(i);
            fprintf(fid, '%d, %d\n', agent_id, count);
        end
        
        % 持续时间统计
        if isfield(activationAnalysis, 'duration_stats') && ~isempty(activationAnalysis.duration_stats.all_durations)
            fprintf(fid, '\n--- 激活持续时间统计 ---\n');
            fprintf(fid, '平均持续时间: %.2f 步\n', activationAnalysis.duration_stats.mean_duration);
            fprintf(fid, '最长持续时间: %d 步\n', activationAnalysis.duration_stats.max_duration);
            fprintf(fid, '最短持续时间: %d 步\n', activationAnalysis.duration_stats.min_duration);
            fprintf(fid, '持续时间标准差: %.2f 步\n', activationAnalysis.duration_stats.std_duration);
        end
        
        % 阶段分析
        if isfield(activationAnalysis, 'phase_analysis')
            fprintf(fid, '\n--- 阶段分析 (三等分) ---\n');
            fprintf(fid, '第一阶段 (1-%d步): %d次激活, 激活率=%.3f\n', ...
                activationAnalysis.phase_analysis.phase1_steps, ...
                activationAnalysis.phase_analysis.phase1_activations, ...
                activationAnalysis.phase_analysis.phase1_rate);
            fprintf(fid, '第二阶段 (%d-%d步): %d次激活, 激活率=%.3f\n', ...
                activationAnalysis.phase_analysis.phase1_steps+1, ...
                activationAnalysis.phase_analysis.phase1_steps + activationAnalysis.phase_analysis.phase2_steps, ...
                activationAnalysis.phase_analysis.phase2_activations, ...
                activationAnalysis.phase_analysis.phase2_rate);
            fprintf(fid, '第三阶段 (%d-%d步): %d次激活, 激活率=%.3f\n', ...
                activationAnalysis.phase_analysis.phase1_steps + activationAnalysis.phase_analysis.phase2_steps + 1, ...
                activationAnalysis.basic_info.total_steps, ...
                activationAnalysis.phase_analysis.phase3_activations, ...
                activationAnalysis.phase_analysis.phase3_rate);
        end
        
        fprintf(fid, '\n=== 分析完成 ===\n');
        fclose(fid);
        
        fprintf('CSV数据保存成功: %s\n', csvFileName);
        fprintf('摘要文件保存成功: %s\n', summaryFileName);
        
        % 显示数据预览
        fprintf('\n--- 时间序列数据预览 (前10行) ---\n');
        if height(timeSeriesTable) > 0
            disp(timeSeriesTable(1:min(10, height(timeSeriesTable)), :));
            if height(timeSeriesTable) > 10
                fprintf('... (共 %d 行数据)\n', height(timeSeriesTable));
            end
        end
        
    catch ME
        fprintf('保存CSV文件时出错: %s\n', ME.message);
        rethrow(ME);
    end
end