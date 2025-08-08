function activationAnalysis = analyzeActivationData(G)
    % 分析激活个体的详细数据
    % 输入: G - 仿真全局结构体
    % 输出: activationAnalysis - 包含详细分析结果的结构体
    
    fprintf('\n=== 激活数据分析开始 ===\n');
    
    % 基本信息
    activationAnalysis.basic_info.total_steps = G.simStep;
    activationAnalysis.basic_info.total_agents = G.maxID;
    activationAnalysis.basic_info.predator_id = G.hawkID;
    activationAnalysis.basic_info.threshold = G.cj_threshold;
    activationAnalysis.basic_info.weight_cj = G.weight_cj;
    
    % 创建时间序列数据
    valid_steps = 1:G.simStep;
    activationAnalysis.time_series.steps = valid_steps;
    activationAnalysis.time_series.activated_count = G.activatedCount(valid_steps);
    activationAnalysis.time_series.max_motion_saliency = G.maxcj(valid_steps);
    activationAnalysis.time_series.warning_count = G.warnNum(valid_steps);
    
    % 激活统计分析
    total_activations = sum(G.activatedCount(valid_steps));
    avg_activations = mean(G.activatedCount(valid_steps));
    max_activations = max(G.activatedCount(valid_steps));
    std_activations = std(G.activatedCount(valid_steps));
    
    activationAnalysis.activation_stats.total_activations = total_activations;
    activationAnalysis.activation_stats.average_per_step = avg_activations;
    activationAnalysis.activation_stats.max_simultaneous = max_activations;
    activationAnalysis.activation_stats.standard_deviation = std_activations;
    
    % 激活率分析
    steps_with_activation = sum(G.activatedCount(valid_steps) > 0);
    activation_rate = steps_with_activation / G.simStep * 100;
    activationAnalysis.activation_stats.steps_with_activation = steps_with_activation;
    activationAnalysis.activation_stats.activation_rate_percent = activation_rate;
    
    % 运动显著性超阈值分析
    above_threshold_steps = sum(G.maxcj(valid_steps) > G.cj_threshold);
    above_threshold_rate = above_threshold_steps / G.simStep * 100;
    activationAnalysis.motion_saliency_stats.above_threshold_steps = above_threshold_steps;
    activationAnalysis.motion_saliency_stats.above_threshold_rate_percent = above_threshold_rate;
    
    % 运动显著性统计
    valid_ms = G.maxcj(G.maxcj > 0);
    if ~isempty(valid_ms)
        activationAnalysis.motion_saliency_stats.min_value = min(valid_ms);
        activationAnalysis.motion_saliency_stats.max_value = max(valid_ms);
        activationAnalysis.motion_saliency_stats.mean_value = mean(valid_ms);
        activationAnalysis.motion_saliency_stats.std_value = std(valid_ms);
    else
        activationAnalysis.motion_saliency_stats.min_value = 0;
        activationAnalysis.motion_saliency_stats.max_value = 0;
        activationAnalysis.motion_saliency_stats.mean_value = 0;
        activationAnalysis.motion_saliency_stats.std_value = 0;
    end
    
    % 个体级别的激活分析
    prey_list = setdiff(1:G.maxID, G.hawkID);
    individual_activation_count = zeros(1, G.maxID);
    individual_activation_steps = cell(1, G.maxID);
    
    % 统计每个个体的激活次数和步骤
    for step = 1:G.simStep
        if ~isempty(G.activatedIDs{step})
            for agent_id = G.activatedIDs{step}
                individual_activation_count(agent_id) = individual_activation_count(agent_id) + 1;
                individual_activation_steps{agent_id} = [individual_activation_steps{agent_id}, step];
            end
        end
    end
    
    activationAnalysis.individual_stats.agent_ids = prey_list;
    activationAnalysis.individual_stats.activation_counts = individual_activation_count(prey_list);
    activationAnalysis.individual_stats.activation_steps = individual_activation_steps(prey_list);
    
    % 激活持续性分析
    activation_durations = [];
    for agent_id = prey_list
        if ~isempty(individual_activation_steps{agent_id})
            steps = individual_activation_steps{agent_id};
            % 找到连续激活的区间
            if length(steps) > 1
                duration_count = 1;
                for i = 2:length(steps)
                    if steps(i) == steps(i-1) + 1
                        duration_count = duration_count + 1;
                    else
                        activation_durations = [activation_durations, duration_count];
                        duration_count = 1;
                    end
                end
                activation_durations = [activation_durations, duration_count];
            else
                activation_durations = [activation_durations, 1];
            end
        end
    end
    
    if ~isempty(activation_durations)
        activationAnalysis.duration_stats.mean_duration = mean(activation_durations);
        activationAnalysis.duration_stats.max_duration = max(activation_durations);
        activationAnalysis.duration_stats.min_duration = min(activation_durations);
        activationAnalysis.duration_stats.std_duration = std(activation_durations);
        activationAnalysis.duration_stats.all_durations = activation_durations;
    else
        activationAnalysis.duration_stats.mean_duration = 0;
        activationAnalysis.duration_stats.max_duration = 0;
        activationAnalysis.duration_stats.min_duration = 0;
        activationAnalysis.duration_stats.std_duration = 0;
        activationAnalysis.duration_stats.all_durations = [];
    end
    
    % 时间段分析 (将仿真分成几个阶段)
    if G.simStep >= 3
        phase1_end = round(G.simStep * 0.33);
        phase2_end = round(G.simStep * 0.67);
        
        phase1_activations = sum(G.activatedCount(1:phase1_end));
        phase2_activations = sum(G.activatedCount(phase1_end+1:phase2_end));
        phase3_activations = sum(G.activatedCount(phase2_end+1:G.simStep));
        
        activationAnalysis.phase_analysis.phase1_steps = phase1_end;
        activationAnalysis.phase_analysis.phase2_steps = phase2_end - phase1_end;
        activationAnalysis.phase_analysis.phase3_steps = G.simStep - phase2_end;
        
        activationAnalysis.phase_analysis.phase1_activations = phase1_activations;
        activationAnalysis.phase_analysis.phase2_activations = phase2_activations;
        activationAnalysis.phase_analysis.phase3_activations = phase3_activations;
        
        activationAnalysis.phase_analysis.phase1_rate = phase1_activations / phase1_end;
        activationAnalysis.phase_analysis.phase2_rate = phase2_activations / (phase2_end - phase1_end);
        activationAnalysis.phase_analysis.phase3_rate = phase3_activations / (G.simStep - phase2_end);
    end
    
    % 输出分析摘要
    fprintf('总仿真步数: %d\n', G.simStep);
    fprintf('总激活次数: %d\n', total_activations);
    fprintf('平均每步激活数: %.3f\n', avg_activations);
    fprintf('最大同时激活数: %d\n', max_activations);
    fprintf('激活率: %.1f%% (%d/%d 步有激活)\n', activation_rate, steps_with_activation, G.simStep);
    fprintf('运动显著性超阈值率: %.1f%% (%d/%d 步)\n', above_threshold_rate, above_threshold_steps, G.simStep);
    
    if ~isempty(activation_durations)
        fprintf('平均激活持续时间: %.2f 步\n', activationAnalysis.duration_stats.mean_duration);
        fprintf('最长激活持续时间: %d 步\n', activationAnalysis.duration_stats.max_duration);
    end
    
    fprintf('=== 激活数据分析完成 ===\n\n');
end