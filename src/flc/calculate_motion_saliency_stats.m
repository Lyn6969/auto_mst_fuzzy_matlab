function [avg_mj, var_mj] = calculate_motion_saliency_stats(focal_agent, neighbors, g_state)
    % 计算邻域运动显著性统计
    %
    % 输入参数:
    %   focal_agent: 焦点智能体结构体
    %   neighbors: 邻居结构体数组
    %   g_state: 全局状态结构体
    %
    % 输出:
    %   avg_mj: 邻域运动显著性均值
    %   var_mj: 邻域运动显著性方差

    if length(neighbors) == 0
        avg_mj = 0.0;
        var_mj = 0.0;
        return;
    end

    % 计算所有邻居的运动显著性
    cj_values = get_candidate_neighbors(focal_agent.id, neighbors, g_state);

    if length(cj_values) == 0
        avg_mj = 0.0;
        var_mj = 0.0;
        return;
    end

    % 计算均值和方差
    avg_mj = mean(cj_values);
    var_mj = var(cj_values);
end
