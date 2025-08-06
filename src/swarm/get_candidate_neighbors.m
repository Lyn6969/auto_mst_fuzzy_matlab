function  cj = get_candidate_neighbors(focal_agent_id, neighbors, G)

    % 运动显著性计算的时间窗口因子
    motion_saliency_time_factor = 5;

    num_neis = numel(neighbors);
    cj = zeros * ones(num_neis, 1);
    
    % 获取当前代理和邻居的位置
    focal_agent = G.actor{focal_agent_id};
    my_pos = focal_agent.pose;
    count = 1;
    for j = 1:num_neis
        nei_pos = neighbors{j}.pose;
        current_diff = nei_pos - my_pos;

        current_diff = current_diff ./ vecnorm(current_diff, 2, 2);
        % disp(current_diff)
    
        if  ~isfield(G.actor{focal_agent_id}, 'memory')
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
        % disp(past_diff)

        angle_cos = max((min(dot(past_diff,current_diff,2), 1)), -1);

        % 计算当前和过去的方向差异
        cj(count) = (acos(angle_cos)/ (G.cycTime * motion_saliency_time_factor)) * G.weight_cj;
        count = count +1;
        
    end



end

    



        



