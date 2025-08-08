%%修正版的逻辑
% 1.准备与防御性检查
%   在进行计算之前，确保焦点个体和邻居个体都存在，并且有足够的历史记录。
% 2.动态确定实际的时间间隔
%   它会取三个值中的最小值：tau_steps、邻居的历史长度和焦点个体的历史长度。 
%   当历史充足的时候，计算会使用完整的时间窗口。
%   当历史不足的时候，计算会缩小时间窗口，以适应历史数据的长度。
% 3.计算与分子匹配的精确时间间隔：一旦确定了实际能回溯的时间步数，就可以计算出实际的时间间隔（delta_t）。
% 4.修正索引并获取数据，修正了原始代码中的差1问题，确保索引正确。
%
% 2025.08.07 21:35


function cj = get_candidate_neighbors(focal_agent_id, neighbors, G)
    % 定义运动显著性计算的时间窗口（单位：时间步数）
    % 这就是公式中的 τ (tau)，但以步数表示
    tau_steps = 5; 

    num_neis = numel(neighbors);
    cj = zeros(num_neis, 1); % 标准的初始化方式

    % 获取焦点个体的信息
    if ~isfield(G.actor{focal_agent_id}, 'pose')
        return; % 如果焦点个体不存在，提前返回
    end
    focal_agent = G.actor{focal_agent_id};
    my_pos = focal_agent.pose;
    
    % 检查焦点个体是否有历史记录
    if ~isfield(focal_agent, 'memory') || isempty(focal_agent.memory)
        return; % 如果没有memory，无法计算，直接返回0
    end

    for j = 1:num_neis
        nei_pos = neighbors{j}.pose;

        % 1. 计算当前时刻的归一化相对位置向量 x_ij(t)
        current_diff = nei_pos - my_pos;
        % 如果距离为0，无法计算方向，可跳过或设为0
        if norm(current_diff) < 1e-6
            cj(j) = 0;
            continue;
        end
        current_diff_normalized = current_diff / norm(current_diff);

        % 检查邻居是否有历史记录
        if ~isfield(neighbors{j}, 'memory') || isempty(neighbors{j}.memory)
            cj(j) = 0; % 没有memory，无法计算
            continue;
        end

        history_length = size(neighbors{j}.memory, 1);
        my_history_length = size(focal_agent.memory, 1);
        
        % 2. 确定用于计算的过去时间点和实际时间间隔 (delta_t)
        actual_steps_back = min([tau_steps, history_length, my_history_length]);
        
        if actual_steps_back == 0
            cj(j) = 0;
            continue;
        end

        % 获取过去的位置
        my_past_pos = focal_agent.memory(end - actual_steps_back + 1, 1:2);
        nei_past_pos = neighbors{j}.memory(end - actual_steps_back + 1, 1:2);
        
        % 计算实际的时间间隔 (delta_t in seconds)
        delta_t = actual_steps_back * G.cycTime;

        % 3. 计算过去时刻的归一化相对位置向量 x_ij(t-τ)
        past_diff = nei_past_pos - my_past_pos;
        % 处理过去位置重合的特殊情况
        if norm(past_diff) < 1e-6
             cj(j) = 0; % 如果过去位置相同，认为没有角度变化
             continue;
        end
        past_diff_normalized = past_diff / norm(past_diff);

        % 4. 计算角度变化率
        % 鲁棒地计算cos值
        angle_cos = max(min(dot(past_diff_normalized, current_diff_normalized), 1), -1);
        
        % 角度（弧度） / 时间（秒）
        angular_velocity = acos(angle_cos) / delta_t;
        
        % 应用权重
        cj(j) = angular_velocity * G.weight_cj;
    end
end
