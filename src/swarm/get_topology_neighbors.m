function [neighbors, num_neighbors, neighbors_id_list] = get_topology_neighbors(G,focal_agent_id)
        % 初始化过滤后的代理数组
        allAgents = G.actor;
        filteredAgents = cell(1, numel(G.robotsList)- 1);
        count = 0;
        
        % 过滤掉给定的代理
        for i = G.robotsList
            if allAgents{i}.id ~= focal_agent_id && allAgents{i}.id ~= G.hawkID
                count = count + 1;
                filteredAgents{count} = allAgents{i};
            end
        end
        
        % 计算每个代理到焦点代理的距离
        distances = zeros(1, count);
        for i = 1:count
            distances(i) = norm(filteredAgents{i}.pose - allAgents{focal_agent_id}.pose);
        end
        
        % 根据距离排序
        [~, sortIndex] = sort(distances);
        sortedAgents = filteredAgents(sortIndex);
        
        % 获取最近的最大邻居数目的代理
        maxIndex = min(G.max_neighbors, length(sortedAgents));
        neighbors = sortedAgents(1:maxIndex);
        
        % 计算邻居数和邻居ID列表
        num_neighbors = length(neighbors);
        neighbors_id_list = zeros(1,num_neighbors );
        for i = 1:numel(neighbors)
            neighbors_id_list(i) = neighbors{i}.id;
        end

end