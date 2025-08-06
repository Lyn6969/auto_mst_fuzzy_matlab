function p_i = calculate_local_polarization(focal_agent, neighbors)
    % 计算局部有序度 p_i
    %
    % 输入参数:
    %   focal_agent: 焦点智能体结构体
    %   neighbors: 邻居结构体数组
    %
    % 输出:
    %   p_i: 局部有序度 [0, 1]

    if length(neighbors) < 1
        p_i = 0.0;
        return;
    end

    % 计算所有邻居的速度单位向量
    unit_velocities = [];
    for i = 1:length(neighbors)
        vel_norm = norm(neighbors{i}.vel);
        if vel_norm > 1e-6  % 避免除零
            unit_velocities = [unit_velocities; neighbors{i}.vel / vel_norm];
        else
            unit_velocities = [unit_velocities; [0.0, 0.0]];
        end
    end

    % 计算单位向量的平均值
    if isempty(unit_velocities)
        p_i = 0.0;
        return;
    end

    sum_unit_vel = sum(unit_velocities, 1);
    polarization = norm(sum_unit_vel) / size(unit_velocities, 1);

    p_i = max(0.0, min(polarization, 1.0));  % 等价于np.clip
end
