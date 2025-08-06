%-----chase-escape with FLC------，追逃场景 + 模糊逻辑控制器

function [desiredTurnAngle,desiredSpeed,G] = algo_mst_ce_flc(G)
    %----- 普通个体的机动过程 ---------
    % 在每次循环的末尾记录激活状态
    activated_this_step = [];  % 存储本步骤激活的个体ID
    activated_src_this_step = [];  % 存储本步骤激活的个体的源ID

    % 创建猎物列表（排除捕食者）
    preyList = setdiff(G.robotsList, G.hawkID);

    % 初始化FLC控制器（如果不存在）
    if ~isfield(G, 'flc_controller')
        G.flc_controller = FuzzyLogicController(0.0, 30.0);
        % 初始化FLC数据记录数组
        G.flc_data.thresholds = zeros(G.maxSimSteps, G.maxID);
        G.flc_data.p_i = zeros(G.maxSimSteps, G.maxID);
        G.flc_data.avg_mj = zeros(G.maxSimSteps, G.maxID);
        G.flc_data.var_mj = zeros(G.maxSimSteps, G.maxID);
        G.flc_data.delta_c = zeros(G.maxSimSteps, G.maxID);
        % 添加运动显著性范围数据
        G.flc_data.ms_min = zeros(G.maxSimSteps, G.maxID);  % 运动显著性最小值
        G.flc_data.ms_max = zeros(G.maxSimSteps, G.maxID);  % 运动显著性最大值
        G.flc_data.ms_range = zeros(G.maxSimSteps, G.maxID); % 运动显著性范围
        G.flc_data.ms_count = zeros(G.maxSimSteps, G.maxID); % 邻居数量
    end

    %% Self-Propeled: 个体自驱动方向
    i_vel = nan*zeros(G.maxID,2); % 个体自驱动方向
    % 只为猎物计算自驱动，捕食者有独立的行为逻辑
    for i = preyList
        % ---自驱作用力：维持原来运动方向的作用
        i_vel(i,[1,2]) = unitvel(G.actor{i}.vel);
    end
    
    %% AGGREGATION：个体速度协同
    dir_vel = nan*zeros(G.maxID,2); % 个体速度协同方向
    % 只为猎物计算速度协同，捕食者不参与群体协同行为
    for i = preyList
        focal_agent = G.actor{i};
        [neighbors, ~, neighbors_id_list] = get_topology_neighbors(G,focal_agent.id);
        
        % FLC自适应阈值更新
        if ~isempty(neighbors)
            % 计算运动显著性值并收集统计信息
            cj_values = get_candidate_neighbors(G.actor{i}.id, neighbors, G);
            if ~isempty(cj_values)
                G.flc_data.ms_min(G.simStep, i) = min(cj_values);
                G.flc_data.ms_max(G.simStep, i) = max(cj_values);
                G.flc_data.ms_range(G.simStep, i) = max(cj_values) - min(cj_values);
                G.flc_data.ms_count(G.simStep, i) = length(cj_values);
            end
            
            % 计算FLC输入变量
            p_i = calculate_local_polarization(G.actor{i}, neighbors);
            [avg_mj, var_mj] = calculate_motion_saliency_stats(G.actor{i}, neighbors, G);
            inputs = [p_i, avg_mj, var_mj];

            % 计算阈值变化量
            delta_c = G.flc_controller.calculate(inputs);

            % 应用Anti-Windup和更新阈值
            current_c = G.actor{i}.cj_threshold;

            % 调试输出（扩展到更多步数来观察5步后的情况）
            if G.simStep <=400 && i <= 3
                fprintf('Step %d, Agent %d: p_i=%.3f, avg_mj=%.3f, var_mj=%.3f -> delta_c=%.3f\n', ...
                    G.simStep, i, p_i, avg_mj, var_mj, delta_c);

                % 显示当前阈值和Anti-Windup状态
                fprintf('  当前阈值: %.3f, 新阈值: %.3f\n', current_c, current_c + delta_c);

                % 检查Anti-Windup是否被触发
                if (current_c >= G.flc_controller.c_max && delta_c > 0)
                    fprintf('  Anti-Windup: 上限饱和阻止增加\n');
                elseif (current_c <= G.flc_controller.c_min && delta_c < 0)
                    fprintf('  Anti-Windup: 下限饱和阻止减少\n');
                end

                % 显示运动显著性的详细信息
                if ~isempty(neighbors)
                    cj_values = get_candidate_neighbors(G.actor{i}.id, neighbors, G);
                    fprintf('  运动显著性值: [');
                    for k = 1:length(cj_values)
                        fprintf('%.3f ', cj_values(k));
                    end
                    fprintf(']\n');

                    % 测试模糊推理系统是否正常工作
                    if G.simStep == 6 && i == 1
                        fprintf('  === 测试模糊推理系统 ===\n');
                        test_inputs = [0.5, 10.0, 50.0];  % 中等值
                        test_output = G.flc_controller.calculate(test_inputs);
                        fprintf('  测试输入[0.5, 10.0, 50.0] -> 输出: %.3f\n', test_output);

                        test_inputs2 = [0.2, 5.0, 20.0];  % 低值
                        test_output2 = G.flc_controller.calculate(test_inputs2);
                        fprintf('  测试输入[0.2, 5.0, 20.0] -> 输出: %.3f\n', test_output2);
                    end
                end
            end

            % Enhanced Anti-Windup
            if (current_c >= G.flc_controller.c_max && delta_c > 0)
                delta_c = 0;
            elseif (current_c <= G.flc_controller.c_min && delta_c < 0)
                delta_c = 0;
            elseif current_c > (G.flc_controller.c_max - G.flc_controller.saturation_margin) && delta_c > 0
                scale_factor = (G.flc_controller.c_max - current_c) / G.flc_controller.saturation_margin;
                delta_c = delta_c * scale_factor;
            elseif current_c < (G.flc_controller.c_min + G.flc_controller.saturation_margin) && delta_c < 0
                scale_factor = (current_c - G.flc_controller.c_min) / G.flc_controller.saturation_margin;
                delta_c = delta_c * scale_factor;
            end

            % 更新阈值
            new_c = current_c + delta_c;
            G.actor{i}.cj_threshold = max(G.flc_controller.c_min, min(new_c, G.flc_controller.c_max));

            % 记录FLC数据
            G.flc_data.thresholds(G.simStep, i) = G.actor{i}.cj_threshold;
            G.flc_data.p_i(G.simStep, i) = p_i;
            G.flc_data.avg_mj(G.simStep, i) = avg_mj;
            G.flc_data.var_mj(G.simStep, i) = var_mj;
            G.flc_data.delta_c(G.simStep, i) = delta_c;
        else
            % 没有邻居时记录当前阈值和运动显著性数据
            G.flc_data.thresholds(G.simStep, i) = G.actor{i}.cj_threshold;
            G.flc_data.p_i(G.simStep, i) = 0;
            G.flc_data.avg_mj(G.simStep, i) = 0;
            G.flc_data.var_mj(G.simStep, i) = 0;
            G.flc_data.delta_c(G.simStep, i) = 0;
            G.flc_data.ms_min(G.simStep, i) = 0;
            G.flc_data.ms_max(G.simStep, i) = 0;
            G.flc_data.ms_range(G.simStep, i) = 0;
            G.flc_data.ms_count(G.simStep, i) = 0;
        end
        
        if ~isempty(neighbors)
            if focal_agent.is_activated
                src_id = focal_agent.src_id;
                % 检查源个体是否仍然存在且不是捕食者
                if src_id ~= G.hawkID && ~isnan(src_id)
                    src_agent = G.actor{src_id};
                    if ( vecnorm(src_agent.vel - focal_agent.vel) < G.deac_threshold)
                        % 取消激活，并且不再跟随，平均速度
                        G.actor{i}.is_activated = false;
                        G.actor{i}.src_id = NaN;
                        temp_vel = [0,0];
                        for j = neighbors_id_list
                            temp_vel = temp_vel + unitvel(G.actor{j}.vel);
                        end
                        dir_vel(i,[1,2]) = unitvel(temp_vel);
                    else
                        % 激活状态，跟随源头
                        dir_vel(i,[1,2]) = unitvel(src_agent.vel);

                        if isfield(G,"now_infoID")
                            if src_agent.id == G.now_infoID
                               disp("信息个体激活");
                            end
                        end
                    end
                else
                    % 源个体无效（可能是捕食者或已被移除），取消激活
                    G.actor{i}.is_activated = false;
                    G.actor{i}.src_id = NaN;
                    temp_vel = [0,0];
                    for j = neighbors_id_list
                        temp_vel = temp_vel + unitvel(G.actor{j}.vel);
                    end
                    dir_vel(i,[1,2]) = unitvel(temp_vel);
                end
            else
                cj = get_candidate_neighbors(focal_agent.id, neighbors, G);
                if ~isempty(cj) && (max(cj) > G.actor{i}.cj_threshold)  % 使用自适应阈值
                    % 激活状态
                    G.actor{i}.is_activated = true;
                    G.maxcj(G.simStep) = max(cj);
                    max_cj_index = cj == max(cj);
                    src_agent = neighbors{max_cj_index};
                    G.actor{i}.src_id = src_agent.id;
                    dir_vel(i,[1,2]) = unitvel(src_agent.vel);
                else
                    % 不激活状态，平均速度
                    temp_vel = [0,0];
                    for j = neighbors_id_list
                        temp_vel = temp_vel + unitvel(G.actor{j}.vel);
                    end
                    dir_vel(i,[1,2]) = unitvel(temp_vel);
                end
            end
        end
        if G.actor{i}.is_activated
            activated_this_step = [activated_this_step, i];
            activated_src_this_step = [activated_src_this_step, G.actor{i}.src_id];
        end
    end
    % 保存本步骤的激活信息
    G.activatedIDs{G.simStep} = activated_this_step;
    G.activatedCount(G.simStep) = numel(activated_this_step);
    G.activatedSrcIDs{G.simStep} = activated_src_this_step;
    
    %% AGGREGATION：个体位置协同
    dir_pos = nan*zeros(G.maxID, 2);
    % 只为猎物计算位置协同，捕食者不参与群体内聚/排斥行为
    for i = preyList
        % 考虑所有其他猎物的内聚/排斥作用（猎物之间的相互作用）
        % 注意：捕食者不参与猎物间的内聚/排斥计算
        neig_pos = setdiff(preyList, i);  % 只考虑其他猎物，排除捕食者
        if ~isempty(neig_pos)
            temp_pos = [0, 0];
            for j = neig_pos
                rij = G.actor{j}.pose - G.actor{i}.pose;
                dij = norm(rij);
                % 避免个体在完全相同位置时除以零
                if dij == 0
                    nij = [0, 0];
                    ra = 10000;  % 强排斥力
                else
                    nij = rij / dij;
                    if dij <= G.Drep
                        ra = G.weight_rep * (dij / G.Drep - 1);
                    elseif dij <= G.Dsen
                        % 修正吸引力计算公式，确保参数有效性
                        denominator = G.Dsen - G.Drep;
                        if denominator <= 0
                            ra = 0;
                        else
                            ra = G.weight_att * (dij - G.Drep) / denominator;
                        end
                    else
                        ra = 0;  % 超出感知范围则没有吸引力
                    end
                end
                temp_pos = temp_pos + ra * nij;
            end
            dir_pos(i, [1, 2]) = unitvel(temp_pos);
        end
    end
    
    %% 远离attacker
    dir_esc = zeros(G.maxID,2);
    warnIDs = [];
    hawk_pos = G.actor{G.hawkID}.pose;
    % 只有猎物需要逃离捕食者（这部分逻辑已经正确）
    for i = preyList  % 使用preyList确保一致性
        vec2hawk = hawk_pos - G.actor{i}.pose;	% prey指向predator的向量
        dir2hawk = unitvel(vec2hawk);           % prey到predator的方向和距离
        dist2hawk = norm(vec2hawk);
        if dist2hawk <= G.R_escape(i)           % 如prey发现危险，标志为 1，反向逃跑
            warnIDs = [warnIDs,i];
            if dist2hawk > 0  % 避免攻击者与猎物位置重合时除以零
                dir_esc(i,[1,2]) = - dir2hawk;
            else
                % 如果距离为零，选择随机逃逸方向
                dir_esc(i,[1,2]) = unitvel(rand(1,2) - 0.5);
            end
        end
    end
    G.warnIDs{G.simStep} = warnIDs;
    G.warnNum(G.simStep) = length(warnIDs);

    %% 行为综合：位置协同&速度协同&避险机动-----
    % 位置协同 + 速度协同 
    temp_socialDir = dir_pos + G.weight_align*dir_vel;
    [unit_socialDir,~] = unitVector(temp_socialDir');
    desDir = unit_socialDir';

    % 避险机动
    if ~isempty(warnIDs)
        desDir(warnIDs,:) = G.weight_esc*dir_esc(warnIDs,:); % 个体避险机动，逃离hawk，没有位置协同和速度协同
    end
    desSpeed = G.v0 + zeros(G.maxID,1);

    %% ------------attacker的运动过程-------------
    % 未发起攻击时，attacker 默认处于静止状态
    desDir(G.hawkID,[1,2]) = [-1,0];
    desSpeed(G.hawkID,1) = 0;
    % attacker发起攻击后，追击距离最近的目标
    if G.simStep >= G.attackStep
        % predator的运动规则
        pos_hawk = G.actor{G.hawkID}.pose;

        % 只考虑猎物，使用向量化计算提高效率
        if ~isempty(preyList)
            prey_poses = zeros(length(preyList), 2);
            for idx = 1:length(preyList)
                prey_poses(idx, :) = G.actor{preyList(idx)}.pose;
            end

            % 向量化计算距离
            vecs_hawk_to_prey = prey_poses - pos_hawk;
            dists_to_prey = sqrt(sum(vecs_hawk_to_prey.^2, 2));

            [target_dist, min_idx] = min(dists_to_prey);
            G.target_dist(G.simStep) = target_dist;

            % 朝向最近猎物的方向
            if target_dist > 0
                desDir(G.hawkID,[1,2]) = unitvel(vecs_hawk_to_prey(min_idx, :));
            else
                % 攻击者在猎物正上方，保持当前方向或默认方向
                current_vel = G.actor{G.hawkID}.vel;
                if norm(current_vel) > 0
                    desDir(G.hawkID,[1,2]) = unitvel(current_vel);
                else
                    desDir(G.hawkID,[1,2]) = [1, 0];
                end
            end
            desSpeed(G.hawkID,1) = G.v0_hawk;
        else
            % 没有猎物了
            G.target_dist(G.simStep) = NaN;
        end
    end

    %----- 转换为t+1期望转动角度 -----
    desTurnAngle = zeros(G.maxID,1);
    for i = G.robotsList
        iDir = unitvel(G.actor{i}.vel);
        desTurnAngle(i,1) = angleOfVectors(iDir',desDir(i,:)');   % 右转+，左转-
        % 保存个体的期望运动状态
        G.actor{i}.desiredTurnAngle(G.simStep,1) = desTurnAngle(i,1);
        G.actor{i}.desiredSpeed(G.simStep,1) = desSpeed(i,1);
    end
    % 返回：期望转角 & 期望角速度
    desiredTurnAngle = desTurnAngle;
    desiredSpeed = desSpeed;  
end
