%-----chase-escape------，追逃场景

function [desiredTurnAngle,desiredSpeed,G] = algo_mst_ce(G)
    %----- 普通个体的机动过程 ---------
    % 在每次循环的末尾记录激活状态
    activated_this_step = [];  % 存储本步骤激活的个体ID
    activated_src_this_step = [];  % 存储本步骤激活的个体的源ID

    % 创建猎物列表（排除捕食者）
    preyList = setdiff(G.robotsList, G.hawkID);

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
                if ~isempty(cj) && (max(cj) > G.cj_threshold)
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
        % 
        % if G.simStep <= 80 && i ~=G.hawkID
        %     desDir(i,[1,2]) = [1,1];
        % end
        desTurnAngle(i,1) = angleOfVectors(iDir',desDir(i,:)');   % 右转+，左转-
        % 保存个体的期望运动状态
        G.actor{i}.desiredTurnAngle(G.simStep,1) = desTurnAngle(i,1);
        G.actor{i}.desiredSpeed(G.simStep,1) = desSpeed(i,1);
    end
    % 返回：期望转角 & 期望角速度
    desiredTurnAngle = desTurnAngle;
    desiredSpeed = desSpeed;  
end