function G = initSimRobots(G, pos_offset, r)
    % 初始化虚拟个体位置和状态
    
    % 设置机器人数量和编号列表
    G.num = G.maxID;
    G.robotsList = 1:G.maxID;
    robotsNum = G.maxID;

    % 生成不重叠的随机位置
    [rp, ~] = randPose(robotsNum);
    % 计算最终位置坐标
    robotsPos = [r * rp(1, :)' - pos_offset(1, 1), ...  % X坐标
                 r * rp(2, :)' - pos_offset(1, 2)];     % Y坐标

    % 获取捕食者位置
    predatorPos = G.actor{G.hawkID}.pose;
    % 计算所有个体到捕食者的距离
    distances = pdist2(robotsPos, predatorPos);
    % 找到距离捕食者最近的个体索引
    [~, idx] = min(distances);

    % 将2号个体与最近个体交换位置
    robotsPos([2, idx], :) = robotsPos([idx, 2], :);

    % 计算单位速度向量
    unit_velocity = unitvel([1, 1]);
    
    % 初始化每个个体的状态
    for i = 1:robotsNum
        G.actor{i}.id = i;                      % 个体编号
        G.actor{i}.pose = robotsPos(i, :);      % 个体位置
        G.actor{i}.vel = unit_velocity;         % 个体速度
        G.actor{i}.is_activated = false;        % 激活状态
        G.actor{i}.src_id = NaN;                % 信息源编号
        G.actor{i}.cj_threshold = G.cj_threshold;  % 初始化个体阈值
    end

    % 设置捕食者初始位置和速度
    G.actor{G.hawkID}.pose = [2000, 2000];
    G.actor{G.hawkID}.vel = [-1, -1];
end