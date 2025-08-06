%---------------------- 机器人运动学模型 -------------------------
% 本版本根据实际轨迹数据进行了校正
% maxRotRate: 机器人可实施的最大角速度
function G = parallelSimRobots_v2(G,desTurnAngle,desSpeed)
    % 仿真中左转为+，右转为-；实际机器人右转为+，左转为-
    % 故先对期望转动角度求反
    desTurnAngle = - desTurnAngle;
    % 在固定更新周期cycTime内，完成转向的期望角速度为：
    rotRate = desTurnAngle / G.cycTime;
    % 限定个体的最大期望角速度，令其不超过maxRotRate
    indx = find(abs(rotRate) > G.maxRotRate);
    if ~isempty(indx)
        rotRate(indx) = sign(rotRate(indx)) .* G.maxRotRate;
    end
    
    % ------ 加入随机噪声 ------
    rotRate = rotRate + random('norm',-0.4,1.2,G.maxID,1);
    desSpeed = desSpeed + random('norm',0.34,0.9,G.maxID,1);
 
    %% ---------- 个体运动状态积分 ---------
    pos_x = []; 
    pos_y = [];
    dir = [];
    for i = 1:G.num
        pos_x(i) = G.actor{i}.pose(1);
        pos_y(i) = G.actor{i}.pose(2);
        heading(i) = vel2heading_deg(G.actor{i}.vel);
        heading(i) = heading(i) + rotRate(i)*G.cycTime;
        dir(i,:) = heading2unit_deg(heading(i));
        pos_x(i) = pos_x(i) + desSpeed(i) * cosd(heading(i))*G.cycTime;
        pos_y(i) = pos_y(i) + desSpeed(i) * sind(heading(i))*G.cycTime;
    end
    robotsPos(:,[1,2]) = real([pos_x',pos_y']);
    robotsPos(:,[3,4]) = real(dir);
    
    
    %% ------ 计算群体实时状态，重新组织数据存储结构 -------
    if G.simStep >=1
        % ---- 记录机器人的实时运动状态
        for i = 1:G.num
            G.actor{i}.id = i;
            G.actor{i}.pose = robotsPos(i,[1,2]);
            G.actor{i}.vel = unitvel(robotsPos(i,[3,4]));
            G.actor{i}.memory(G.simStep,:) = robotsPos(i,:);
        end
        % ---- 计算并记录群体的平均速度和方向
        aveDir = [0,0];
        aveSpeed = [0];
        for j = 1:G.num
        	aveDir = aveDir + unitvel(G.actor{j}.vel);
        	aveSpeed = aveSpeed + norm(G.actor{j}.vel);
        end
        aveDir = real(aveDir/G.num);
        G.groupDir([1,2],G.simStep) = unitvel(aveDir);
        G.groupHeading(1,G.simStep) = vel2heading_deg(aveDir);
        G.groupSpeed(1,G.simStep) = aveSpeed/G.num;
        % ---- 计算并记录op,order parameter
        temp = [0 0];
        for j = 1:G.num
        	temp = temp + unitvel(G.actor{j}.vel);
        end
        gop = ((temp(1)^2+temp(2)^2)^0.5) / G.num;      
        G.op(1,G.simStep) = real(gop);
        % ---- 计算并记录gam
        gam = 0;
        G.gam(1,G.simStep) = real(gam);
    end
end



