% ------- 代码运行一下，让个体的位置不重复-------号
function [pos,vel] = randpose_unif(Nc)
    t_max = 10;               % 最大仿真步数
    dt = 0.1;                  % 仿真步长（s）   
    la = 3; lc = 10;            % repution & attraction related factors
    max_force = 5;             % maximum acceleration              
    v_0 = 0; relaxTime = 0.1;   % self-propulsion related factors
    noiseStr = 0.5;             % noise strength

    % initial pose & velocity of individuals
    pos = rand(2,Nc)*2;        % 初始位置随机分布
    vel = ones(2,Nc);      % 初始速度~0，基本静止

    for t = 1 : t_max
        % auto force
        u_auto = zeros(2,Nc);
        irate = (vel(1,:).^2+vel(2,:).^2).^(0.5);
        u_auto = repmat((v_0-irate)/relaxTime,2,1) .* (vel./repmat(irate,2,1));
        % attraction/repulation force
        u_grad = zeros(2,Nc);
        for i = 1:Nc
            rij = pos - repmat(pos(:,i),1,Nc);
            dij = (rij(1,:).^2 + rij(2,:).^2).^(0.5);
            nij = rij ./ repmat(dij,2,1);
            ra = (1-(la./dij).^2).*exp(-dij/lc);
            temp = repmat(ra,2,1) .* nij;
            u_grad(:,i) = nansum(temp(:,:),2);
        end

        %------ this is the states update part ------
        u_sum = u_auto + u_grad;  
        force = (u_sum(1,:).^2 + u_sum(2,:).^2).^(0.5);               
        ut = repmat(min(max_force,force),2,1) .* (u_sum./repmat(force,2,1));    % 加速能力设置上限
        u = ut + unifrnd(-noiseStr,noiseStr,2,Nc);                      % add noise
        vel = vel + u * dt;
        pos = pos + vel * dt;
    end
end



