% LYN 的chase and escacpe实验 
clc;
clear
%% ------实验基本设置-----
G = struct();
G.simStep = 0;

isParalleSimulation = 1;        % 是否进行平行仿真
isShowAnimitation = 0;       % 是否显示动画
% 机器人基本参数设置
G.maxID = 50;                   % 机器人最大数量(maxID)
G.cycTime = 0.2;                % sec, 状态更新周期(经测试最短约为0.1s)
G.v0 = 12;                      % mm/s，机器人中心线速度可取值：{0, 5, 10, 12, 15, 18, 20, 24};
G.maxRotRate = 12*1.91;         % deg/s,机器人最大角速度
G.r_sense = 1000;   

% ----- 速度协同&位置协同相关参数 -----=        % 速度协同模式切换：'allinfo','fol','ave'
G.weight_align = 10;           % alignment的权重：0=仅位置协同；越大速度协同越强 1-8        % social interaction的权重：0=沿当前方向（惯性大）；1=按照邻居方向（无惯性）
G.Drep = 300;                  % 排斥力作用距离，mm
G.Dsen = 1000;                 % 个体感知距离，即吸引力作用距离，mm
G.weight_rep = 8;              % 位置协同中排斥项的权重
G.weight_att = 0.01;           % 位置协同中吸引项的权重：0=无吸引作用
G.cj_threshold = 1;            % ms阈值
G.weight_cj = 100;             % cj的尺度
G.deac_threshold = 0.2;        % 取消激活阈值
G.noise_mov = 0;               % 运动噪声0/0.1
G.max_neighbors = 7;           % 拓扑交互个体数目


% ----追逐逃逸实验相关参数-----
G.align_type = 'ave';    
G.weight_esc = 1;               % 逃逸行为权重
G.hawkID = 1;               % 被追逐的目标ID
G.attackStep = 1;               % 追逐开始的时间
G.R_dead = 120;                % 猎杀半径
G.v0_hawk = 24;              % 追逐者速度
% -- 设置个体的预警半径----
G.R_escape = 300 + zeros(G.maxID,1);  % 一般个体的预警半径，设定是足够近之后，就会激活逃逸行为
G.R_escape(2,1) = 1000;               % 信息个体的预警半径

% ----- 实验步长及信息个体设置 -----
G.maxSimSteps = 1200;
G.maxcj = zeros(G.maxSimSteps,1);

% ----- 实验时间戳 -----
timeStamp = datetime('now','Format','yyyyMMdd''T''HHmmss');
dateString = string(timeStamp);


%% ------ 运行仿真，测试相关参数 -----
if isParalleSimulation
    % 初始化虚拟个体
    G.actor{G.hawkID}.pose = [2000,2000];
    G.actor{G.hawkID}.vel = [-1,-1];
    G = initSimRobots(G,'unif',[0,0],100,isParalleSimulation);
    % 开始仿真loop
    for t = 1:G.maxSimSteps
        G.simStep = t;
        [desTurnAngle,desSpeed,G] = algo_mst_ce(G);        % 集群算法
        G = parallelSimRobots_v2(G,desTurnAngle,desSpeed);	% 虚拟个体
        fprintf('sim. steps = %d\n',t);                 % 打印相关信息
        if G.simStep >= G.attackStep && G.target_dist(end) <= G.R_dead
            break;
        end
        % 动态显示机器人运动数据
        if isShowAnimitation
            drawRobotsMotion_dynamic(G);        
            pause(0.01);
        end
    end
    a = find(G.warnNum == 1);
    G.sur_time = G.simStep - a(1,1);
    % 显示机器人运动数
    fprintf('仿真测试结束，结果如图.....\n');
    drawRobotsTraj(G);
    
    % 创建时间戳和文件保存目录
    savefileDir = "./Data_Sim/ce/" + dateString;
    if ~exist(savefileDir, 'dir')
        mkdir(savefileDir);
    end
    % 组合文件名并保存仿真数据
    fileName = savefileDir + "/SimData_ce_C" + num2str(G.cj_threshold) + "_ali" + num2str(G.weight_align) + "_rep" + num2str(G.weight_rep) + "_att" + num2str(G.weight_att) + "_N" + num2str(G.maxID) + "_" + dateString + '.mat';
    save(fileName, 'G');

    % 询问是否开始实机实验 
    choice = menu('是否开始机器人实验？','Ok','Exit');
    if choice == 2
        close all; return;
    end
end
%%  --------执行集群机器人实验主任务 -----
delete(instrfindall);  % 先删除所有已经加载的串口
fprintf('尝试打开串口&动捕...\n');
scom1 = openSerialPort('COM3');     % 打开串口1
scom2 = openSerialPort('COM6');     % 打开串口2
NOKOV_Value = mCortexExit();        % 先关闭动捕，解决前次意外关闭问题   
NOKOV_Value = openNOKOV();          % 打开动捕 
% step-1: 初始化个体的位置等信息
G = getSwarmBangMotionInfo(G); 
pause(0.5);
% step-2: 发送任务开始指令(连发3次)
fprintf('机器人实验开始...\n');
cmd_start = '01000000000000000000000000000000000000000000000000000000000006';	% 预设指令
sendFrameData_hexStr(scom1,cmd_start,3); 
sendFrameData_hexStr(scom2,cmd_start,3); 
pause(0.5);

% check机器人是否正常，确认后开始实验
answer = questdlg('要不要进行机器人自检？', ...
    '自检', ...
    '是，马上自检','不，直接进行实验',"取消实验","取消实验");
% Handle response
switch answer
    case '是，马上自检'
        disp([answer '自检中...'])
        robotsTurn_P25(scom1,1:25);
        robotsTurn_P25(scom2,1:25);
        answer_exp = questdlg('要不要进行机器人实验？', ...
                '是否实验', ...
                '是','否','否');
        switch answer_exp
            case '是'
                disp([answer_exp '实验中...'])
            case '否'
                disp([answer_exp '取消实验...'])
                all_stop();
                close all; return;
        end
    case '不，直接进行实验'
        disp([answer '直接进行实验...'])
    case '取消实验'
        disp([answer '取消实验...'])
        all_stop();
        close all; return;
end

a = find(G.warnNum == 1);
G.sur_time = G.simStep - a(1,1);
% step-4：机器人暂停运动，显示机器人运动数据
robotsStop_P25(scom1,[1:25]);
robotsStop_P25(scom2,[1:25]); 
% step-5：关闭串口、动捕等硬件
closeSerialPort(scom1);         % 关闭串口
closeSerialPort(scom2);         % 关闭串口
exitNOKOV(NOKOV_Value);         % 关闭动捕
fprintf('关闭串口 & 动捕...\n');
% 保存机器人实验数据
savefileDir = "./Data_Exp/ce/" + dateString + "_C" + num2str(G.cj_threshold) + "_sur" + num2str(G.sur_time);
if ~exist(savefileDir, 'dir')
    mkdir(savefileDir);
end
% 组合文件名并保存仿真数据
fileName = savefileDir + "/ExpData_ce_C" + num2str(G.cj_threshold) + "_ali" + num2str(G.weight_align) + "_rep" + num2str(G.weight_rep) + "_att" + num2str(G.weight_att) +  "_cjscal" + num2str(G.weight_cj)+ "_N" + num2str(G.maxID) + "_" + dateString+ '.mat';
save(fileName, 'G');
fprintf('数据已保存，实验结束！\n');
disp(dateString);
% 显示机器人运动数据
fprintf('实验结束，机器人运动轨迹如图...\n');
drawRobotsTraj(G);

%% ---------初始化虚拟个体 ----------
function G = initSimRobots(G,rand_mode,pos_offset,r,isParalleSimulation)
    % ----- 机器人的数量及全体编号列表 -----
    G.num = G.maxID;
    G.robotsList = 1:G.maxID;
    robotsNum = G.maxID;
    
    % ------ 初始化个体位置及方向 ------
    % mode-1：位置&方向完全随机，可能重叠
    if strcmp(rand_mode,'rand')
        robotsPos(:,[1,2]) = r*rand(robotsNum,2);
        robotsPos(:,[3,4]) = 0.5 - rand(robotsNum,2);
    end
    % mode-2：基本均匀的随机分布，不会重叠
    if strcmp(rand_mode,'unif')
        [rp,~] = randpose_unif(robotsNum);
        robotsPos(:,1) = r*rp(1,:)' - pos_offset(1,1);     % mm,适当平移（沿X-轴）
        robotsPos(:,2) = r*rp(2,:)' - pos_offset(1,2);      % mm,适当平移（沿Y-轴）
        robotsPos(:,[3,4]) = 0.1*(0.5 - rand(robotsNum,2)); % 方向小范围随机

        if isParalleSimulation
            % 获取捕食者的位置
            predatorPos = G.actor{G.hawkID}.pose;
            % 计算每个位置到捕食者的距离
            distances = sqrt(sum((robotsPos(:,1:2) - predatorPos).^2, 2));
            % 找到最近的位置
            [~, idx] = min(distances);
        
            % 将2号个体设置为最近的位置
            tempPos = robotsPos(2,:);
            robotsPos(2,:) = robotsPos(idx,:);
            robotsPos(idx,:) = tempPos;
        end
    end

    % ---- 记录机器人的实时运动状态
    for i = 1:robotsNum
        G.actor{i}.id = i;
        G.actor{i}.pose = robotsPos(i,[1,2]);
        G.actor{i}.vel = unitvel([1,1]);%unitvel(robotsPos(i,[3,4]));
        % 个体的激活状态初始化
        G.actor{i}.is_activated = 0;
        G.actor{i}.src_id = NaN;
    end

    G.actor{G.hawkID}.pose = [2000,2000];
    G.actor{G.hawkID}.vel = [-1,-1];
end

%% 画轨迹图
function drawRobotsTraj(G)
    %  load ('ExpsData_predation_mAve_N20_20220415T110918.mat'); 
        % --------- 计算相关统计量 ---------    
        heading = nan * zeros(G.simStep,G.maxID);
        speed = nan * zeros(G.simStep,G.maxID);
        rotRate = nan * zeros(G.simStep,G.maxID);
        flockpos = nan * zeros(G.simStep,2);
        for t = 1:G.simStep
            posDir = [];
            p_posDir = [];
            for i = 1:G.maxID
                heading(t,i) = vel2heading_deg(G.actor{i}.memory(t,[3,4]));
                posDir(i,[1,2,3,4]) = G.actor{i}.memory(t,[1,2,3,4]);
                if t>=2
                    p_posDir(i,[1,2,3,4]) = G.actor{i}.memory(t-1,[1,2,3,4]);
                    speed(t,i) = norm(posDir(i,[1,2])-p_posDir(i,[1,2]))/G.cycTime;
                    rotRate(t,i) = acosd(dot(posDir(i,[3,4]),p_posDir(i,[3,4])))/G.cycTime;
                end
            end
            flockpos(t,:) = nanmean(posDir(:,[1,2]),1);
        end
        % 计算 first catch time
        firstCatchTime = inf;
        indx = find(G.target_dist(G.attackStep:end)<=G.R_dead);
        if length(indx)>=1
            firstCatchTime = indx(1);
        end
        
        % --------- 绘图窗口设置 ---------
        figure('posi',[200,200,1000,600]);   	% 主窗口句柄
        h_trajAxes = axes('Posi',[0.05 0.08 0.5 0.9]);    	% 机器人轨迹显示
        xlim([-3000,3000]); ylim([-3000,3000]); 
    %     h_trajAxes_in = axes('Posi',[0.05 0.08 0.5 0.35]);    	% 机器人初始状态显示
    %     xlim([-3000,3000]); ylim([-3000,3000]); 
        h_Axes1 = axes('Posi',[0.6 0.82 0.35 0.15]);    % 机器人线速度显示 
        h_Axes2 = axes('Posi',[0.6 0.57 0.35 0.15]);	% 机器人朝向显示 
        h_Axes3 = axes('Posi',[0.6 0.33 0.35 0.15]);	% 机器人角速度显示      
        h_opAxes = axes('Posi',[0.6 0.08 0.35 0.15]);   % 群体序参量显示
        
        % ------------  主窗口 --------------
        % 绘制：机器人位置、heading、轨迹    
        r = 30;             % 机器人半径mm
        arrow_scale = 30;   % 机器人heading
        axes(h_trajAxes); box on; grid on; axis equal; 
        %% 显示末状态位置
    %     for i = 1:G.maxID
    %         pos = G.actor{i}.pose;
    %         vel = G.actor{i}.vel;
    %         tailTraj = G.actor{i}.memory(:,[1,2]);
    %         if ~isnan(pos)
    %             % 显示机器人运动轨迹
    %             quiver(pos(1),pos(2),arrow_scale*vel(1),arrow_scale*vel(2),0,'k','linewidth',1); 
    %             hold on;
    %             line(tailTraj(:,1),tailTraj(:,2),'linestyle','--','linewidth',0.5,'color',[0.5,0.5,0.5]); 
    %             hold on;
    %             rectangle('Position', [pos(1)-r, pos(2)-r, r*2, r*2], 'Curvature', [1 1]); 
    %             hold on;
    %         end
    %     end
    %% 显示初始状态位置
        for i = 1:G.maxID
            pos = G.actor{i}.memory(1,[1,2]);
            vel =G.actor{i}.memory(1,[3,4]);
            tailTraj = G.actor{i}.memory(:,[1,2]);
            if ~isnan(pos)
                % 显示机器人运动轨迹
                quiver(pos(1),pos(2),arrow_scale*vel(1),arrow_scale*vel(2),0,'k','linewidth',1); 
                hold on;
                line(tailTraj(:,1),tailTraj(:,2),'linestyle','--','linewidth',0.5,'color',[0.5,0.5,0.5]); 
                hold on;
                rectangle('Position', [pos(1)-r, pos(2)-r, r*2, r*2], 'Curvature', [1 1],'linewidth',0.9); 
                hold on;
            end
        end
         for i = G.robotsList
         pos = floor(G.actor{i}.pose);
            rectangle('Position', [pos(1)-r, pos(2)-r, r*2, r*2], 'Curvature', [1 1],'linewidth',0.9,'linestyle','-','edgecolor',[0.5,0.5,0.5]); hold on;
         end
       %% 绘制其他信息 
        % 绘制：群体中心
        line(flockpos(:,1),flockpos(:,2),'linestyle','-','linewidth',1.5,'color',[0,0,1]); hold on;
        % 绘制：个体的感知范围（id=2）
        pos = G.actor{2}.pose;
    %     rectangle('Position', [pos(1)-G.r_sense, pos(2)-G.r_sense, G.r_sense*2, G.r_sense*2], 'Curvature', [1 1],'edgecolor',[0,1,0],'linestyle','--'); hold on; %绘制末态范围
        pos = G.actor{2}.memory(1,[1,2]);
        rectangle('Position', [pos(1)-G.r_sense, pos(2)-G.r_sense, G.r_sense*2, G.r_sense*2], 'Curvature', [1 1],'edgecolor',[87/255,176/255,88/255],'linewidth',1); hold on;%绘制初始状态范围
        % 绘制：predator's traj. 
        tailTraj = G.actor{G.hawkID}.memory(:,[1,2]);
        line(tailTraj(:,1),tailTraj(:,2),'linestyle','--','linewidth',1,'color',[206/255,60/255,53/255]); hold on;
        pos = G.actor{1}.pose;
        R_esp = max(G.R_escape);
    %     rectangle('Position', [pos(1)-R_esp, pos(2)-R_esp, R_esp*2, R_esp*2], 'Curvature', [1 1],'edgecolor',[1,0,0],'linestyle','--'); hold on;
        rectangle('Position', [pos(1)-G.R_dead, pos(2)-G.R_dead, G.R_dead*2, G.R_dead*2], 'Curvature', [1 1],'edgecolor',[206/255,60/255,53/255]); hold on;
        % 绘制：场地边界
        rectangle('Position', [-2790,-2890,5380, 5680], 'Curvature', [0 0],'linewidth',2); hold on;
        rectangle('Position', [-2100,-2350,4200, 4700], 'Curvature', [0 0],'linewidth',3,'edgecolor',[5/255,74/255,145/255]); hold on;
        box on; grid on; axis equal; 
         % 绘制：攻击初始方向箭头
        pos1 = [G.actor{1}.memory(1,1) G.actor{1}.memory(1,2)];
        pos2 = (G.actor{1}.memory(1,[1,2])+G.actor{1}.memory(1,[3,4]));
        arrow(pos1,pos2,'Color',[206/255,60/255,53/255],'Length',18,'BaseAngle',45);hold on;
        % 绘制：逃避初始方向箭头
        pos1 = [G.actor{2}.memory(1,1) G.actor{2}.memory(1,2)];
        pos2 = (G.actor{2}.memory(1,[1,2])+G.actor{2}.memory(1,[3,4]));
        arrow(pos1,pos2,'Color',[87/255,176/255,88/255],'Length',18,'BaseAngle',45);hold on;
        % 绘制：当前使用模型类型（mave/sfol）与逃逸时间
        annotation('textbox', [.06 .87, .1, .1],'string',G.align_type,'fontsize', 20,'color','black','edgecolor','none');hold on;
        annotation('textbox', [.14 .87, .1, .1],'string',num2str(G.sur_time),'fontsize', 20,'color','r','edgecolor','none');hold on;
    %     [trueStep] = Exps_swarmBang_predation_step(G);
        
        % 绘制：图注
         arrow([-2500,-2650],[-2500,-2600],'Color',[87/255,176/255,88/255],'Length',15,'BaseAngle',45);hold on;
         annotation('textbox', [.08 .05, .1, .1],'string',':群体逃逸方向','fontsize', 13,'color','b','edgecolor','none');hold on;
         arrow([-1000,-2650],[-1000,-2600],'Color',[206/255,60/255,53/255],'Length',13,'BaseAngle',45);hold on;
         annotation('textbox', [.22 .05, .1, .1],'string',':捕食者进攻方向','fontsize', 13,'color','r','edgecolor','none');hold on;
         rectangle('Position', [650, -2750, 150, 150], 'Curvature', [1 1],'edgecolor',[87/255,176/255,88/255],'linewidth',1); hold on;
         annotation('textbox', [.383 .05, .1, .1],'string',':群体感知范围','fontsize', 13,'color','black','edgecolor','none');hold on;
         
    %      %% 绘制：机器人初始位置与攻击方向    
    %     axes(h_trajAxes_in); box on; grid on; axis equal; 
    %     for i = 1:G.maxID
    %         pos =  G.actor{i}.memory(1,[1,2]);
    %         vel =  G.actor{i}.memory(1,[3,4]);
    %         if ~isnan(pos)
    %             quiver(pos(1),pos(2),arrow_scale*vel(1),arrow_scale*vel(2),0,'k','linewidth',1); 
    %             hold on;
    %             rectangle('Position', [pos(1)-r, pos(2)-r, r*2, r*2], 'Curvature', [1 1]); 
    %             hold on;
    %         end
    %     end
    %     title('群体初始状态');
    %         % 绘制：场地边界
    %     rectangle('Position', [-1790,-1890,3380, 3680], 'Curvature', [0 0],'linewidth',0.05); hold on;
    %     box on; grid on; axis equal; 
    %      % 绘制：个体的感知范围（id=2）
    %     pos = G.actor{2}.memory(1,[1,2]);
    %     rectangle('Position', [pos(1)-G.r_sense, pos(2)-G.r_sense, G.r_sense*2, G.r_sense*2], 'Curvature', [1 1],'edgecolor',[0,1,0]); hold on;
    %     % 绘制：攻击方向箭头
    %     pos1 = [G.actor{1}.memory(1,1) G.actor{1}.memory(1,2)];
    %     pos2 = (G.actor{1}.memory(1,[1,2])+G.actor{1}.memory(1,[3,4]));
    %     arrow(pos1,pos2,'Color','r','Length',13);hold on;
    % 
    %     pos1 = [G.actor{2}.memory(1,1) G.actor{2}.memory(1,2)];
    %     pos2 = (G.actor{2}.memory(1,[1,2])+G.actor{2}.memory(1,[3,4]));
    %     arrow(pos1,pos2,'Color','g','Length',13);hold on;
        
        % ------------- 辅助窗口 ------------
        % 显示：预警个体数量曲线
        axes(h_Axes1); 
        line(1:G.simStep,G.warnNum(1,:),'linestyle','-','linewidth',1); hold on;
        box on; grid on; 
        xlabel('time/step'); 
        ylabel('N_{warn}'); % ylim([0,1]);
        % 显示：heading曲线
        axes(h_Axes2);
        plot(1:G.simStep,heading); hold on;
        plot(1:G.simStep,heading(:,G.hawkID),'linewidth',1,'color',[1,0,0]); hold on;
        xlabel('steps');
        ylabel('heading(deg)');
        % 显示：dist2target曲线
        axes(h_Axes3);
        plot(1:G.simStep,G.target_dist); hold on;
        xlabel('steps');
        ylabel('dist2target');
        % 显示捕获时间
        title(['fct(捕获时间)=',num2str(G.sur_time),'步']);
        % 显示：op曲线
        axes(h_opAxes); 
        box on; grid on; xlabel('time/step'); ylabel('op'); ylim([0,1]);
        line(1:G.simStep,G.op(1,:),'linestyle','-','linewidth',1); hold on;
end
%% 画轨迹图
drawRobotsTraj(G)