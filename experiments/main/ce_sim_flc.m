% CE的纯sim实验 with FLC
clc
clear;
close all;

%% ------实验基本设置-----

% 初始化保存数据的结构体
G = struct ();
G.simStep = 0; % 当前的仿真步数


% 集群实验参数
G.maxID = 50                                ; % 机器人最大数量(maxID)
G.maxSimSteps = 1200; % 最大仿真步数
isShowAnimitation = 0; % 是否显示实时仿真动画

% 机器人基本参数设置
G.cycTime = 0.2; % sec, 状态更新周期(经测试最短约为0.1s)
G.v0 = 12;% mm/s，机器人中心线速度可取值：{0, 5, 10, 12, 15, 18, 20, 24};
G.maxRotRate = 12*1.91; % deg/s,机器人最大角速度
G.r_sense = 1000; % 机器人感知半径
G.R_escape = 300 + zeros(G.maxID,1);  % 一般个体的预警半径，设定是足够近之后，就会激活逃逸行为
G.R_escape(2,1) = 1000;               % 信息个体的预警半径

% ----- 速度协同&位置协同相关参数 -----=        % 速度协同模式切换：'allinfo','fol','ave'
G.weight_align = 10;           % alignment的权重：0=仅位置协同；越大速度协同越强 1-8        % social interaction的权重：0=沿当前方向（惯性大）；1=按照邻居方向（无惯性）
G.Drep = 300;                  % 排斥力作用距离，mm
G.Dsen = 1000;                 % 个体感知距离，即吸引力作用距离，mm
G.weight_rep = 8;              % 位置协同中排斥项的权重
G.weight_att = 0.01;           % 位置协同中吸引项的权重：0=无吸引作用
G.cj_threshold = 0.01;            % 初始ms阈值（将被FLC动态调整）
G.weight_cj = 100;             % cj的尺度
G.deac_threshold = 0.1;        % 取消激活阈值
G.noise_mov = 0;               % 运动噪声0/0.1
G.max_neighbors = 7;           % 拓扑交互个体数目

% ----追逐逃逸实验相关参数-----
G.weight_esc = 1;               % 逃逸行为权重
G.hawkID = 1;               % 被追逐的目标ID
G.attackStep = 1;               % 追逐开始的时间
G.R_dead = 120;                % 猎杀半径
G.v0_hawk = 24;              % 追逐者速度

%----实验统计量-------
G.maxcj = zeros(G.maxSimSteps,1);


%--实验时间戳-----
timeStamp = datetime('now','Format','yyyyMMdd''T''HHmmss');
dateString = string(timeStamp);


G.actor{G.hawkID}.pose = [2000,2000];
G.actor{G.hawkID}.vel = [-1,-1];
G = initSimRobots(G,[0,0],100);
% 开始仿真loop
for t = 1:G.maxSimSteps
    G.simStep = t;
    [desTurnAngle,desSpeed,G] = algo_mst_ce_flc(G);        % 使用FLC版本的集群算法
    G = parallelSimRobots_v2(G,desTurnAngle,desSpeed);	% 虚拟个体
    fprintf('sim. steps = %d\n',t);                 % 打印相关信息
    if G.simStep >= G.attackStep && G.target_dist(end) <= G.R_dead
        break;
    end
    % 动态显示机器人运动数据
    showAnimationIfEnabled(G, isShowAnimitation);
end



%% 绘制仿真结果图
warnTime = find(G.warnNum == 1);
G.sur_time = G.simStep - warnTime(1,1);
fprintf('仿真测试结束，结果如图.....\n');
drawRobotsTraj(G);

%% 绘制FLC统计数据
fprintf('绘制FLC自适应阈值控制统计图...\n');
drawFLCStats(G);

%% 绘制运动显著性统计数据
fprintf('绘制运动显著性统计图...\n');
drawMotionSaliencyStats(G);

%%  保存仿真数据
savefileDir = "./Data_Sim/ce/" + dateString;
if ~exist(savefileDir, 'dir')
    mkdir(savefileDir);
end
% 组合文件名并保存仿真数据
fileName = savefileDir + "/SimData_ce_FLC_ali" + num2str(G.weight_align) + "_rep" + num2str(G.weight_rep) + "_att" + num2str(G.weight_att) + "_N" + num2str(G.maxID) + "_" + dateString + '.mat';
save(fileName, 'G');