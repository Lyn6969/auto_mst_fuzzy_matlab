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
        % annotation('textbox', [.06 .87, .1, .1],'string',G.align_type,'fontsize', 20,'color','black','edgecolor','none');hold on;
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
