function drawRobotsTraj(G)
    %  load ('ExpsData_predation_mAve_N20_20220415T110918.mat'); 
        % 设置中文字体支持
        set(0,'DefaultAxesFontName','SimSun');
        set(0,'DefaultTextFontName','SimSun');
        
        % --------- �������ͳ���� ---------    
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
        % ���� first catch time
        firstCatchTime = inf;
        indx = find(G.target_dist(G.attackStep:end)<=G.R_dead);
        if length(indx)>=1
            firstCatchTime = indx(1);
        end
        
        % --------- ��ͼ�������� ---------
        figure('posi',[200,200,1000,600]);   	% �����ھ��
        h_trajAxes = axes('Posi',[0.05 0.08 0.5 0.9]);    	% �����˹켣��ʾ
        xlim([-3000,3000]); ylim([-3000,3000]); 
    %     h_trajAxes_in = axes('Posi',[0.05 0.08 0.5 0.35]);    	% �����˳�ʼ״̬��ʾ
    %     xlim([-3000,3000]); ylim([-3000,3000]); 
        h_Axes1 = axes('Posi',[0.6 0.82 0.35 0.15]);    % ���������ٶ���ʾ 
        h_Axes2 = axes('Posi',[0.6 0.57 0.35 0.15]);	% �����˳�����ʾ 
        h_Axes3 = axes('Posi',[0.6 0.33 0.35 0.15]);	% �����˽��ٶ���ʾ      
        h_opAxes = axes('Posi',[0.6 0.08 0.35 0.15]);   % Ⱥ���������ʾ
        
        % ------------  ������ --------------
        % ���ƣ�������λ�á�heading���켣    
        r = 30;             % �����˰뾶mm
        arrow_scale = 30;   % ������heading
        axes(h_trajAxes); box on; grid on; axis equal; 
        %% ��ʾĩ״̬λ��
    %     for i = 1:G.maxID
    %         pos = G.actor{i}.pose;
    %         vel = G.actor{i}.vel;
    %         tailTraj = G.actor{i}.memory(:,[1,2]);
    %         if ~isnan(pos)
    %             % ��ʾ�������˶��켣
    %             quiver(pos(1),pos(2),arrow_scale*vel(1),arrow_scale*vel(2),0,'k','linewidth',1); 
    %             hold on;
    %             line(tailTraj(:,1),tailTraj(:,2),'linestyle','--','linewidth',0.5,'color',[0.5,0.5,0.5]); 
    %             hold on;
    %             rectangle('Position', [pos(1)-r, pos(2)-r, r*2, r*2], 'Curvature', [1 1]); 
    %             hold on;
    %         end
    %     end
    %% ��ʾ��ʼ״̬λ��
        for i = 1:G.maxID
            pos = G.actor{i}.memory(1,[1,2]);
            vel =G.actor{i}.memory(1,[3,4]);
            tailTraj = G.actor{i}.memory(:,[1,2]);
            if ~isnan(pos)
                % ��ʾ�������˶��켣
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
       %% ����������Ϣ 
        % ���ƣ�Ⱥ������
        line(flockpos(:,1),flockpos(:,2),'linestyle','-','linewidth',1.5,'color',[0,0,1]); hold on;
        % ���ƣ�����ĸ�֪��Χ��id=2��
        pos = G.actor{2}.pose;
    %     rectangle('Position', [pos(1)-G.r_sense, pos(2)-G.r_sense, G.r_sense*2, G.r_sense*2], 'Curvature', [1 1],'edgecolor',[0,1,0],'linestyle','--'); hold on; %����ĩ̬��Χ
        pos = G.actor{2}.memory(1,[1,2]);
        rectangle('Position', [pos(1)-G.r_sense, pos(2)-G.r_sense, G.r_sense*2, G.r_sense*2], 'Curvature', [1 1],'edgecolor',[87/255,176/255,88/255],'linewidth',1); hold on;%���Ƴ�ʼ״̬��Χ
        % ���ƣ�predator's traj. 
        tailTraj = G.actor{G.hawkID}.memory(:,[1,2]);
        line(tailTraj(:,1),tailTraj(:,2),'linestyle','--','linewidth',1,'color',[206/255,60/255,53/255]); hold on;
        pos = G.actor{1}.pose;
        R_esp = max(G.R_escape);
    %     rectangle('Position', [pos(1)-R_esp, pos(2)-R_esp, R_esp*2, R_esp*2], 'Curvature', [1 1],'edgecolor',[1,0,0],'linestyle','--'); hold on;
        rectangle('Position', [pos(1)-G.R_dead, pos(2)-G.R_dead, G.R_dead*2, G.R_dead*2], 'Curvature', [1 1],'edgecolor',[206/255,60/255,53/255]); hold on;
        % ���ƣ����ر߽�
        rectangle('Position', [-2790,-2890,5380, 5680], 'Curvature', [0 0],'linewidth',2); hold on;
        rectangle('Position', [-2100,-2350,4200, 4700], 'Curvature', [0 0],'linewidth',3,'edgecolor',[5/255,74/255,145/255]); hold on;
        box on; grid on; axis equal; 
         % ���ƣ�������ʼ�����ͷ
        pos1 = [G.actor{1}.memory(1,1) G.actor{1}.memory(1,2)];
        pos2 = (G.actor{1}.memory(1,[1,2])+G.actor{1}.memory(1,[3,4]));
        arrow(pos1,pos2,'Color',[206/255,60/255,53/255],'Length',18,'BaseAngle',45);hold on;
        % ���ƣ��ӱܳ�ʼ�����ͷ
        pos1 = [G.actor{2}.memory(1,1) G.actor{2}.memory(1,2)];
        pos2 = (G.actor{2}.memory(1,[1,2])+G.actor{2}.memory(1,[3,4]));
        arrow(pos1,pos2,'Color',[87/255,176/255,88/255],'Length',18,'BaseAngle',45);hold on;
        % ���ƣ���ǰʹ��ģ�����ͣ�mave/sfol��������ʱ��
        % annotation('textbox', [.06 .87, .1, .1],'string',G.align_type,'fontsize', 20,'color','black','edgecolor','none');hold on;
        annotation('textbox', [.14 .87, .1, .1],'string',num2str(G.sur_time),'fontsize', 20,'color','r','edgecolor','none');hold on;
    %     [trueStep] = Exps_swarmBang_predation_step(G);
        
        % 绘制：图注
         arrow([-2500,-2650],[-2500,-2600],'Color',[87/255,176/255,88/255],'Length',15,'BaseAngle',45);hold on;
         annotation('textbox', [.08 .05, .1, .1],'string',':群体运动方向','fontsize', 13,'color','b','edgecolor','none');hold on;
         arrow([-1000,-2650],[-1000,-2600],'Color',[206/255,60/255,53/255],'Length',13,'BaseAngle',45);hold on;
         annotation('textbox', [.22 .05, .1, .1],'string',':捕食者进攻方向','fontsize', 13,'color','r','edgecolor','none');hold on;
         rectangle('Position', [650, -2750, 150, 150], 'Curvature', [1 1],'edgecolor',[87/255,176/255,88/255],'linewidth',1); hold on;
         annotation('textbox', [.383 .05, .1, .1],'string',':群体感知范围','fontsize', 13,'color','black','edgecolor','none');hold on;
         
    %      %% ���ƣ������˳�ʼλ���빥������    
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
    %     title('Ⱥ���ʼ״̬');
    %         % ���ƣ����ر߽�
    %     rectangle('Position', [-1790,-1890,3380, 3680], 'Curvature', [0 0],'linewidth',0.05); hold on;
    %     box on; grid on; axis equal; 
    %      % ���ƣ�����ĸ�֪��Χ��id=2��
    %     pos = G.actor{2}.memory(1,[1,2]);
    %     rectangle('Position', [pos(1)-G.r_sense, pos(2)-G.r_sense, G.r_sense*2, G.r_sense*2], 'Curvature', [1 1],'edgecolor',[0,1,0]); hold on;
    %     % ���ƣ����������ͷ
    %     pos1 = [G.actor{1}.memory(1,1) G.actor{1}.memory(1,2)];
    %     pos2 = (G.actor{1}.memory(1,[1,2])+G.actor{1}.memory(1,[3,4]));
    %     arrow(pos1,pos2,'Color','r','Length',13);hold on;
    % 
    %     pos1 = [G.actor{2}.memory(1,1) G.actor{2}.memory(1,2)];
    %     pos2 = (G.actor{2}.memory(1,[1,2])+G.actor{2}.memory(1,[3,4]));
    %     arrow(pos1,pos2,'Color','g','Length',13);hold on;
        
        % ------------- �������� ------------
        % ��ʾ��Ԥ��������������
        axes(h_Axes1); 
        line(1:G.simStep,G.warnNum(1,:),'linestyle','-','linewidth',1); hold on;
        box on; grid on; 
        xlabel('time/step'); 
        ylabel('N_{warn}'); % ylim([0,1]);
        % ��ʾ��heading����
        axes(h_Axes2);
        plot(1:G.simStep,heading); hold on;
        plot(1:G.simStep,heading(:,G.hawkID),'linewidth',1,'color',[1,0,0]); hold on;
        xlabel('steps');
        ylabel('heading(deg)');
        % ��ʾ��dist2target����
        axes(h_Axes3);
        plot(1:G.simStep,G.target_dist); hold on;
        xlabel('steps');
        ylabel('dist2target');
        % 显示生存时间
        title(['fct(生存时间)=',num2str(G.sur_time),'步']);
        % ��ʾ��op����
        axes(h_opAxes); 
        box on; grid on; xlabel('time/step'); ylabel('op'); ylim([0,1]);
        line(1:G.simStep,G.op(1,:),'linestyle','-','linewidth',1); hold on;
end
