% ******************* 显示机器人实时运动状态 **********************
function drawRobotsMotion_dynamic(G)
    global h_trajAxes;
    global h_speedAxes;
    global h_headingAxes;
    global h_opAxes;
    global h_robTrajs;
    global h_op;
    global h_heading;
    global h_speed;
    
    % 机器人半径mm
    r = 30;
    arrow_scale = 30;
    
    myColors = parula(G.maxID);
    
    simStep = G.simStep;
    if simStep == 1
        figure('posi',[100,200,1000,500]);   	% 主窗口句柄
        h_trajAxes = axes('Posi',[0.05 0.08 0.45 0.89]);    	% 机器人实时轨迹显示
        xlim([-2000,2000]); ylim([-2000,2000]); 
        h_headingAxes = axes('Posi',[0.61 0.72 0.35 0.25]);    	% 机器人线速度显示 
        h_speedAxes = axes('Posi',[0.61 0.40 0.35 0.25]);		% 机器人朝向显示 
        h_opAxes = axes('Posi',[0.61 0.08 0.35 0.25]);          % 群体序参量显示
    end
    %
    for i = 1:G.num
        pos = G.actor{i}.pose;
        vel = G.actor{i}.vel;
        tailTraj = G.actor{i}.memory(:,[1,2]);
        for t = 1:G.simStep
            heading(1,t) = vel2heading_deg(G.actor{i}.memory(t,[3,4]));
            speed(1,t) = norm(G.actor{i}.memory(t,[3,4]));
        end
        if ~isnan(pos)
            if simStep == 1
                % 显示机器人运动轨迹
                axes(h_trajAxes); box on; grid on; axis equal;
                h_robTrajs(1,i) = rectangle('Position', [pos(1)-r, pos(2)-r, r*2, r*2], 'Curvature', [1 1]); hold on;
                h_robTrajs(2,i) = quiver(pos(1),pos(2),arrow_scale*vel(1),arrow_scale*vel(2),0,'k','linewidth',1); hold on;
                h_robTrajs(3,i) = line(tailTraj(:,1),tailTraj(:,2),'linestyle','-','linewidth',1,'color',myColors(i,:)); hold on;
                % 显示op
                axes(h_opAxes); box on; grid on; xlabel('time/step'); ylabel('op'); ylim([0,1]);
                h_op = line([1:simStep],G.op(1,:),'linestyle','-','linewidth',1); hold on;
                % 显示speed
                axes(h_speedAxes); box on; grid on; xlabel('time/step'); ylabel('speed'); 
                h_speed(1,i) = line([1:simStep],speed(1,:),'linestyle','-','linewidth',1,'color',myColors(i,:)); hold on;
                % 显示heading
                axes(h_headingAxes); box on; grid on; xlabel('time/step'); ylabel('heading');
                h_heading(1,i) = line([1:simStep],heading(1,:),'linestyle','-','linewidth',1,'color',myColors(i,:)); hold on;
            else
                set(h_robTrajs(1,i),'Position', [pos(1)-r, pos(2)-r, r*2, r*2]); 
                set(h_robTrajs(2,i),'XData', pos(1), 'YData',pos(2),'UData',arrow_scale*vel(1),'VData',arrow_scale*vel(2));
                set(h_robTrajs(3,i),'XData',tailTraj(:,1),'YData',tailTraj(:,2));
                title(['sim steps = ',num2str(simStep)]);
                set(h_op,'XData',[1:simStep],'YData',G.op(1,:));
                set(h_heading(1,i),'XData',[1:simStep],'YData',heading(1,:));
                set(h_speed(1,i),'XData',[1:simStep],'YData',speed(1,:));
            end
        end
    end
end