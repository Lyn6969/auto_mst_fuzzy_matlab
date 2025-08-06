function test_randPose_visualization()
    % 可视化测试randPose函数的效果
    % 显示个体位置分散过程的动画和最终结果
    
    close all; clear; clc;
    
    %% 测试参数设置
    num_agents = 50;           % 个体数量
    show_animation = true;     % 是否显示动画
    save_animation = false;    % 是否保存动画为GIF
    
    %% 创建图形窗口
    figure('Position', [100, 100, 1200, 800], 'Name', 'randPose函数可视化测试');
    
    %% 调用原始函数获取结果
    [final_positions, final_velocities] = randPose(num_agents);
    
    %% 如果需要动画，重新运行一遍显示过程
    if show_animation
        % 重新实现仿真过程以显示动画
        simulate_with_animation(num_agents, save_animation);
    end
    
    %% 显示最终结果分析
    show_final_results(final_positions, final_velocities, num_agents);
end

function simulate_with_animation(num_agents, save_animation)
    % 重新实现仿真过程以显示动画
    
    % === 参数设置 ===
    max_simulation_steps = 10;
    time_step = 0.1;
    repulsion_range = 3;
    attraction_decay = 10;
    max_acceleration = 5;
    target_speed = 0;
    speed_relaxation_time = 0.1;
    noise_strength = 0.5;
    
    % === 初始化 ===
    current_positions = rand(2, num_agents) * 2;
    current_velocities = ones(2, num_agents);
    
    % 记录轨迹
    trajectory = zeros(2, num_agents, max_simulation_steps);
    
    % 创建子图
    subplot(2, 2, [1, 3]);
    
    % 初始化动画
    colors = hsv(num_agents);  % 为每个个体分配不同颜色
    agent_handles = [];
    trail_handles = [];
    
    % 设置动画保存
    if save_animation
        gif_filename = 'randPose_animation.gif';
        frame_delay = 0.3;
    end
    
    %% 仿真循环
    for step = 1:max_simulation_steps
        % --- 计算自驱力 ---
        current_speeds = vecnorm(current_velocities, 2, 1);
        speed_error = (target_speed - current_speeds) / speed_relaxation_time;
        safe_speeds = max(current_speeds, eps);
        velocity_directions = current_velocities ./ safe_speeds;
        self_propulsion_force = velocity_directions .* speed_error;
        
        % --- 计算相互作用力 ---
        pos_x = current_positions(1,:);
        pos_y = current_positions(2,:);
        
        distance_matrix = pdist2(current_positions', current_positions');
        distance_matrix(distance_matrix == 0) = inf;
        
        % 计算单位方向向量
        dx = pos_x - pos_x';
        dy = pos_y - pos_y';
        
        force_magnitude_matrix = (1 - (repulsion_range ./ distance_matrix).^2) .* ...
                                exp(-distance_matrix / attraction_decay);
        
        dx_norm = dx ./ distance_matrix;
        dy_norm = dy ./ distance_matrix;
        
        fx_total = sum(force_magnitude_matrix .* dx_norm, 2, 'omitnan')';
        fy_total = sum(force_magnitude_matrix .* dy_norm, 2, 'omitnan')';
        
        interaction_force = [fx_total; fy_total];
        
        % --- 状态更新 ---
        total_force = self_propulsion_force + interaction_force;
        force_magnitudes = vecnorm(total_force, 2, 1);
        scale_factors = min(max_acceleration ./ max(force_magnitudes, eps), 1);
        limited_force = total_force .* scale_factors;
        
        noise = (rand(2, num_agents) - 0.5) * 2 * noise_strength;
        acceleration = limited_force + noise;
        
        current_velocities = current_velocities + acceleration * time_step;
        current_positions = current_positions + current_velocities * time_step;
        
        % 记录轨迹
        trajectory(:, :, step) = current_positions;
        
        % 绘制当前帧
        cla;
        hold on;
        
        % 绘制个体轨迹
        for i = 1:num_agents
            if step > 1
                traj_x = squeeze(trajectory(1, i, 1:step));
                traj_y = squeeze(trajectory(2, i, 1:step));
                plot(traj_x, traj_y, '--', 'Color', colors(i,:), 'LineWidth', 0.5);
            end
        end
        
        % 绘制当前个体位置
        for i = 1:num_agents
            pos = current_positions(:, i);
            vel = current_velocities(:, i);
            
            % 绘制个体圆圈
            circle_size = 50;
            scatter(pos(1), pos(2), circle_size, colors(i,:), 'filled', 'MarkerEdgeColor', 'k');
            
            % 绘制速度箭头
            arrow_scale = 0.1;
            if norm(vel) > 0.01
                quiver(pos(1), pos(2), vel(1)*arrow_scale, vel(2)*arrow_scale, ...
                       'Color', colors(i,:), 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
            end
        end
        
        % 绘制感知范围示例（第一个个体）
        if step == 1
            pos_example = current_positions(:, 1);
            circle_range = viscircles([pos_example(1), pos_example(2)], repulsion_range/2, ...
                                    'Color', 'r', 'LineStyle', '--', 'LineWidth', 1);
        end
        
        % 设置图形属性
        axis equal;
        xlim([-1, 3]);
        ylim([-1, 3]);
        grid on;
        title(sprintf('个体位置分散过程 - 步骤 %d/%d', step, max_simulation_steps), 'FontSize', 14);
        xlabel('X坐标', 'FontSize', 12);
        ylabel('Y坐标', 'FontSize', 12);
        
        % 图例已移除
        
        drawnow;
        
        % 保存动画帧
        if save_animation
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256);
            if step == 1
                imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', frame_delay);
            else
                imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', frame_delay);
            end
        end
        
        pause(0.3);  % 控制动画速度
    end
    
    if save_animation
        fprintf('动画已保存为: %s\n', gif_filename);
    end
end

function show_final_results(final_positions, final_velocities, num_agents)
    % 显示最终结果分析
    
    %% 最终位置分布
    subplot(2, 2, 2);
    scatter(final_positions(1,:), final_positions(2,:), 100, 'filled', 'MarkerEdgeColor', 'k');
    axis equal;
    grid on;
    title('最终位置分布', 'FontSize', 14);
    xlabel('X坐标', 'FontSize', 12);
    ylabel('Y坐标', 'FontSize', 12);
    
    % 计算最小距离
    distances = pdist(final_positions');
    min_distance = min(distances);
    mean_distance = mean(distances);
    
    text(0.05, 0.95, sprintf('最小距离: %.3f', min_distance), 'Units', 'normalized', 'FontSize', 10);
    text(0.05, 0.85, sprintf('平均距离: %.3f', mean_distance), 'Units', 'normalized', 'FontSize', 10);
    
    %% 速度分析
    subplot(2, 2, 4);
    final_speeds = vecnorm(final_velocities, 2, 1);
    
    % 速度直方图
    histogram(final_speeds, 10, 'FaceColor', 'skyblue', 'EdgeColor', 'k');
    xlabel('速度大小', 'FontSize', 12);
    ylabel('个体数量', 'FontSize', 12);
    title('最终速度分布', 'FontSize', 14);
    grid on;
    
    % 添加统计信息
    mean_speed = mean(final_speeds);
    std_speed = std(final_speeds);
    
    text(0.6, 0.8, sprintf('平均速度: %.3f', mean_speed), 'Units', 'normalized', 'FontSize', 10);
    text(0.6, 0.7, sprintf('速度标准差: %.3f', std_speed), 'Units', 'normalized', 'FontSize', 10);
    
    %% 打印总结信息
    fprintf('\n=== randPose函数测试结果 ===\n');
    fprintf('个体数量: %d\n', num_agents);
    fprintf('最小个体间距离: %.4f\n', min_distance);
    fprintf('平均个体间距离: %.4f\n', mean_distance);
    fprintf('最终平均速度: %.4f\n', mean_speed);
    fprintf('速度标准差: %.4f\n', std_speed);
    
    % 检查是否有重叠
    overlap_threshold = 0.1;
    if min_distance < overlap_threshold
        fprintf('警告: 检测到个体可能重叠 (最小距离 < %.2f)\n', overlap_threshold);
    else
        fprintf('✓ 所有个体位置分散良好，无重叠\n');
    end
    
    if mean_speed < 0.5
        fprintf('✓ 个体运动趋于稳定 (平均速度 < 0.5)\n');
    else
        fprintf('注意: 个体运动仍较活跃 (平均速度 = %.3f)\n', mean_speed);
    end
    
    fprintf('===============================\n\n');
end