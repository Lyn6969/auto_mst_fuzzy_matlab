classdef FuzzyLogicController < handle
    % 模糊逻辑控制器完整实现
    
    properties
        c_min                % 最小阈值
        c_max                % 最大阈值
        saturation_margin    % 饱和边界
        fis                  % 模糊推理系统
    end
    
    methods
        function obj = FuzzyLogicController(c_min, c_max)
            % 构造函数
            if nargin < 1, c_min = 0.01; end
            if nargin < 2, c_max = 25.0; end
            
            obj.c_min = c_min;
            obj.c_max = c_max;
            obj.saturation_margin = 2.0;
            obj.setup_fuzzy_system();
        end
        
        function setup_fuzzy_system(obj)
            % 设置模糊推理系统
            obj.fis = obj.create_flc_fuzzy_system();
            obj.fis = obj.add_fuzzy_rules(obj.fis);
        end
        
        function fis = create_flc_fuzzy_system(obj)
            % 创建模糊逻辑控制器的模糊推理系统
            fis = mamfis('Name', 'FLC_Threshold_Controller');
            
            % 添加输入变量1: 局部有序度 p_i [0.8, 1] (基于实际数据分布调整)
            fis = addInput(fis, [0.7 1], 'Name', 'p_i');
            fis = addMF(fis, 'p_i', 'trapmf', [0.7 0.7 0.85 0.95], 'Name', 'Low');
            fis = addMF(fis, 'p_i', 'trimf', [0.9 0.95 1.0], 'Name', 'Medium'); 
            fis = addMF(fis, 'p_i', 'trapmf', [0.95 1.0 1.0 1.0], 'Name', 'High');
            
            % 添加输入变量2: 邻域运动显著性均值 avg(M_j) [0, 20] (基于实际数据分布调整)
            fis = addInput(fis, [0 20], 'Name', 'avg_Mj');
            % 降低了avg_Mj的隶属度函数边界，使其对较小的运动显著性更加敏感
            fis = addMF(fis, 'avg_Mj', 'trapmf', [0 0 1 2], 'Name', 'Low');
            fis = addMF(fis, 'avg_Mj', 'trimf', [1.5 3 6], 'Name', 'Medium');
            fis = addMF(fis, 'avg_Mj', 'trapmf', [5 7 20 20], 'Name', 'High');
            
            % 添加输入变量3: 邻域运动显著性方差 var(M_j) [0, 5] (基于实际数据分布调整)
            fis = addInput(fis, [0 5], 'Name', 'var_Mj');
            fis = addMF(fis, 'var_Mj', 'trapmf', [0 0 0.5 1.5], 'Name', 'Low');
            fis = addMF(fis, 'var_Mj', 'trimf', [1.0 2.5 4.0], 'Name', 'Medium');
            fis = addMF(fis, 'var_Mj', 'trapmf', [3.0 4.0 5 5], 'Name', 'High');
            
            % 添加输出变量: 阈值变化量 ΔC [-5, 5]
            fis = addOutput(fis, [-5 5], 'Name', 'delta_C');
            fis = addMF(fis, 'delta_C', 'trimf', [-5 -5 -2.5], 'Name', 'DL');
            fis = addMF(fis, 'delta_C', 'trimf', [-5 -2.5 0], 'Name', 'DS');
            fis = addMF(fis, 'delta_C', 'trimf', [-2.5 0 2.5], 'Name', 'ZE');
            fis = addMF(fis, 'delta_C', 'trimf', [0 2.5 5], 'Name', 'IS');
            fis = addMF(fis, 'delta_C', 'trimf', [2.5 5 5], 'Name', 'IL');
        end
        
        function fis = add_fuzzy_rules(obj, fis)
            % 向模糊推理系统添加27条模糊规则
            rules = [
                % 第一部分：当 p_i is Low (邻里混乱)
                1 1 1 5 1 1;  % 规则1.1.1: Low Low Low -> IL (大幅提高阈值，让群体快速稳定)
                1 1 2 2 1 1;  % 规则1.1.2: Low Low Medium -> DS (小幅减小阈值，提高敏感性)
                1 1 3 2 1 1;  % 规则1.1.3: Low Low High -> DS
                1 2 1 2 1 1;  % 规则1.2.1: Low Medium Low -> DS
                1 2 2 2 1 1;  % 规则1.2.2: Low Medium Medium -> DS
                1 2 3 1 1 1;  % 规则1.2.3: Low Medium High -> DL
                1 3 1 1 1 1;  % 规则1.3.1: Low High Low -> DL
                1 3 2 1 1 1;  % 规则1.3.2: Low High Medium -> DL
                1 3 3 1 1 1;  % 规则1.3.3: Low High High -> DL
                
                % 第二部分：当 p_i is Medium (邻里适中)
                2 1 1 4 1 1;  % 规则2.1.1: Medium Low Low -> IS
                2 1 2 4 1 1;  % 规则2.1.2: Medium Low Medium -> IS
                2 1 3 3 1 1;  % 规则2.1.3: Medium Low High -> ZE
                2 2 1 3 1 1;  % 规则2.2.1: Medium Medium Low -> ZE
                2 2 2 2 1 1;  % 规则2.2.2: Medium Medium Medium -> DS
                2 2 3 2 1 1;  % 规则2.2.3: Medium Medium High -> DS
                2 3 1 2 1 1;  % 规则2.3.1: Medium High Low -> DS
                2 3 2 1 1 1;  % 规则2.3.2: Medium High Medium -> DL
                2 3 3 1 1 1;  % 规则2.3.3: Medium High High -> DL
                
                % 第三部分：当 p_i is High (邻里有序)
                3 1 1 3 1 1;  % 规则3.1.1: High Low Low -> ZE
                3 1 2 4 1 1;  % 规则3.1.2: High Low Medium -> IS
                3 1 3 4 1 1;  % 规则3.1.3: High Low High -> IS
                3 2 1 4 1 1;  % 规则3.2.1: High Medium Low -> IS
                3 2 2 3 1 1;  % 规则3.2.2: High Medium Medium -> ZE
                3 2 3 5 1 1;  % 规则3.2.3: High Medium High -> IL
                3 3 1 3 1 1;  % 规则3.3.1: High High Low -> ZE
                3 3 2 5 1 1;  % 规则3.3.2: High High Medium -> IL
                3 3 3 5 1 1   % 规则3.3.3: High High High -> IL
            ];
            
            % 添加规则到模糊推理系统
            fis = addRule(fis, rules);
        end
        
        function delta_c = calculate(obj, inputs)
            % 主计算函数
            % 输入: inputs = [p_i, avg_mj, var_mj]
            % 输出: delta_c - 阈值变化量

            % 输入越界处理 - 将输入值限制在定义范围内
            clamped_inputs = inputs;

            % p_i: [0.7, 1.0]
            clamped_inputs(1) = max(0.7, min(inputs(1), 1.0));

            % avg_Mj: [0, 20]
            clamped_inputs(2) = max(0, min(inputs(2), 20));

            % var_Mj: [0, 5]
            clamped_inputs(3) = max(0, min(inputs(3), 5));

            % 记录越界情况（用于调试）
            if any(inputs ~= clamped_inputs)
                fprintf('FLC输入越界: 原始[%.3f, %.3f, %.3f] -> 限制后[%.3f, %.3f, %.3f]\n', ...
                    inputs(1), inputs(2), inputs(3), ...
                    clamped_inputs(1), clamped_inputs(2), clamped_inputs(3));
            end

            delta_c = evalfis(obj.fis, clamped_inputs);
        end
        
        function new_threshold = update_threshold(obj, robot, neighbors, g_state)
            % 更新单个机器人的阈值
            
            % 计算输入变量
            p_i = calculate_local_polarization(robot, neighbors);
            [avg_mj, var_mj] = calculate_motion_saliency_stats(robot, neighbors, g_state);
            
            inputs = [p_i, avg_mj, var_mj];
            
            % FLC计算
            delta_c = obj.calculate(inputs);
            
            % 应用Anti-Windup和更新阈值
            current_c = robot.cj_threshold;
            
            % Enhanced Anti-Windup
            if (current_c >= obj.c_max && delta_c > 0)
                delta_c = 0;
            elseif (current_c <= obj.c_min && delta_c < 0)
                delta_c = 0;
            elseif current_c > (obj.c_max - obj.saturation_margin) && delta_c > 0
                scale_factor = (obj.c_max - current_c) / obj.saturation_margin;
                delta_c = delta_c * scale_factor;
            elseif current_c < (obj.c_min + obj.saturation_margin) && delta_c < 0
                scale_factor = (current_c - obj.c_min) / obj.saturation_margin;
                delta_c = delta_c * scale_factor;
            end
            
            % 更新阈值
            new_c = current_c + delta_c;
            robot.cj_threshold = max(obj.c_min, min(new_c, obj.c_max));
            
            new_threshold = robot.cj_threshold;
        end
    end
end
