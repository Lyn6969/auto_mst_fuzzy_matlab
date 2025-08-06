%% 扩展Weight_cj参数分析脚本
% 
% 目的：在更多的weight_cj值上进行详细测试
% 提供多种测试范围和密度选项

function extended_weight_cj_analysis(test_type)
    if nargin < 1
        test_type = 'comprehensive';
    end
    
    clc;
    fprintf('=== 扩展Weight_cj参数分析 ===\n');
    
    %% 定义不同的测试范围
    test_ranges = struct();
    
    % 快速测试范围
    test_ranges.quick = [25, 50, 100, 150, 200];
    
    % 标准测试范围
    test_ranges.standard = [5, 10, 15, 20, 25, 30, 40, 50, 60, 75, 100, 125, 150, 175, 200, 250, 300];
    
    % 精细测试范围
    test_ranges.fine = [5, 8, 10, 12, 15, 18, 20, 22, 25, 28, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 110, 120, 125, 130, 140, 150, 160, 175, 200, 225, 250, 275, 300];
    
    % 全面测试范围
    test_ranges.comprehensive = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 15, 18, 20, 22, 25, 28, 30, 32, 35, 38, 40, 42, 45, 48, 50, 52, 55, 58, 60, 62, 65, 68, 70, 72, 75, 78, 80, 82, 85, 88, 90, 92, 95, 98, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 175, 180, 185, 190, 195, 200, 210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 320, 340, 360, 380, 400];
    
    % 低值密集测试
    test_ranges.low_dense = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50];
    
    % 高值密集测试
    test_ranges.high_dense = [100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 320, 340, 360, 380, 400, 420, 440, 460, 480, 500];
    
    % 线性关系验证测试
    test_ranges.linear_verify = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200];
    
    %% 显示测试选项
    fprintf('可用的测试类型：\n');
    fprintf('1. quick         - 快速测试 (%d个点)\n', length(test_ranges.quick));
    fprintf('2. standard      - 标准测试 (%d个点)\n', length(test_ranges.standard));
    fprintf('3. fine          - 精细测试 (%d个点)\n', length(test_ranges.fine));
    fprintf('4. comprehensive - 全面测试 (%d个点)\n', length(test_ranges.comprehensive));
    fprintf('5. low_dense     - 低值密集测试 (%d个点)\n', length(test_ranges.low_dense));
    fprintf('6. high_dense    - 高值密集测试 (%d个点)\n', length(test_ranges.high_dense));
    fprintf('7. linear_verify - 线性关系验证 (%d个点)\n', length(test_ranges.linear_verify));
    
    %% 选择测试范围
    if ~isfield(test_ranges, test_type)
        fprintf('\n无效的测试类型，使用标准测试\n');
        test_type = 'standard';
    end
    
    weight_cj_values = test_ranges.(test_type);
    
    fprintf('\n选择的测试类型: %s\n', test_type);
    fprintf('测试点数量: %d\n', length(weight_cj_values));
    fprintf('测试范围: [%d, %d]\n', min(weight_cj_values), max(weight_cj_values));
    fprintf('测试值: %s\n', mat2str(weight_cj_values));
    
    %% 估算运行时间
    estimated_time = estimate_runtime(test_type, length(weight_cj_values));
    fprintf('\n预估运行时间: %.1f分钟\n', estimated_time);
    
    %% 确认是否继续
    if estimated_time > 10
        choice = input('预估运行时间较长，是否继续？(y/n): ', 's');
        if lower(choice) ~= 'y'
            fprintf('分析已取消\n');
            return;
        end
    end
    
    %% 运行分析
    fprintf('\n=== 开始扩展分析 ===\n');
    
    try
        % 根据测试类型调整仿真参数
        [num_sims, enable_par] = adjust_simulation_params(test_type, length(weight_cj_values));
        
        analyze_motion_saliency_parameters_optimized( ...
            'weight_cj_values', weight_cj_values, ...
            'num_simulations', num_sims, ...
            'max_sim_steps', 1200, ...
            'robot_count', 50, ...
            'enable_parallel', enable_par, ...
            'verbose', true);
        
        fprintf('\n✓ 扩展分析完成！\n');
        
        %% 生成特殊分析报告
        generate_extended_analysis_report(test_type, weight_cj_values);
        
    catch ME
        fprintf('✗ 扩展分析失败: %s\n', ME.message);
        fprintf('错误位置: %s (第%d行)\n', ME.stack(1).name, ME.stack(1).line);
    end
    
    fprintf('=== 扩展分析结束 ===\n');
end

function estimated_time = estimate_runtime(test_type, num_points)
    % 估算运行时间（分钟）
    
    % 基准：每个点每次仿真约需要10-15秒
    base_time_per_point = 12; % 秒
    
    switch test_type
        case 'quick'
            num_sims = 5;
        case 'standard'
            num_sims = 10;
        case 'fine'
            num_sims = 10;
        case 'comprehensive'
            num_sims = 10; % 点数多，稍微减少但保持较高精度
        case 'low_dense'
            num_sims = 10;
        case 'high_dense'
            num_sims = 10;
        case 'linear_verify'
            num_sims = 12; % 增加仿真次数提高精度
        otherwise
            num_sims = 10;
    end
    
    % 并行加速比（假设4核）
    parallel_speedup = 3;
    
    total_simulations = num_points * num_sims;
    estimated_time = (total_simulations * base_time_per_point / parallel_speedup) / 60;
end

function [num_sims, enable_par] = adjust_simulation_params(test_type, num_points)
    % 根据测试类型调整仿真参数
    
    switch test_type
        case 'quick'
            num_sims = 5;  % 提高到5次
            enable_par = true;
        case 'standard'
            num_sims = 10; % 使用默认的10次
            enable_par = true;
        case 'fine'
            num_sims = 10; % 使用默认的10次
            enable_par = true;
        case 'comprehensive'
            num_sims = 8;  % 点数多，稍微减少但保持较高精度
            enable_par = true;
        case 'low_dense'
            num_sims = 10; % 重要的低值区域，使用默认次数
            enable_par = true;
        case 'high_dense'
            num_sims = 10; % 使用默认的10次
            enable_par = true;
        case 'linear_verify'
            num_sims = 12; % 验证线性关系需要高精度
            enable_par = true;
        otherwise
            num_sims = 10; % 默认使用10次
            enable_par = true;
    end
    
    % 如果点数过多，强制启用并行
    if num_points > 20
        enable_par = true;
    end
end

function generate_extended_analysis_report(test_type, weight_cj_values)
    % 生成扩展分析报告
    
    timestamp = datetime('now','Format','yyyyMMdd_HHmmss');
    report_filename = sprintf('extended_analysis_report_%s_%s.txt', test_type, timestamp);
    
    fid = fopen(report_filename, 'w');
    if fid == -1
        fprintf('无法创建报告文件\n');
        return;
    end
    
    fprintf(fid, '扩展Weight_cj参数分析报告\n');
    fprintf(fid, '========================\n\n');
    fprintf(fid, '测试类型: %s\n', test_type);
    fprintf(fid, '测试时间: %s\n', datestr(now));
    fprintf(fid, '测试点数量: %d\n', length(weight_cj_values));
    fprintf(fid, '测试范围: [%d, %d]\n', min(weight_cj_values), max(weight_cj_values));
    fprintf(fid, '\n测试值列表:\n');
    
    % 分行显示测试值
    for i = 1:length(weight_cj_values)
        fprintf(fid, '%d', weight_cj_values(i));
        if i < length(weight_cj_values)
            fprintf(fid, ', ');
        end
        if mod(i, 10) == 0
            fprintf(fid, '\n');
        end
    end
    
    fprintf(fid, '\n\n分析建议:\n');
    fprintf(fid, '1. 重点关注线性关系的偏差点\n');
    fprintf(fid, '2. 分析激活率的变化趋势\n');
    fprintf(fid, '3. 识别最优的weight_cj取值范围\n');
    fprintf(fid, '4. 验证理论预测与实际结果的一致性\n');
    
    fclose(fid);
    
    fprintf('扩展分析报告已保存为: %s\n', report_filename);
end

%% 快速调用函数
function run_quick_test()
    extended_weight_cj_analysis('quick');
end

function run_standard_test()
    extended_weight_cj_analysis('standard');
end

function run_comprehensive_test()
    extended_weight_cj_analysis('comprehensive');
end

function run_linear_verify_test()
    extended_weight_cj_analysis('linear_verify');
end