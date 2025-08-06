%% 扩展Weight_cj分析 - 选择界面
% 
% 提供多种测试范围选项的便捷入口

clc;
fprintf('=== Weight_cj扩展分析选择界面 ===\n\n');

fprintf('请选择分析类型：\n');
fprintf('1. 快速测试        (5个点,  5次仿真, ~8分钟)\n');
fprintf('2. 标准测试        (17个点, 10次仿真, ~30分钟)\n');
fprintf('3. 精细测试        (38个点, 10次仿真, ~65分钟)\n');
fprintf('4. 全面测试        (85个点, 8次仿真, ~95分钟)\n');
fprintf('5. 低值密集测试    (35个点, 10次仿真, ~60分钟)\n');
fprintf('6. 高值密集测试    (31个点, 10次仿真, ~55分钟)\n');
fprintf('7. 线性关系验证    (20个点, 12次仿真, ~40分钟)\n');
fprintf('8. 自定义范围\n');
fprintf('9. 退出\n\n');

choice = input('请输入选择 (1-9): ');

switch choice
    case 1
        fprintf('运行快速测试...\n');
        extended_weight_cj_analysis('quick');
        
    case 2
        fprintf('运行标准测试...\n');
        extended_weight_cj_analysis('standard');
        
    case 3
        fprintf('运行精细测试...\n');
        extended_weight_cj_analysis('fine');
        
    case 4
        fprintf('运行全面测试...\n');
        extended_weight_cj_analysis('comprehensive');
        
    case 5
        fprintf('运行低值密集测试...\n');
        extended_weight_cj_analysis('low_dense');
        
    case 6
        fprintf('运行高值密集测试...\n');
        extended_weight_cj_analysis('high_dense');
        
    case 7
        fprintf('运行线性关系验证...\n');
        extended_weight_cj_analysis('linear_verify');
        
    case 8
        fprintf('自定义范围...\n');
        run_custom_analysis();
        
    case 9
        fprintf('退出\n');
        return;
        
    otherwise
        fprintf('无效选择，运行标准测试\n');
        extended_weight_cj_analysis('standard');
end

function run_custom_analysis()
    % 自定义分析范围
    
    fprintf('\n=== 自定义分析范围 ===\n');
    
    % 获取用户输入
    fprintf('请输入weight_cj的测试范围：\n');
    fprintf('格式1: 起始值:步长:结束值 (例如: 10:10:200)\n');
    fprintf('格式2: [值1, 值2, 值3, ...] (例如: [10, 25, 50, 100, 200])\n');
    
    range_input = input('请输入范围: ', 's');
    
    try
        % 尝试解析输入
        if contains(range_input, ':')
            % 格式1: 起始值:步长:结束值
            eval(['weight_cj_values = ' range_input ';']);
        else
            % 格式2: [值1, 值2, ...]
            eval(['weight_cj_values = ' range_input ';']);
        end
        
        % 验证输入
        if ~isnumeric(weight_cj_values) || any(weight_cj_values <= 0)
            error('输入的值必须是正数');
        end
        
        % 排序并去重
        weight_cj_values = unique(sort(weight_cj_values));
        
        fprintf('解析的测试值: %s\n', mat2str(weight_cj_values));
        fprintf('测试点数量: %d\n', length(weight_cj_values));
        
        % 估算时间
        estimated_time = length(weight_cj_values) * 3 * 12 / 180; % 分钟
        fprintf('预估运行时间: %.1f分钟\n', estimated_time);
        
        % 确认
        choice = input('是否继续？(y/n): ', 's');
        if lower(choice) == 'y'
            % 运行自定义分析
            analyze_motion_saliency_parameters_optimized( ...
                'weight_cj_values', weight_cj_values, ...
                'num_simulations', 10, ...
                'max_sim_steps', 1200, ...
                'robot_count', 50, ...
                'enable_parallel', true, ...
                'verbose', true);
            
            fprintf('自定义分析完成！\n');
        else
            fprintf('自定义分析已取消\n');
        end
        
    catch ME
        fprintf('输入格式错误: %s\n', ME.message);
        fprintf('请检查输入格式\n');
    end
end