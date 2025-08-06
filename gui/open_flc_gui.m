% 打开FLC模糊推理系统的图形化界面
clc;
clear;

% 创建FLC控制器实例
flc = FuzzyLogicController(0.0, 30.0);

% 方法1: 使用fuzzyLogicDesigner打开图形化界面
fuzzyLogicDesigner(flc.fis);

% 方法2: 也可以使用以下命令查看不同组件
% plotfis(flc.fis);                    % 显示FIS结构图
% plotmf(flc.fis, 'input', 1);         % 显示输入1的隶属函数
% plotmf(flc.fis, 'input', 2);         % 显示输入2的隶属函数  
% plotmf(flc.fis, 'input', 3);         % 显示输入3的隶属函数
% plotmf(flc.fis, 'output', 1);        % 显示输出的隶属函数
% gensurf(flc.fis, [1 2], 3);          % 生成控制面（固定第3个输入）
