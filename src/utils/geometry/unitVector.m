function [unit_vec, magnitude] = unitVector(vec)
    % 将向量转换为单位向量
    % 输入: vec - 输入向量矩阵，每列为一个向量
    % 输出: unit_vec - 单位向量矩阵
    %       magnitude - 原向量的模长

    magnitude = sqrt(sum(vec.^2, 1));
    
    % 避免除零
    magnitude(magnitude == 0) = 1;
    
    unit_vec = vec ./ magnitude;
    
    % 如果原向量为零向量，则单位向量也为零向量
    zero_indices = sqrt(sum(vec.^2, 1)) == 0;
    unit_vec(:, zero_indices) = 0;
end
