function unit_vec = heading2unit_deg(heading)
    % 将角度（度）转换为单位向量
    % 输入: heading - 角度（度）
    % 输出: unit_vec - 单位向量 [x, y]
    
    unit_vec = [cosd(heading), sind(heading)];
end
