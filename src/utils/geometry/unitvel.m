
% function: 向量转单位向量
function uvel = unitvel(vel)
% vel -> unit velocity vector ([U_Vel_X U_Vel_Y])    
    uvel = vel;
    if norm(vel) ~= 0
        uvel = vel/norm(vel);
    end
end
