function heading = vel2heading_deg( vel )
%	velocity vector -> heading (degree)
    if size(vel,1) == 1
        heading = atan2d(vel(2),vel(1));
    else
        row = size(vel,1);
        heading = zeros(1,row);
        for i = 1:row
            heading(1,i) = atan2d(vel(i,2),vel(i,1));
        end
    end
end

