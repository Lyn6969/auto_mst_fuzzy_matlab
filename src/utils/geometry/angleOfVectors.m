
% function: 向量之间的夹角,right(+) or left(-)
function ang = angleOfVectors(baseVector, vector)
    [dir,~] = unitVector(vector);
    [baseDir,~] = unitVector(baseVector);
    M = size(baseDir,2);
    tA = [baseDir; zeros(1,M)];
    tB = [dir; zeros(1,M)];
    tC = cross(tA,tB);
    ang = zeros(1,M);
    rightIndx = find(tC(3,:) < 0);      % right(+)
    ang(1,rightIndx) = acosd(dot(dir(:,rightIndx),baseDir(:,rightIndx)));
    leftIndx = find(tC(3,:) > 0);       % left(-)
    ang(1,leftIndx) = -acosd(dot(dir(:,leftIndx),baseDir(:,leftIndx)));
    
    ang = real(ang);
end

