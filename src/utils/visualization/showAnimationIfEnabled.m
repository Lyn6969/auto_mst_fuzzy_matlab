function showAnimationIfEnabled(G, enableAnimation, pauseTime)
    if nargin < 3
        pauseTime = 0.01;  % 默认暂停时间
    end
    
    if enableAnimation
        drawRobotsMotion_dynamic(G);
        pause(pauseTime);
    end
end