function input = MakeObstacles(input, obstacles, robot, N, N_Obstacles)

% xlim([-5, 40]);
% ylim([-15, 25]);

% Each obstacle has x y r.
od = [];

for i = 1:N_Obstacles
    od = [od ...
        obstacles(i).x ...
        obstacles(i).y ...
        obstacles(i).r];
end

od = od.*ones(N+1,1);

input.od = od;
    
end