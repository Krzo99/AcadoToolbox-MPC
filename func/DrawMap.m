figure(map)

subplot(2, 3, [1, 2, 3])
hold on;

% Draw obstacles
for obst = obstacles
    DrawCircle(obst.x, obst.y, obst.r, 1);
end

% Draw robot
if exist("robot_drawing", "var")
    delete(robot_drawing)
end
robot_drawing = DrawRobot(robot);

% Draw Ref path
plot(ref_path(1, :), ref_path(2, :), '--', 'color', [.5 .5 .5], 'LineWidth', 1);
% for i = 1:length(ref_path)
%     text(ref_path(1, i), ref_path(2, i) + 1, "" + i);
% end


% Draw Local path
if exist("local_path_drawing", "var")
    delete(local_path_drawing)
end
local_path_drawing = plot(local_path(1, :), local_path(2, :), 'r--', 'LineWidth', 1);

% Draw calculated trajectory
if exist("calc_traj_drawing", "var")
    delete(calc_traj_drawing)
end
calc_traj_drawing = plot(output.x(:, 1), output.x(:, 2), 'g-', 'LineWidth', 1);

% path_history = [path_history [robot.x;robot.y]];
% path_history_speed = [path_history_speed input.x(1, 4)];
% z = zeros(size(path_history_speed));
% col = -path_history_speed;
% surface([path_history(1,:);path_history(1,:)],[path_history(2,:);path_history(2,:)],[z;z],[col;col],...
%         'facecol','no',...
%         'edgecol','interp',...
%         'linew',1);
% colormap('jet');

xlim([-5, 40]);
ylim([-15, 25]);

% subplot(2, 3, 4)
% speed = [speed, output.x(1, 4)];
% plot(speed, 'g');
% title("Speed");
% 
% subplot(2, 3, 5)
% steering = [steering, output.x(1, 5)];
% plot(steering, 'b');
% title("Steering");
% 
% subplot(2, 3, 6)
% yaw_actual = [yaw_actual, mod(robot.yaw, 2*pi)];
% yaw_expected = [yaw_expected, mod(input.y(1, 3), 2*pi)];
% error = yaw_actual - yaw_expected;
% hold on;
% plot(yaw_actual, 'g');
% plot(yaw_expected, 'r');
% plot(error, 'b--');
% title("Yaw")
% legend("robot.yaw", "input.y(1, 3)", "error")

drawnow;  