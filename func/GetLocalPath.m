function out = GetLocalPath(robot, ref_path, lookaheadPoints, N)
    [~, closest_ref_path_index] = min(sqrt((robot.x - ref_path(1, :)).^2 + (robot.y - ref_path(2, :)).^2));
    local_goal_index = closest_ref_path_index + lookaheadPoints;

    if (local_goal_index > size(ref_path, 2))
        out = [ref_path(:, closest_ref_path_index:end) ref_path(:, 1:(local_goal_index - size(ref_path, 2)))];
        local_goal_index = local_goal_index - size(ref_path, 2);
    else
        out = ref_path(:, closest_ref_path_index:local_goal_index);
    end
    
    %out = [linspace(robot.x , ref_path(1, local_goal_index), N); linspace(robot.y, ref_path(2, local_goal_index), N)];
    %out = [linspace(ref_path(1, closest_ref_path_index) , ref_path(1, local_goal_index), N); linspace(ref_path(2, closest_ref_path_index), ref_path(2, local_goal_index), N)];
    
end    