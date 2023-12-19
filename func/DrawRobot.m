function robotDrawing = DrawRobot(robot)
    
    centre = [robot.x; robot.y];

    A = [-robot.length / 2;  robot.width / 2];
    B = [ robot.length / 2;  robot.width / 2];
    C = [ robot.length / 2; -robot.width / 2];
    D = [-robot.length / 2; -robot.width / 2];
    E = [robot.length; 0];

    M = [A B C D E];

    % Rotate
    R = [cos(robot.yaw) -sin(robot.yaw);sin(robot.yaw) cos(robot.yaw)];
    M = R*M + centre;

    robotDrawing(1) = plot(polyshape(M(1, 1:end-1), M(2, 1:end-1)));
    robotDrawing(1).FaceColor = "r";
    robotDrawing(2) = plot([robot.x M(1, end)], [robot.y M(2, end)], 'b', LineWidth=3);
    
end    