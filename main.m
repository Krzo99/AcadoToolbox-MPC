% View: https://github.com/lee-ck/Model-Predictive-Control

% close all;
clear all;
clc;

addpath(genpath('./func/'))
addpath(genpath('./class/'))
addpath(genpath('./ACADO/'))

%% ------------- Config ---------------
LookaheadDistance = 19;
N = LookaheadDistance + 1;
dt = 0.5;
T = 500;
N_Obstacles = 5;

RECOMPILE_ACADO = false;    % Set to 1 everytime you change N or dt.
if RECOMPILE_ACADO
    ACADO_main(N, dt);
    error("done, disable recompile.")
end

robot = Robot(0, 0, 0, 1, 1);
obstacles = [...
    Obstacle(15, 11, 3)...
    Obstacle(21, 8, 2)...
    Obstacle(33, 5, 4)...
    Obstacle(10, -4, 2)...
    Obstacle(-3, -3, 1)...
    ];

%% Setup
map = figure(1);
clf(map)
ref_path = MakePath();

% Initial condition
X0 = [robot.x robot.y robot.yaw...
      0 0 0];                           % velocity steer acceleration
U0 = [0 0];                             % Initial inputs

X = repmat(X0, N+1, 1);                 % Initial + horizon Diff. States
U = repmat(U0, N, 1);                   % Inputs for all horizon points.

Xref = X0;  
Xref = repmat(Xref,N,1);
Uref = [0 0];
Uref = repmat(Uref,N,1);

input.u = U;
input.x = X;
input.x0 = X0;
input.y = [Xref(1:N,:) Uref zeros(N, 1)];
input.yN = Xref(N,:);

% Weight matrixes
input.W  = diag([1,   1,  1,...  % x y yaw
                 0,   0,   0,...    % velocity steer acceleration
                 1,   1,   0.8]);     % del_steer del_acceleration obstacles

input.WN  = diag([1, 1, 1,...  % x y yaw
                 0, 0, 0]);        % velocity steer acceleration


% Telemetry
speed = [];
steering = [];
yaw_actual = [];
yaw_expected = [];

path_history = [];
path_history_speed = [];


%% Simulation loop

% How many loops have we had 0 velocity. Push it back, to try to recover.
zero_velocity_times = 0;

time = 0;
while time < T

    % Set reference path.
    local_path = GetLocalPath(robot, ref_path, LookaheadDistance, N);
    input.y(:, 1:2) = local_path';
    
    for i = 1:N-1
        Ay = local_path(2, i+1) - local_path(2, i);
        Ax = local_path(1, i+1) - local_path(1, i);
        input.y(i, 3) = atan2(Ay, Ax);
    end
    
    input.y(N, 3) = input.y(N-1, 3);
    input = yaw_discontinuity(input);
    input.yN(1, 1:3) = input.y(end, 1:3);
    
    % Make OnlineData obstacles
    input = MakeObstacles(input, obstacles, robot, N, N_Obstacles);

    output = ACADO_robot(input);

    if (abs(output.u(1, 2)) < 0.01 )
        zero_velocity_times = zero_velocity_times + 1;
        if (zero_velocity_times > 20)
            disp("Recovering!")
            output.u(:, 2) = output.u(:, 2) + 1;
            zero_velocity_times = 0;
        end
    else
        zero_velocity_times = 0;
    end
    
    integrate_input.x = input.x(1, :)';
    integrate_input.u = output.u(1, :)';
    states = integrate_robot(integrate_input);
    
    % Update state
    input.x0 = states.value';
    input.x = [input.x0; output.x(2:end,:)];
    input.u = output.u;

    robot.x = states.value(1);
    robot.y = states.value(2);
    robot.yaw = states.value(3);

    DrawMap;

    time = time + dt;

end













