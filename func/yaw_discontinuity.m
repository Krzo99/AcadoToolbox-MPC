function input = yaw_discontinuity(input)
% Remap local path angles to angles under which the robot will see them.
%https://www.desmos.com/calculator/qy0d1wqbr9

for i = 1 : size(input.y, 1)
    dif = mod(input.y(i, 3) - input.x(i, 3), 2*pi);
    if dif > pi 
        dif = dif - 2*pi;
    end
   
    input.y(i, 3) = input.x(i, 3) + dif;
end
