function out = MakePath()
    % th = 0:pi/500:2*pi;
    % xunit = 30 * cos(th) + 50;
    % yunit = 30 * sin(th) + 50;
    % 
    % out = [xunit;yunit];

    points = [0 5 10 10 20 25 30 35 35 30 30 20 10  0  -5   0 ;...
              0 0 0  10 10 15 15 10 5  0  -5 -5 -5 -5  -2.5 0];
    % points = points .* 2.2;
    % points(2, :) = points(2, :) + 30;
    % points(2, :) = points(2, :) .* 1.5;
    % points(1, :) = points(1, :) + 20;

    % Create smooth spline
    px = [points(1,1) points(1,:) points(1,end)];
    py = [points(2,1) points(2,:) points(2,end)];

    pp = spcrv([px;py],4);

    out = pp;
    
end