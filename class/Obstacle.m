classdef Obstacle
   properties
      x {mustBeNumeric}
      y {mustBeNumeric}
      r {mustBeNumeric}
   end
   methods        
    function obj = Obstacle(x, y, r)
        obj.x = x;
        obj.y = y;
        obj.r = r;
    end
end
end