classdef Robot
   properties
      x {mustBeNumeric}
      y {mustBeNumeric}
      yaw {mustBeNumeric}

      width {mustBeNumeric}
      length {mustBeNumeric}

      v {mustBeNumeric}
      steer_angle {mustBeNumeric}

   end
   methods        
        function obj = Robot(x_init, y_init, yaw_init, width, length)
            obj.x = x_init;
            obj.y = y_init;
            obj.yaw = yaw_init;
            obj.width = width;
            obj.length = length;

            obj.v = 0;
            obj.steer_angle = 0;
        end
end
end