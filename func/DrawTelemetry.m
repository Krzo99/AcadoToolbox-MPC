
figure(telemetry);

subplot(2, 3, 1)
if (isvalid(vel_cmd_draw))
    addpoints(vel_cmd_draw, vel_cmd_x, integrate_input.u(2))
    drawnow;
    vel_cmd_x = vel_cmd_x + 1;
end