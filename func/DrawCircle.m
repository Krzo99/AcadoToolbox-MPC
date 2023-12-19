function p = DrawCircle(x, y, r, linewidth)
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    p = plot(xunit, yunit,'k','LineWidth',linewidth);
end    