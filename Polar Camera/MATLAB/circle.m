function [x, y] = circle(x_c, y_c, r, d_theta);
% x_c = center x coordinate
% y_c = center y coordinate
% r = radius
% d_theta = spacing (in radians) between each point
% returns row vectors x, y with sampled values

ang = 0:d_theta:2*pi-d_theta;
xp = r*cos(ang);
yp = r*sin(ang);

x = x_c + xp;
y = y_c + yp;

end

