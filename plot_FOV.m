function [out,platform,f] = plot_FOV(x_r, y_r, theta, is_fill,f_level)
% plot the FOV of robot
% is_fill = true / false

l = 0.1/2;
h = 0.1/2;
f = NaN;
platform = NaN;
% x_r = 1;
% y_r = 1;
% theta = pi/4;

x = [x_r+l*cos(theta)+h*sin(theta),x_r+l*cos(theta)-h*sin(theta),...
    x_r-l*cos(theta)-h*sin(theta),x_r-l*cos(theta)+h*sin(theta)];
y = [y_r+l*sin(theta)-h*cos(theta),y_r+l*sin(theta)+h*cos(theta),...
    y_r-l*sin(theta)+h*cos(theta),y_r-l*sin(theta)-h*cos(theta)];

% figure
for i = 1:length(x)-1
    plot(x(i:i+1),y(i:i+1),'b-','LineWidth',1.5)
    hold on
end
x
y
plot([x(1),x(length(x))],[y(1),y(length(y))],'b-','LineWidth',1.5)
hold on
% xlim([0.8,2])
% ylim([0.8,2])

if is_fill
    % fill FOV
    L_fov = 0.45;
    PHI = 100/180*pi;

    x_FOV = zeros(3,1);
    y_FOV = zeros(3,1);
    x_FOV(1) = (x(1)+x(2))/2;
    x_FOV(2) = x_FOV(1) + L_fov*cos(theta-PHI/2);
    x_FOV(3) = x_FOV(1) + L_fov*cos(theta+PHI/2);

    y_FOV(1) = (y(1)+y(2))/2;
    y_FOV(2) = y_FOV(1) + L_fov*sin(theta-PHI/2);
    y_FOV(3) = y_FOV(1) + L_fov*sin(theta+PHI/2);

    f = fill(x_FOV,y_FOV,'g');
    hold on
    if f_level == 1
        alpha(f, 0.06);
    elseif f_level == 2
        alpha(f, 0.20);
    elseif f_level == 3
        alpha(f, 0.50);
    else
        alpha(f,0.90)
        
    end
end

out = true;
end

