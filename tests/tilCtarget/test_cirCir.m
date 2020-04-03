

%      loc: [2.6500 0.4000]
%      rad: 0.0781
%      loc: [4.1937 2.8460]
%      rad: 0.1600

% Tested.
% x1 = 1; y1 = 1; r1 = 1; x2 = 9; y2 = 1; r2 = 1.5;
x1 = 2.6500; y1 = 0.4; r1 = 0.0781; x2 = 2.44; y2 = 0.4435; r2 = 0.1600;
[xout,yout] = circcirc(x1,y1,r1,x2,y2,r2);
tf = true; % circle conflict
nan = [NaN NaN];
if ~any(~isnan(xout))  % all are nan
    if dist1(x1,y1,x2,y2)>= max(r1,r2)
    tf = false;
    end
end
disp(tf)



figure;
r = r1;
%// center
c = [x1   y1];
pos = [c-r 2*r 2*r];
rectangle('Position',pos,'Curvature',[1 1],'EdgeColor','k');
hold on
axis equal
r = r2;
%// center
c = [x2   y2];
pos = [c-r 2*r 2*r];
rectangle('Position',pos,'Curvature',[1 1],'EdgeColor','k');

% distance between two points
function d = dist1(x1,y1,x2,y2)
d = sqrt((y2-y1)^2+(x2-x1)^2);
end
