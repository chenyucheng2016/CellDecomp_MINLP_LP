function flag = is_collisionfree(obs_ind,obs, xy1,xy2)
% check if the line from xy1 to xy2 is collision free
% obs: recFur
flag = true;
x = [xy1(1), xy2(1)];
y = [xy1(2), xy2(2)];

angle = atan2(y(2)-y(1), x(2)-x(1));
% offset = 0.1/cos(angle);
for i = 1:length(obs)
    v = obs(obs_ind(i)).vertices;
    xbox = v(1,:);
    xbox = [xbox, xbox(1)];
    ybox = v(2,:);
    ybox = [ybox, ybox(1)];
    flag = flag & isempty(polyxpoly(x,y,xbox,ybox));
    flag = flag & isempty(polyxpoly(x-0.1*sin(angle),y+0.1*cos(angle),xbox,ybox));
    flag = flag & isempty(polyxpoly(x+0.1*sin(angle),y-0.1*cos(angle),xbox,ybox));
end
end

