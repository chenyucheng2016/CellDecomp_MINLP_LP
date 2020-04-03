waypoints = [0,0,0;
             1,-0.2,0;
             1.4,2.1,pi/2;
             -0.7,2.9,pi;
             0.3,3.9,pi/2;
             2.7,4.1,0;
             3.8,3.0,0;
             5.9,3.7,0;
             5.5,2.6,3*pi/2;
             3.7,2.4,pi;
             3.1,2.0,3*pi/2;
             5.6,0.7,3*pi/2;
             4.6,0.2,3*pi/2];
num = size(waypoints,1);
dist = 0;
rot_ang = 0;
for i = 2:num
    prev_pose = waypoints(i-1,:);
    cur_pose = waypoints(i,:);
    pointTo = abs(wholeAngle(atan2(cur_pose(2) - prev_pose(2), cur_pose(1) - prev_pose(1))) - prev_pose(3));
    toCur = abs(wholeAngle(atan2(cur_pose(2) - prev_pose(2), cur_pose(1) - prev_pose(1))) - cur_pose(3));
    rot_ang = rot_ang + pointTo + toCur;
    dist = dist + norm(prev_pose(1:2) - cur_pose(1:2));
end



function angle_new = wholeAngle(angle)
if angle < 0
    angle_new = angle + 2*pi;
else
    angle_new = angle;
end
end
