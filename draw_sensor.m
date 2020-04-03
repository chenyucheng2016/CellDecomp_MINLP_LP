function draw_sensor(sensor, workspace, mkr, face_alpha)
% draw the sensor position and FoV. The FoV is representative of occlusions
% caused by obstacles in the workspace.

if nargin < 3
    mkr = 'ko';
    face_alpha = 0.1;
elseif nargin < 4
    face_alpha = 0.1;
end

% sensor forward direction vector
d = [cos(sensor.theta); sin(sensor.theta)];
FOV_sector = [sensor.x];
for nu_idx = 1:length(sensor.nu_vec)
    point = [cos(sensor.theta + sensor.nu_vec(nu_idx)); ...
        sin(sensor.theta + sensor.nu_vec(nu_idx))];
    FOV_sector = [FOV_sector, sensor.x+sensor.r_max*point];
end
FOV = polyshape(FOV_sector');

% subtract shadow region of each obstacle from the C-target
for m = 1:length(workspace.obstacles)
    B = workspace.obstacles(m);
    UnionSet = [B.Vertices',sensor.x];
    R = convhull(UnionSet(1,:),UnionSet(2,:));

    % construct coverage cone
    VectorSet = B.Vertices' - sensor.x;
    VectorSlopes = atan2(VectorSet(2,:),VectorSet(1,:));
    % cone can't be wider than pi
    if abs(max(VectorSlopes) - min(VectorSlopes)) > pi
        VectorSlopes(VectorSlopes < 0) = ...
            VectorSlopes(VectorSlopes < 0) + 2*pi;
    end
    [~,k1_idx] = max(VectorSlopes);
    [~,k2_idx] = min(VectorSlopes);
    k1 = VectorSet(:,k1_idx);
    k2 = VectorSet(:,k2_idx);
    cone_length = 10000; % 1e10 ~ infty.
    CoverageCone = [sensor.x, sensor.x + cone_length*k1, sensor.x + cone_length*k2];

    % construct shadow region
    R_polygon = polyshape(UnionSet(1,R),UnionSet(2,R));
    K_polygon = polyshape(CoverageCone(1,:),CoverageCone(2,:));
    D_polygon = subtract(K_polygon,R_polygon);

    % if subtraction made a hole in D, then put a small buffer around R
    % until there is no hole
    while D_polygon.NumHoles > 0
        R_polygon = polybuffer(R_polygon, .001);
        D_polygon = subtract(K_polygon,R_polygon);
    end
    
    if m == 1
       PV = subtract(FOV,D_polygon);
    else
        PV = subtract(PV,D_polygon);
    end
end
% subtract all the obstacles from the result
%PV = subtract(PV,workspace.obstacle_union);

plot(PV,'FaceColor','r','FaceAlpha',face_alpha);
axis equal; hold on; %axis(workspace.limits);
plot(sensor.x(1),sensor.x(2),mkr,'MarkerFaceColor','k')


end








