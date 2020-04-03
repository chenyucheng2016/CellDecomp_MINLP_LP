% information of rectangle furniture
%  ?X, ?Z, the left-bottom corner on the outerwall is (-0.7,-0.1)
recFurCor = cell(1,10);  % x,y;  10 points to bound polygon; 30 polygons



% (?X, ?Z, dx,dz)
recFurCor{1} = convert(0.022521,1.16671,0.8,1.8);
recFurCor{2} = convert(0.0526071,6.07178,1,1.1);
recFurCor{3} = convert(4.12+0.25-0.12,2.59116+0.11,0.5,0.8);
recFurCor{4} = convert(4.45348-0.1,8.5,0.7,0.7);

recFurCor{5} = convert(6.35183,6.91049,1.8,1);

recFurCor{6} = convert(6.35183+0.9,6.91049,0.5,0.5);  % up chair
recFurCor{7} = convert(6.35183-0.9,6.91049,0.5,0.5);  % down chair
recFurCor{8} = convert(6.35183,6.91049-0.5,0.5,0.5);  % left 
recFurCor{9} = convert(6.35183,6.91049+0.5,0.5,0.5);  % right 

% rotation one
input =  [7.6370    7.6370    8.7370    8.7370; 0.6740    1.6740    1.6740    0.6740];  
recFurCor{10}= rot(input,0.6545);
disp(recFurCor{10});

% convert the center(x,z)and width x length of object (dx,dz), 
% transform from ( ?Z, ?X)to vertices(?X,?Y) and remove offsets
% only works with no rotations
function myCoors = convert(x,z,dx,dz)
% offset
osX = -0.7;
osZ = -0.1;
% in matlab coors
    leftX = z-dz/2-osZ;   % in weBot(leftZ)
    rightX = z+dz/2-osZ;  % in weBot(rightZ)
    upY = x+dx/2-osX;   % in weBot(upX)
    downY = x-dx/2-osX;   % in weBot(downX) 
    myCoors = [leftX  leftX  rightX  rightX;downY upY  upY  downY];
end


% convert the center(x,z)and width x length of object (dx,dz), and rotation
% angle in rad,CCW+
% transform from ( ?Z, ?X)to vertices(?X,?Y) and remove offsets
function coorRot = rot(input, angle)

% Define A
%  input =  [7.6370    7.6370    8.7370    8.7370; 0.6740    1.6740    1.6740    0.6740];  % coors of rec in matlab, without rotation
% A = [input(1,:) input(1,1);input(2,:) input(2,1); 1 1 1 1 1 ];
A = [-0.55   -0.55   0.55   0.55  -0.55;-0.5   0.5   0.5   -0.5  -0.5; 1 1 1 1 1];

% Define Translation Matrix
trans = @(x,y,z) repmat([x; y; z],[1 5]);

% Define Rotation Matrix
se2 = @(x, y, theta) [
    cos(theta), -sin(theta), x;
    sin(theta), cos(theta), y;
    0,        0,           1];

x = (A(1,4)-A(1,1))/2 +A(1,1);
y = (A(2,3)-A(2,1))/2 +A(2,1);
% x = 0;
% y = 0;
% Calculate Rotated Rect
B = se2(x,y,angle) * (A - trans(0,0,0) ) + trans(0,0,0);
%CCW+ 
%pi/2- 0.6545
% Plot Rectangles
figure; plot(A(1,:),A(2,:),'b')
hold on;
plot(B(1,:),B(2,:),'r')

plot(x,y,'*')

axis equal

x = (input(1,4)-input(1,1))/2 +input(1,1);
y = (input(2,3)-input(2,1))/2 +input(2,1);
C = [input(1,:) input(1,1);input(2,:) input(2,1)];
D = [x+B(1,:); y+B(2,:)];
plot(C(1,:),C(2,:),'b');
plot(D(1,:),D(2,:),'r');
coorRot = D;

end
