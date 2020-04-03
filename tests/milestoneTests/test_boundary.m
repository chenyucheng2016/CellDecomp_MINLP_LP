% global A2d mapLimit
% myStart = mapLimit(1);
% myEnd = mapLimit(2);
% 
% step = (myEnd-myStart)/n;
% xs = linspace(myStart,myEnd,n);
% ys = linspace(myStart,myEnd,n);
% B = zeros(n,n);
% 
% P = B3d{3};
figure;

% x = gallery('uniformdata',30,1,1);
% y = gallery('uniformdata',30,1,10);
% plot(x,y,'.')
% xlim([-0.2 1.2])
% ylim([-0.2 1.2]) 
% k = boundary(x,y);
% hold on;
% plot(x(k),y(k));

P = [1.001  1  4  6  1; 1  1  5  1  2];  %;1  1  1  1  1
x = (P(1,:))'; y = (P(2,:))';
j = boundary(x,y);
plot(x,y,'.')
hold on;
plot(x(j),y(j));

% k = boundary(P);
% hold on
% trisurf(k,P(:,1),P(:,2),P(:,3),'Facecolor','red','FaceAlpha',0.1)