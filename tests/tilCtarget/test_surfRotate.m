close all
% 
% newVert =[7.6370    7.6370    8.7370    8.7370    7.6370;
%     0.6740    1.6740    1.6740    0.6740    0.6740];
% h = surf(newVert(1,:),newVert(2,:), -pi*ones(5));
% direction = [0 0 1];
% rotate(h,direction,45)
% hold on 
% surf(newVert(1,:),newVert(2,:), -pi*ones(5));

newVert =[7.6370    7.6370    8.7370    8.7370    7.6370;
    0.6740    1.6740    1.6740    0.6740    0.6740];
plot3(newVert(1,:),newVert(2,:),-pi*ones(5));
hold on
newVert =[8.0550    7.4463    8.3190    8.9277    8.0550;
    0.4425    1.2359    1.9055    1.1121    0.4425];
plot3(newVert(1,:),newVert(2,:),-pi*ones(5));
xlim([0  9.2])
ylim([0  9.2])  
grid on