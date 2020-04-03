% information of circular furniture
%  ?X, ?Z, the left-bottom corner on the outerwall is (-0.7,-0.1)
cirFurCor = cell(1,4);  % x,y;  10 points to bound polygon; 30 polygons



% (?X, ?Z, dx,dz)
cirFurCor{1} = [7.29402+0.1  -0.0937764+0.7   0.3/2];
cirFurCor{2} = [4.09368+0.1  2.14599+0.7   0.32/2];
cirFurCor{3} = [2.33999+0.1  -0.256474+0.7   0.32/2];
cirFurCor{4} = [0.438551+0.1  0.719622+0.7   0.3/2];


% r = 0.3/2;
% 
% %// center
% c = [ 7.29402+0.1  -0.0937764+0.7];
% 
% pos = [c-r 2*r 2*r];
% disp(pos)