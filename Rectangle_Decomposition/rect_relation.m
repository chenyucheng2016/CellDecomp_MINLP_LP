function [out, area] =rect_relation(R1, R2)
% R1, R2 are defined by vertex.
%R1(:,i)=[x;y], the ith vertex of R1
%out=-1, R1 is in R2
%out=-2, R2 is in R1
%out=0, intersection, with intersected area >0
%out=1, not intersected with intersected area=0

R1=round(R1*10)/10;
R2=round(R2*10)/10;

out=0;
min_x_a=min(R1(1,:));
max_x_a=max(R1(1,:));
min_y_a=min(R1(2,:));
max_y_a=max(R1(2,:));
min_x_b=min(R2(1,:));
max_x_b=max(R2(1,:));
min_y_b=min(R2(2,:));
max_y_b=max(R2(2,:));
if round(min_x_a*1e3)>=round(min_x_b*1e3) & round(max_x_a*1e3)<=round(max_x_b*1e3) & round(min_y_a*1e3)>=round(min_y_b*1e3) & round(max_y_a*1e3)<=round(max_y_b*1e3)
    out=-1; area=(max_x_a-min_x_a)*(max_y_a-min_y_a); return;
elseif round(min_x_b*1e3)>=round(min_x_a*1e3) & round(max_x_b*1e3)<=round(max_x_a*1e3) & round(min_y_b*1e3)>=round(min_y_a*1e3) & round(max_y_b*1e3)<=round(max_y_a*1e3)
    out=-2; area=(max_x_b-min_x_b)*(max_y_b-min_y_b); return;    
end
a=[min_x_a min_y_a (max_x_a-min_x_a) (max_y_a-min_y_a) ];
b=[min_x_b min_y_b (max_x_b-min_x_b) (max_y_b-min_y_b) ];
area = rectint(a,b);
if area<=(0+0.005)
    out=1;
end