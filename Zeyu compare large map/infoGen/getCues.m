close all
clear all
datas = importdata('CPT1.mat');       
a = datas.data{101};
x3 = a(3,:);
ys = a(4,:);
cueLev = zeros(1,length(a));
for n = 1:length(cueLev)
    cueLev(n) =  randi([0  3]);  % cue given
end
targetInfo = [x3;ys;cueLev];