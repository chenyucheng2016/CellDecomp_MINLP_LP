close all
clear all
datas = importdata('CPT1.mat');       
a = datas.data{101};
wblocs = a(1:2,:);
locs(1,:) = wblocs(2,:)+0.1;  % revert ZX to XY, and remove offset

locs(2,:) = wblocs(1,:)+0.7;