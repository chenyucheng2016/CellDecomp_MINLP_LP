clear all
close all
addpath('/Users/zhengmin321123/Desktop/cornellGD/pathPlanning(1)/classdef')

cirFur = cirObs([3,3],1,'furniture');
plotObs(cirFur,'c'); %plotting obs
lineX = [0  3]; lineY = [0  3]
tf = ifLineCross(cirFur,lineX,lineY,'show') 

xlim = [0  9.2];
ylim = [0  9.2];
axis equal