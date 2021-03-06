% Min. 7/12/2017

%{
Update: 
1. changed dimension of CPT1 from 1,2 to 2,1
2. updateCPTs TESTed

Todo:
TEST UPDATE CPTS
3. compute EER on known cues

Problems/ questions
1. log base 2 right?

%}

% generate CPTs from input

% initializations
global CPT1 CPT2 CPT3

CPT1 = zeros(2,1);
CPT2 = zeros(4,2);
CPT3 = zeros(8,4);

% build CPTs
CPT1 = buildCPT1(0.43,CPT1);
CPT2 = buildCPT2(0.64,0.69,CPT2);
CPT3 = buildCPT3(0.69,0.37,0.08,0.21,CPT3);

% konwing 0 cue
zeroCueTable = zeroKnownRes(CPT1,CPT2,CPT3)

% knowing 1 cue

%%%%% NOW TESTING
% [newCPT1,newCPT2,newCPT3] = updateCPTs(1,2); %knownLev,classifiedID
% resTable = oneKnownRes(newCPT1,newCPT2,newCPT3)
% 
% [newCPT1,newCPT2,newCPT3] = updateCPTs(1,1); %knownLev,classifiedID
% resTable = oneKnownRes(newCPT1,newCPT2,newCPT3)
[newCPT1,newCPT2,newCPT3] = updateCPTs(2,3); %knownLev,classifiedID
resTable = twoKnownRes(newCPT1,newCPT2,newCPT3)
%%%%%

%%%%%%%%%FUNCTIONS
% input value of CPT1(1,1)
function CPT1 = buildCPT1(a,CPT1) 
CPT1(1) = a;
CPT1(2) = 1-a;
end

% input value of a = CPT2(1,1) . b = CPT2(3,2)
function CPT2 = buildCPT2(a,b,CPT2)
CPT2(1,1) = a;
CPT2(2,1) = 1-a;
CPT2(3,2) = b;
CPT2(4,2) = 1-b;
end

% input value of a = CPT3(1,1) . b = CPT3(3,2) c = CPT3(5,3) . d = CPT3(7,4)
function CPT3 = buildCPT3(a,b,c,d,CPT3)
CPT3(1,1) = a;
CPT3(2,1) = 1-a;
CPT3(3,2) = b;
CPT3(4,2) = 1-b;
CPT3(5,3) = c;
CPT3(6,3) = 1-c;
CPT3(7,4) = d;
CPT3(8,4) = 1-d;
end

% calulates P*log(P)
function res = PlogP(p)
if p == 0
    res = 0;
else
    res = p*log2(p);
end
end

% create tables with same size as input CPT, but with PlogP as value
function infoT = infoTable(CPTX)
infoT = zeros(size(CPTX));
[m,n] = size(CPTX);
for i = 1:m
    for j = 1:n
        infoT(i,j) = PlogP(CPTX(i,j));
    end
end

end

%%% compute H table from knowing 0 cue
function resTable = zeroKnownRes(CPT1,CPT2,CPT3)
% results
orgH = -sum(infoTable(CPT1));
firH = -sum(CPT1'*(infoTable(CPT2))');

% sencond H
margP3 = CPT1'*CPT2';  
secH = -sum(margP3*(infoTable(CPT3))');
resTable = [orgH  firH  secH];
end

% update CPTs . 
function newCPT = updateCPT(classifiedID,CPTX)
newCPT = zeros(size(CPTX));
rowNum = ceil(classifiedID/2);
if rowNum == 0
    rowNum = 1;
end
newCPT(classifiedID,rowNum) = 1;
end

% update (actually make copies of) all CPT to adapt known info 
% knownLev = 2. classifiedID = 4: known as black box
function [newCPT1,newCPT2,newCPT3] = updateCPTs(knownLev,classifiedID)
global CPT1 CPT2 CPT3
newCPT1 = CPT1;
newCPT2 = CPT2;
newCPT3 = CPT3;
if  knownLev == 1
    newCPT1 = updateCPT(classifiedID,CPT1);
elseif knownLev == 2
    CPT1id = ceil(classifiedID/2);
    newCPT1 = updateCPT(CPT1id,CPT1);
    newCPT2 = updateCPT(classifiedID,CPT2);
elseif knownLev == 3
    newCPT3 = updateCPT(classifiedID,CPT3);
    CPT2id = ceil(classifiedID/2);
    newCPT2 = updateCPT(CPT2id,CPT2);
    CPT1id = ceil(CPT2id/2);
    newCPT1 = updateCPT(CPT1id,CPT1);   
end

end

%%% compute H table from knowing first cue
function resTable = oneKnownRes(CPT1,CPT2,CPT3)
% results
firH = -sum(CPT1'*(infoTable(CPT2))');  % sum of H of second cue
% second H
margP3 = CPT1'*CPT2';  
secH = -sum(margP3*(infoTable(CPT3))');
resTable = [firH  secH];
end

%%% compute H table from knowing second cue
function resTable = twoKnownRes(CPT1,CPT2,CPT3)

% second H
margP3 = CPT1'*CPT2';  
secH = -sum(margP3*(infoTable(CPT3))');
resTable = [secH];
end

