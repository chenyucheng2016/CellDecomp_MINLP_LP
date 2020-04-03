% Min. 7/06/2017

%{
Update: 
1. make org CPT global (not sure if it's a good choice)
org CPTs read only, serves as reference

Todo:
TEST UPDATE CPTS
1. able to update CPT according to change of known cues
2. identify which stage we are(how many cues to go)
3. compute EER on known cues


Problems/ questions
1. log base 2 right?

%}

% generate CPTs from input

% initializations
global CPT1 CPT2 CPT3

CPT1 = zeros(1,2);
CPT2 = zeros(4,2);
CPT3 = zeros(8,4);

% build CPTs
CPT1 = buildCPT1(0.43,CPT1);
CPT2 = buildCPT2(0.64,0.69,CPT2);
CPT3 = buildCPT3(0.69,0.37,0.08,0.21,CPT3);

% konwing 0 cue
zeroCueTable = firstRound(CPT1,CPT2,CPT3);

% knowing 1 cue


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

% compute H table from knowing 0 cue
function resTable = firstRound(CPT1,CPT2,CPT3)
% results
orgH = -sum(infoTable(CPT1));
firH = -sum(CPT1*(infoTable(CPT2))');

% sencond H
margP3 = CPT1*CPT2';  
secH = -sum(margP3*(infoTable(CPT3))');
resTable = [orgH  firH  secH];
end


% update CPTs . % NOT TESTED YET
function newCPT1 = updateCPT1(classifiedID)
global CPT1
% newCPT1 =  CPT1;  % checked, it won't update CPT1
newCPT1 = zeros(size(CPT1));
newCPT1(classifiedID) = 1;
end

function newCPT2 = updateCPT2(classifiedID)
global CPT2
newCPT2 = zeros(size(CPT2));
rowNum = ceil(classifiedID/2);
newCPT2(classifiedID,rowNum) = 1;
end

function newCPT3 = updateCPT3(classifiedID)
global CPT3
newCPT3 = zeros(size(CPT3));
rowNum = ceil(classifiedID/2);
newCPT3(classifiedID,rowNum) = 1;
end

% update (actually make copies of) all CPT to adapt known info 
% knownLev = 2. classifiedID = 4: known as black box
function [newCPT1,newCPT2,newCPT3] = updateCPTs(knownLev,classifiedID)
if  knownLev == 1
    newCPT1 = updateCPT1(classifiedID);
elseif knownLev == 2
    CPT1id = ceil(classifiedID/2);
    newCPT1 = updateCPT1(CPT1id);
    newCPT2 = updateCPT2(classifiedID);
elseif knownLev == 3
    newCPT3 = updateCPT3(classifiedID);
    CPT2id = ceil(classifiedID/2);
    newCPT2 = updateCPT2(CPT2id);
    CPT1id = ceil(CPT2id/2);
    newCPT1 = updateCPT1(CPT1id);   
end

end

