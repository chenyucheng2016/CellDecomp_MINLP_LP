% Min. 7/18/2017

%{
Update: 
1. can calculate all EER with given cueLevel and ID

%}
clear all
% generate CPTs from input

% initializations
global CPT1 CPT2 CPT3  CPT4

buildCPTs();

%%%%============ NOW TESTING


% %%%% konwing 0 cue  (TESTED)
from0 = fromZeroCue()

% %%%% konwing 1 cue 
from1 = fromOneCue(1) %cue1_ID = 1

% %%%% konwing cue 2
from2 = fromTwoCue(1) % cue2_ID = 1;

% %%%% konwing cue 3
from3 = fromThreeCue(1)


%%%%============ NOW TESTING


%%%%%%%%%FUNCTIONS%%%%%%%%%%%

%========= build CPTs======================================================
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

% input  1x8 vector of [0.1,0.1....] 
function CPT4 = buildCPT4(vector,CPT4)
for i = 1:length(vector)
    CPT4(1,i) = vector(i);
    CPT4(2,i) = 1-vector(i);
end
end

function buildCPTs()
global CPT1 CPT2 CPT3  CPT4
CPT1 = zeros(2,1);
CPT2 = zeros(4,2);
CPT3 = zeros(8,4);
CPT4 = zeros(2,8);

% build CPTs
CPT1 = buildCPT1(0.43,CPT1);
CPT2 = buildCPT2(0.64,0.69,CPT2);
CPT3 = buildCPT3(0.69,0.37,0.08,0.21,CPT3);
CPT4 = buildCPT4([0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1],CPT4);
end


%========= basic calculations==============================================
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


% to compute for a=b-1, 
%H(xa|xb)= -sum_a{{ sum_b (p(xa|xb))log(p(xa|xb))*p(xa)}}
function res = twoSum(margA,CPTa,CPTb)
    lenA = size(CPTa,1);
    lenB = size(CPTb,1);
    res = 0;
    for i = 1:lenA
        for j = 1:lenB
            res = res +  PlogP(CPTb(j,i))* margA(i);
        end        
    end
    res = -res;
end

% compute H table from knowing 0 cue USING TWO SUM
function from0 = fromZeroCue()
global CPT1 CPT2 CPT3  CPT4
orgH = -sum(infoTable(CPT1));
marg1 = CPT1;
Hx2gx1 = twoSum(marg1,CPT1,CPT2);  % H(x2|x1)
marg2 = CPT1'*CPT2';   % P(x2|x1)P(x1)
Hx3gx2 = twoSum(marg2,CPT2,CPT3);   % H(x3|x2)
marg3 = CPT1' * CPT2' * CPT3';
Hx4gx3 = twoSum(marg3,CPT3,CPT4);   % H(x4|x3)

from0 = NaN(1,4);
from0(1) = orgH + Hx2gx1 + Hx3gx2 + Hx4gx3;  % H(x1,x2,x3,x4)
from0(2) = Hx2gx1 + Hx3gx2 + Hx4gx3;  %% H(x2,x3,x4)
from0(3) = Hx3gx2 + Hx4gx3;  %% H(x3,x4)
from0(4) = Hx4gx3;  %% H(x4)
end


% ========== with KNOWN CUES===============================================
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

% given current known cue state, update the CPTs as given
function C = givenCPTs(knownLev,classifiedID)
global CPT1 CPT2 CPT3  CPT4

[newCPT1,newCPT2,newCPT3] = updateCPTs(knownLev,classifiedID);
A = {newCPT1,newCPT2,newCPT3};
B = {CPT1,CPT2,CPT3,CPT4};
newTopCPT = 1;
for i = 1:knownLev
    newTopCPT = newTopCPT*A{i}';
end
% this is the new CPT as given
newTopCPT = newTopCPT*B{knownLev+1}';
% all of current CPTs
C = {newTopCPT, B{(knownLev+2):end}};
end

% know cue1
function from1 = fromOneCue(cue1_ID)
global CPT1 CPT2 CPT3  CPT4
C = givenCPTs(1,cue1_ID);  %knownLev,classifiedID
%  has CPT 2,3,4; CPT2 1x4
myCPT2 = C{1}; myCPT3 = C{2}; myCPT4 = C{3};

secH = -sum(infoTable(myCPT2));
marg2 = myCPT2;   % P(x2|x1)P(x1)
Hx3gx2 = twoSum(marg2,myCPT2,myCPT3);   % H(x3|x2)

marg3 = myCPT2 * myCPT3';
Hx4gx3 = twoSum(marg3,myCPT3,myCPT4);   % H(x4|x3)

from1 = NaN(1,4);
from1(2) = secH + Hx3gx2 + Hx4gx3;  %% H(x2,x3,x4)
from1(3) = Hx3gx2 + Hx4gx3;  %% H(x3,x4)
from1(4) = Hx4gx3;  %% H(x4)
end

function from2 = fromTwoCue(cue2_ID)
C = givenCPTs(2,cue2_ID);  %knownLev,classifiedID
%  has CPT 3,4;  CPT3 1x8
myCPT3 = C{1}; myCPT4 = C{2};

thirdH = -sum(infoTable(myCPT3));
Hx4gx3 = twoSum(myCPT3,myCPT3',myCPT4);   % H(x4|x3)

from2 = NaN(1,4);
from2(3) = thirdH + Hx4gx3;  %% H(x3,x4)
from2(4) = Hx4gx3;  %% H(x4)
end

function from3 = fromThreeCue(cue3_ID)
C = givenCPTs(3,cue3_ID);  %knownLev,classifiedID
%  has CPT 4;  1x2
myCPT4 = C{1};
lastH = -sum(infoTable(myCPT4));
from3 = NaN(1,4);
from3(4) = lastH;  %% H(x4)
end
