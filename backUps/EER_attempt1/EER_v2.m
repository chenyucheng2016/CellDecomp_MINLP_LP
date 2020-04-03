% Min. 7/06/2017

%{
Update: 
1. PlogP() . function
2. now have result for "from 0 cue"

Todo:
1. able to update CPT according to change of known cues
2. identify which stage we are(how many cues to go)

Problems/ questions
1. log base 2 right?

%}

% generate CPTs from input

% initializations
CPT1 = zeros(1,2);
CPT2 = zeros(4,2);
CPT3 = zeros(8,4);

% build CPTs
CPT1 = buildCPT1(0.43,CPT1);
CPT2 = buildCPT2(0.64,0.69,CPT2);
CPT3 = buildCPT3(0.69,0.37,0.08,0.21,CPT3);

% konwing 0 cue
resTable = firstRound(CPT1,CPT2,CPT3);


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
