% Min. 7/24/2017

%{
Update: calculate single-step
trueTable is P(Y|...) summary result

%}

clear all
% generate CPTs from input

% initializations
global CPT1 CPT2 CPT3  CPT4

buildCPTs();

%%%%============ NOW TESTING
trueTable = NaN(4,8);

% I. P(Y)
PYtrue = 0;
PYfalse = 0;
for i = 1:2
    for j = 1:4
        for k = 1:8
            PYtrue = PYtrue + CPT1(i)*CPT2(j,i)*CPT3(k,j)*CPT4(1,k);
            PYfalse = PYfalse + CPT1(i)*CPT2(j,i)*CPT3(k,j)*CPT4(2,k);
        end
    end
end
PY = [PYtrue  PYfalse];
trueTable(1,1) = PYtrue;

% II. P(Y|X1=x1,i) (i = {1,2})
PYgx11_true = 0; PYgx11_false = 0;
PYgx12_true = 0; PYgx12_false = 0;

for j = 1:4
    for k = 1:8
        i = 1;
        PYgx11_true = PYgx11_true + CPT2(j,i)*CPT3(k,j)*CPT4(1,k);
        PYgx11_false = PYgx11_false + CPT2(j,i)*CPT3(k,j)*CPT4(2,k);
        i = 2;
        PYgx12_true = PYgx12_true + CPT2(j,i)*CPT3(k,j)*CPT4(1,k);
        PYgx12_false = PYgx12_false + CPT2(j,i)*CPT3(k,j)*CPT4(2,k);        
    end
end
trueTable(2,1) = PYgx11_true; trueTable(2,2) = PYgx12_true;

% III. P(Y|X2 = 2,j; X1 = x1.i) =  P(Y, X2 = 2,j|X1 = x1.i)/ P(X2 = 2,j|X1 = x1.i)
PYgX2X1_true = zeros(2,4);% P(Y=true|X2 = 2,j; X1 = x1.i)
PYgX2X1_false = zeros(2,4);
tops_true = zeros(2,4); % P(Y, X2 = 2,j|X1 = x1.i)
tops_false = zeros(2,4);
for i = 1:2
    for j = 1:4
        temp = 0; temp2 = 0;
        for k = 1:8
            temp = temp+CPT2(j,i)*CPT3(k,j)*CPT4(1,k);
            temp2 = temp2+CPT2(j,i)*CPT3(k,j)*CPT4(2,k);
        end
        if temp ==0
            PYgX2X1_true(i,j) = 0;
            PYgX2X1_false(i,j) = 0;
            tops_true(i,j) = 0;
            tops_false(i,j) = 0;
        else
            PYgX2X1_true(i,j) = temp/CPT2(j,i);
            PYgX2X1_false(i,j) = temp2/CPT2(j,i);
            tops_true(i,j) = temp;
            tops_false(i,j) = temp2;
        end
%         if temp2 ==0
%             tops2(i,j) = 0;
%         else
%             tops2(i,j) = temp2/CPT2(j,i);
%         end
    end
end
temp = sum(PYgX2X1_true,1);
trueTable(3,1:4) = temp;

tops_true = sum(tops_true,1);
tops_false = sum(tops_false,1);

PYgX2X1_true = sum(PYgX2X1_true,1);
PYgX2X1_false = sum(PYgX2X1_false,1);
% IV P(Y|X3 = 3,k) CPT4(1,k) for true and CPT4(2,k) for false
trueTable(4,1:8) = CPT4(1,:);
% P(X)
PX2 = zeros(1,4);
PX3 = zeros(1,8);

% % P(X2)
for j = 1:4
    temp = 0;
    for i = 1:2
        temp = temp + CPT1(i)*CPT2(j,i);
    end
    PX2(j) = temp;
end

% % P(X3)
for k = 1:8
    temp = 0;
    for j = 1:4        
        for i = 1:2
            temp = temp + CPT1(i)*CPT2(j,i)*CPT3(k,j);
        end       
    end
    PX3(k) = temp;
end

% I.H(Y) % CHECKEED
HY = 0;
for i = 1:length(PY)
    HY = HY + PlogP(PY(i));
end
HY = -HY;

%  II. H(Y|X1)

% II. H(Y|X1) checked
HY_x11 = -CPT1(1)*(PlogP(PYgx11_true)+PlogP(PYgx11_false));
HY_x12 = -CPT1(2)*(PlogP(PYgx12_true)+PlogP(PYgx12_false));
HYgX1 = HY_x11+HY_x12;



% III. H(Y|X2 = x2,j;X1 = x1,i) = -sum_Y(P(Y,X2 = x2,j|X1 = x1,i)log2(P(Y|X2 = x2,j;X1 = x1,i)))
% % tops =  P(Y=true, X2 = 2,j|X1 = x1.i)
all_true = sum(PYgX2X1_true,1); % P(Y=true| X2 = 2,j,X1 = x1.i)


HYgx2x1 = zeros(1,4); 

for j=1:2
    HYgx2x1(j) = CPT2(j,1)*calcEntropy(all_true(j));
end

for j=3:4
    HYgx2x1(j) = CPT2(j,2)*calcEntropy(all_true(j));
end
HYgx2x11 = sum(HYgx2x1(1:2)); % H(Y|X2;given X1 = x1,1)
HYgx2x12 = sum(HYgx2x1(3:4));% H(Y|X2;given X1 = x1,2)
 


% %IV. H(Y|X3x2x1)
HYgx3x2x1 = zeros(1,4); 
tempCPT3 = sum(CPT3,2);
for p = 1:4
    k1 = 2*p-1;
    k2 = 2*p;
    HYgx3x2x1(p) = tempCPT3(k1)*calcEntropy(CPT4(1,k1))+tempCPT3(k2)*calcEntropy(CPT4(1,k2));
end

function res = calcEntropy(Y_ture)
res = -(PlogP(Y_ture)+PlogP(1-Y_ture));
end




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
CPT4 = buildCPT4([0.9,0.1,0.9,0.9,0.9,0.9,0.9,0.9],CPT4);
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
