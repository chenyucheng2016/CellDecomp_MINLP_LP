% This code is to calculate the EER for the Webots experiments
% Consider the classification problem governed by the joint distribution
% P(Y,X1,X2,X3), where R.V. Y is the class label and X1,...,X3 are features
% The EER is defined as the entropy reduction of distribution of class
% In the current version, one-step EERs are all calculated.
% 
% By Ping at LISC, Jul, 25, 2017


%%
% Load CPTs

% P(X1):
CPT1 = [0.43;   0.57];
% P(X2|X1):
CPT2 = [0.64,   0
        0.36,   0
        0,      0.69
        0,      0.31
        ];    
% P(X3|X2):
CPT3 = [0.69,   0,      0,      0
        0.31,   0,      0,      0
        0,      0.37,   0,      0
        0,      0.63,   0,      0
        0,      0,      0.08,   0
        0,      0,      0.92,   0
        0,      0,      0,      0.21
        0,      0,      0,      0.79
        ];    
% P(Y|X3):        
% CPT4 = [0.9,    0.1,    0.9,    0.9,    0.9,    0.9,    0.9,    0.9 
%         0.1,    0.9,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1
%         ];    
CPT4 = [0.9,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1 
        0.1,    0.9,    0.9,    0.9,    0.9,    0.9,    0.9,    0.9
        ];  
%%
% Get matrix sizes
n_X1 = size(CPT1,1);
n_X2 = size(CPT2,1);
n_X3 = size(CPT3,1);
n_Y = size(CPT4,1);


% Calculate the joint probability P(X2,X1) = P(X2|X1)P(X1):
P_X1 = CPT1;
P_X2X1 = CPT2 .* repmat(P_X1',n_X2,1);
% Calculate the probability P(X2) = \sum_{X1} P(X2,X1): 
P_X2 = sum(P_X2X1,2);
% Calculate the joint probability P(X3,X2) = P(X3|X2)P(X1):
P_X3X2 = CPT3 .* repmat(P_X2',n_X3,1);
% Calculate the probability P(X3) = \sum_{X2} P(X3,X2):
P_X3 = sum(P_X3X2,2);
% Calculate the joint probability P(Y,X3) = P(Y|X3)P(X3):
P_YX3 = CPT4 .* repmat(P_X3',n_Y,1);
% Calculate the probability P(Y) = \sum_{X3} P(Y,X3):
P_Y = sum(P_YX3,2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load the P(Y|X3):
P_Y_X3 = CPT4;
% Load the P(X3|X2):
P_X3_X2 = CPT3;
% Load the P(X2|X1):
P_X2_X1 = CPT2;

% Calculate the conditional probability P(Y|X2) = P(Y|X3) * P(X3|X2):
P_Y_X2 = P_Y_X3 * P_X3_X2;

% Calculate the conditional probability P(Y|X1) = P(Y|X2) * P(X1):
P_Y_X1 = P_Y_X2 * P_X2_X1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Calculate one-step EER:
% 1. Calculate the EER: EER_X1 = H(P(Y)) - H(P(Y|X1)):
EER_X1 = calEntropy(P_Y) - calEntropy(P_Y_X1) * P_X1;

% 2. Calculate the EER: EER_x1X2(X1=x1i, X2) = H(P(Y|X1=x1i)) - H(P(Y|X1=x1i, X2)):

Hx2 = calEntropy(P_Y_X2);
Hx2 = reshape(Hx2,2,length(Hx2)/2);

P_X2_Norm = repmat(P_X1',2,1);
P_X2_Norm = P_X2_Norm(:);
P_X2_Norm = P_X2 ./ P_X2_Norm;
P_X2_Norm = reshape(P_X2_Norm,2,length(P_X2_Norm)/2);


EER_x1X2 = calEntropy(P_Y_X1) - sum(Hx2.*P_X2_Norm);

% 3. Calculate the EER: EER_x1x2X3(X1=x1i, X2=x2j, X3) = H(P(Y|X1=x1i,
% X2=x2j)) - H(P(Y|X1=x1i, X2=x2j, X3)):

Hx3 = calEntropy(P_Y_X3);
Hx3 = reshape(Hx3,2,length(Hx3)/2);

P_X3_Norm = repmat(P_X2',2,1);
P_X3_Norm = P_X3_Norm(:);
P_X3_Norm = P_X3 ./ P_X3_Norm;
P_X3_Norm = reshape(P_X3_Norm,2,length(P_X3_Norm)/2);

EER_x1x2X3 = calEntropy(P_Y_X2) - sum(Hx3.*P_X3_Norm);

% Calculate two-step EER:
% 4. Calculate the EER: EER_X1X2 = H(P(Y)) - H(P(Y|X1,X2)):
EER_X1X2 = calEntropy(P_Y) - calEntropy(P_Y_X2) * P_X2;

% 5. Calculate the EER: EER_x1X2X3 = H(P(Y|x1i)) - H(P(Y|x1i,X2,X3)):
P_X2X3_Norm = repmat(P_X1',4,1);
P_X2X3_Norm = P_X2X3_Norm(:);
P_X2X3_Norm = P_X3 ./ P_X2X3_Norm;
P_X2X3_Norm = reshape(P_X2X3_Norm,length(P_X2X3_Norm)/2,2);

HX3 = calEntropy(P_Y_X3);
HX3 = reshape(HX3,length(HX3)/2,2); 

EER_x1X2X3 = calEntropy(P_Y_X1) - sum(P_X2X3_Norm .* HX3);

% Calculate three-step EER:
% 6. Calculate the EER: EER_X1X2X3 = H(P(Y)) - H(P(Y|X1,X2,X3)):
EER_X1X2X3 = calEntropy(P_Y) - calEntropy(P_Y_X3) * P_X3;

