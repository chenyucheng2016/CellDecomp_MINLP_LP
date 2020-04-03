function X = optimalPath(dist_mat,maxMeasure,omegas,moneyUpper,initFeatureCode)
addpath('OPTI-master/')
starting_node_index = 1;
dist_mat_vector = dist_mat(:);
nodeSize = size(dist_mat,1);
x0 = zeros(nodeSize);
%x0(1,1) = 1;
for i = 1:nodeSize-1
    x0(i,i+1) = 1;
end
x0_v = x0(:);
nodeVec = length(x0_v);
A = [];
for i = 1:nodeSize
    line = [];
    for j = 1:nodeSize
        if i == j
            line = [line,ones(1,nodeSize)];
        else
            line = [line,zeros(1,nodeSize)];
        end
    end
    A = [A;line];
end
B = [];
for i = 1:nodeSize
    line = zeros(1,nodeVec);
    for j = 1:nodeSize
        line(i+(j-1)*nodeSize) = 1;
    end
    B = [B;line];
end
b = ones(nodeSize,1);
Aeq = [0,ones(1,nodeSize-1),zeros(1,(nodeSize-starting_node_index)*nodeSize)];
line = zeros(1,nodeVec);
for i = 1:nodeSize
    line(i+(i-1)*(nodeSize-1)) = 1;
end
%Here we compute the relaven matrics from linear programming !ONCE!
EER_1level = [zeros(1,8),0.5866,0.0595,0.2704,0.0623,0.0786,0.0372,0.0049];
EER_2level = [zeros(1,8),0.5866,0.0595,0.2704,0.0623,0.1158,0.3242,0.0618];
EER_3level = [zeros(1,8),0.5866,0.0595,0.2704,0.0623,0.1158,0.3242,0.2776];
ClassGain = 1./[0.3669,0.1985,0.6191,0.6687,0.4049,0.2269,...
    0.6768,0.2536,0.6799,0.6768,0.5670,0.6494,0.6908,0.6883,0.69295];
alpha = 0.25;
G = [];%information gain matrix for linear programming
C = [];%information cost matrix for linear programming
U = [];
for i = 1:(nodeSize-1)
    initFeature = initFeatureCode(i);
    gains = alpha*ClassGain(initFeature) + (1 - alpha)*[0, EER_1level(initFeature),EER_2level(initFeature),EER_3level(initFeature)];
    line_g = zeros(1,4*(nodeSize-1));
    line_c = zeros(1,4*(nodeSize-1));
    line_u = zeros(1,4*(nodeSize-1));
    line_g(4*(i-1)+1:4*i) = gains;
    line_c(4*(i-1)+1:4*i) = [0,1,2,3];
    line_u(4*(i-1)+1:4*i) = [1,1,1,1];
    G = [G;line_g];
    C = [C;line_c];
    U = [U;line_u];
end
ObjMat = ones(1,nodeSize-1)*(omegas(3)*C - omegas(2)*G);
constraintMat = [ones(1,nodeSize-1)*C;U];%constraints on 1). the mony upper bound 2). each column should exist at most one 1
Aeq = [Aeq;line];
beq = [0;1];
%options = optimoptions('ga','CrossoverFcn', ...  
%{@crossoverintermediate, ratio});
%costFnc(x0_v,dist_mat_vector',maxMeasure,nodeSize,omegas,moneyUpper,ObjMat,constraintMat)
opts = optimoptions('ga','CrossoverFcn',{@crossoverintermediate, 10*ones(1,length(x0_v))},'MutationFcn',@mutationadaptfeasible,'MaxStallGenerations',5e5,'PopulationSize',8000,'FunctionTolerance',1e-1,...
                'MaxGenerations',100,'UseParallel',true,'PlotFcn', @gaplotbestf);
x = ga(@(x)costFnc(x,dist_mat_vector',maxMeasure,nodeSize,omegas,moneyUpper,ObjMat,constraintMat),nodeVec,[A;B;Aeq;-Aeq],[b;b;beq;-beq],[],[],zeros(nodeVec,1),ones(nodeVec,1),@(x)nonlinearConstraint(x,nodeSize),1:nodeVec,opts);
X = reshape(x,[nodeSize,nodeSize])
end
