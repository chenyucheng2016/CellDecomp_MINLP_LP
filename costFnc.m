function cost = costFnc(x,dist_vec,maxMeasure,nodeSize,omegas, moneyUpper,objMat,constraintMat)
X = reshape(x,[nodeSize,nodeSize]);
[~,col] = find(X);
if length(col)~=length(unique(col)) || isequal(col,1)
    cost = dist_vec*x';
    return;
else
    %here is how we formulate the linear/convex optimization problem
    options = optimoptions('linprog','Algorithm','dual-simplex','Display','off');
    constraintRhs = zeros(nodeSize-1,1);
    constraintRhs(col - 1) = 1;
    constraintMat = [constraintMat;-constraintMat(col,:)];
    constraintRhs = [constraintRhs;-ones(length(col),1)];
    measure = linprog(objMat,constraintMat,[moneyUpper;constraintRhs],[],[],zeros(4*(nodeSize-1),1),ones(4*(nodeSize-1),1),options);
    cost = omegas(1)*dist_vec*x' + objMat * measure;
end
end