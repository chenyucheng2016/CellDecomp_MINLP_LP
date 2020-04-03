function [nlout,neq] = nonlinearConstraint(x,nodeSize)
%make sure it is connected and no circle
X = reshape(x,[nodeSize,nodeSize]);
X_corner = zeros(nodeSize);
X_corner(1,1) = 1;
if isequal(X_corner,X)
    X(1,1) = 0;
end
X_nodeSize = X^nodeSize;
nlout = zeros(nodeSize);
for i = 1:nodeSize
    for j = 1:nodeSize
        if X(i,j) == 1
            %connection
            if i ~= 1
                nlout(i,j) = -(sum(X(:,i)) - 1);
            end
        end
    end
end
nlout = [nlout(:);X_nodeSize(:)];
neq = [];
end