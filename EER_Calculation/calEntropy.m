% This code is to calculate the entropy of the probability P
function H = calEntropy(PM)

    [L,N] = size(PM); % L: number of probable values; N: number of probabilities
    if N == 1
        P = PM;
        indZero = find(P == 0);
        P(indZero) = realmin;
        H = -sum(P.*log2(P));
    else
        H = zeros(N,1);
        for n = 1:N
            P = PM(:,n);
            indZero = find(P == 0);
            P(indZero) = realmin;
            H(n) = -sum(P.*log2(P));
        end
        H = H';
    end

end

