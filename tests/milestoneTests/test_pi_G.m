% clear all
% B = load('test_B_15');  B = B.B;
n = size(B); n = n(1);

for i = 1:n
    for j = 1:n  % for qi in C
        if I(i,j) == 1  % qi in CB
        sum_qo = 0;
        for iCB = 1:n
            for jCB = 1:n
                if 2*i-iCB<1 || 2*i-iCB>n || 2*j-jCB<1 || 2*j-jCB>n 
                    continue
                else
                sum_qo = sum_qo + B{i,j}{2*i-iCB,2*j-jCB}*I(2*i-iCB,2*j-jCB)*I(iCB,jCB);
                end
            end
        end
        piG{i,j} = sum_qo;
        else 
            piG{i,j} = zeros(n);
        end
        
    end
end
