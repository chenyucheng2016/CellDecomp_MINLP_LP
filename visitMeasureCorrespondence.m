function [c,ceq] = visitMeasureCorrespondence(x)
%function c = visitMeasureCorrespondence(x)
ceq = [];
var_length = length(x)/2;
y = x(1:var_length);
u = x(var_length+1:end);
c = zeros(var_length,1);
for i = 1:var_length
    c(i) = -(y(i) - 1)*u(i);
end
end