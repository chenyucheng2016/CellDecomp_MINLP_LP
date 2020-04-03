function h = computeEntropy(p)
h = -(p*log(p) + (1-p)*log(1-p));
end