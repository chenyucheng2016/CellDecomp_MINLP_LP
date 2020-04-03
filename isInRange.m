function isin = isInRange(psi,rangl)
if length(rangl)==2
    isin = (psi > rangl(1) && psi < rangl(2));
else
    isin = (isInRange(psi,rangl(1:2))||isInRange(psi,rangl(3:4)));
end
end