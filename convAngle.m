function theta = convAngle(thetar)
if thetar < 0
    theta = thetar + pi;
else
    theta = thetar - pi;
end