function level = computeFeatureLevel(feature)
level = -1;
if feature <= 8
    level = 3;
elseif feature >= 9 && feature <= 12
    level = 2;
elseif feature >= 13 && feature <= 14
    level = 1;
elseif feature == 15
    level = 0;
end
end