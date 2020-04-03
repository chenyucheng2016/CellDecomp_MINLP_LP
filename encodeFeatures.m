function featureCode = encodeFeatures(features)
featureCode = -ones(1,length(features));
for i = 1:length(features)
    f = features{i};
    switch f
        case 'Object'
            featureCode(i) = 15;
        case 'Box'
            featureCode(i) = 13;
        case 'Sphere'
            featureCode(i) = 14;
        case 'GreenSphere'
            featureCode(i) = 12;
        case 'BlackBox'
            featureCode(i) = 11;
        case 'BrownBox'
            featureCode(i) = 10;
        case 'OrangeSphere'
            featureCode(i) = 9;
        case 'Apple'
            featureCode(i) = 8;
        case 'Watermelon'
            featureCode(i) = 7;
        case 'Book'
            featureCode(i) = 6;
        case 'Computer'
            featureCode(i) = 5;
        case 'WoodenBox'
             featureCode(i) = 4;
        case 'CardboardBox'
             featureCode(i) = 3;
        case 'Basketball'
             featureCode(i) = 2;
        case 'Orange'
             featureCode(i) = 1;
        otherwise
            warning('Unidentified string')
    end
end
end