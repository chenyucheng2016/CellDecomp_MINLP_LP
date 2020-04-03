function V = computeInfoGainActual(tarsMeasure,targetInfos,unvisitedTargets)
init_features = targetInfos{2};
identities = targetInfos{3};
init_features_code = zeros(1,30);
EER_1level = [zeros(1,8),0.5866,0.0595,0.2704,0.0623,0.0786,0.0372,0.0049];
ClassGain = 1./[0.3669,0.1985,0.6191,0.6687,0.4049,0.2269,...
    0.6768,0.2536,0.6799,0.6768,0.5670,0.6494,0.6908,0.6883,0.69295];
alpha = 0.65;
for i = 1:11
    init_features_code(i) = encodeFeatures(init_features(i));
end
V = 0;
for i = 1:11
    if ~ismember(i,unvisitedTargets)
        tar = i;
        iden = identities(tar);
        curfeature = encodeFeatures(init_features(tar));
        curlevel = computeFeatureLevel(curfeature);
        if tarsMeasure(tar) > 0
            V = V + alpha*ClassGain(curfeature);
            for flevel = curlevel:3
                V = V + (1-alpha)*EER_1level(curfeature);
                if curfeature == 15
                    if ismember(iden,[3,4,5,6])
                        curfeature = 13;
                    elseif ismember(iden,[1,2,7,8])
                        curfeature = 14;
                    end
                elseif curfeature == 13
                    if ismember(iden,[3,4])
                        curfeature = 10;
                    elseif ismember(iden,[5,6])
                        curfeature = 11;
                    end
                elseif curfeature == 14
                    if ismember(iden,[1,2])
                        curfeature = 9;
                    elseif ismember(iden,[7,8])
                        curfeature = 12;
                    end
                else
                    curfeature = iden;
                end
            end
        else
             V = V + alpha*ClassGain(curfeature);
        end
    end
end