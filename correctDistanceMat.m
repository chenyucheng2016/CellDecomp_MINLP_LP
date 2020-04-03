function [distance_matrix_new,tarsMeasure,tarsMeasure_Ideal] = correctDistanceMat(distance_matrix,targetInfos,omegas,cluster_target_guide)
%%The information value and costs for each cluster
%%Three options: 1). classify 2). One measurement and classify 3). Two
%%measurements and classify 4). Three measurements and classify
%1:Orange,2:Basketball,3:CardboardBox,4:Woodenbox,5:Computer,6:Book,7:Watermelon,8:Apple,9:OrangeSphere,
%10:BrownBox,11:BlackBox,12:GreenSphere,13:Box,14:Sphere,15:Object
tarsMeasure = zeros(1,30);
tarsMeasure_Ideal = zeros(1,30);
EER_1level = [zeros(1,8),0.5866,0.0595,0.2704,0.0623,0.0786,0.0372,0.0049];
EER_2level = [zeros(1,12),0.1158,0.3242,0.0618];
EER_3level = 0.2776;
ClassGain = 1./[0.3669,0.1985,0.6191,0.6687,0.4049,0.2269,...
    0.6768,0.2536,0.6799,0.6768,0.5670,0.6494,0.6908,0.6883,0.69295];
InfoVals = zeros(1,length(cluster_target_guide));
init_features = targetInfos{2};
alpha = 0.65;
for i = 1:length(cluster_target_guide)
    tars = cluster_target_guide(i);
    tot_infoval = 0;
    for tar = tars
        init_feature = init_features(tar);
        init_feature_index = encodeFeatures(init_feature);
        feature_level = computeFeatureLevel(init_feature_index);
        tarsMeasure_Ideal(tar) = 3 - feature_level;
        classg = ClassGain(init_feature_index);
        if init_feature_index > 8 && init_feature_index < 13
            [infoval,measureTimes] = max([0,omegas(2)*EER_1level(init_feature_index) - omegas(3)]);
            tot_infoval = tot_infoval + infoval*alpha + (1 - alpha)*classg;
            tarsMeasure(tar) = measureTimes-1;
        elseif init_feature_index < 14
            [infoval,measureTimes] = max([0,omegas(2)*EER_1level(init_feature_index) - omegas(3),omegas(2)*EER_2level(init_feature_index) - omegas(3)*2]);
            tot_infoval = tot_infoval + infoval*alpha + (1 - alpha)*classg;
            tarsMeasure(tar) = measureTimes-1;
        else
            [infoval,measureTimes] = max([0,omegas(2)*EER_1level(init_feature_index) - omegas(3),omegas(2)*EER_2level(init_feature_index) - omegas(3)*2, omegas(2)*EER_3level - omegas(3)*3]);
            tot_infoval = tot_infoval + infoval*alpha + (1 - alpha)*classg;
            tarsMeasure(tar) = measureTimes-1;
        end
    end
    InfoVals(i) = tot_infoval; 
end
distance_matrix_new = distance_matrix;
numClusters  = size(distance_matrix,1);
for i = 1:numClusters
    for j = 1:numClusters
        if (i == j || j <= 1)
            distance_matrix_new(i,j) = omegas(1) * distance_matrix(i,j);
        else
            distance_matrix_new(i,j) = omegas(1) * distance_matrix(i,j) - InfoVals(j-1);
        end
    end
end

% dmin = min(distance_matrix_new(:));
% 
% distance_matrix_new = distance_matrix_new + abs(dmin);
end

