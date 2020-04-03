% The code is to measure the information gain of cue given samples
% Data set format is matrix 5*N

function [InfoGain,indCueModel] = InfoGainCalc2(Dataset)
numCue = 4;
numFeat = 3;
N = size(Dataset,2);

nC = 1; 
indCueModel = cell(1,15);
for iN = 1 : 4 % all possible combinations
    indCue = nchoosek([1 2 3 4],iN);
    for iC = 1 : size(indCue,1)
        indCueModel{nC} = indCue(iC,:);
        nC = nC + 1;
    end
end

InfoGain = zeros(numCue,1);
% Dataset(5,:) = Dataset(5,:) - 1; 
index = find(Dataset(5,:)==1);
p = length(index)/N;
Hc = -(p * log2(p) + (1-p) * log2(1-p));

% for iC = 1:numCue
%     H = zeros(3,1);
%     perc = zeros(3,1);
%     for iF = 1:numFeat
%         index = find(Dataset(iC,:) == iF);
%         numIndex = length(index);
%         perc(iF) = numIndex/N;
%         if numIndex == 0;
%             p = 0;
%         else
%             p = sum(Dataset(5,index))/numIndex;
%         end
%         if p == 0 || p== 1
%             H(iF) = 0;
%         else
%             H(iF) = -(p * log2(p) + (1-p) * log2(1-p));
%         end
%     end
%     InfoGain(iC) = Hc - H'*perc;
% end

InfoGain = zeros(15,1);
indexC1 = find(Dataset(5,:)==1);            
indexC2 = find(Dataset(5,:)==2);
Dataset1 = Dataset(:,Dataset(5,:)==1);
Dataset2 = Dataset(:,Dataset(5,:)==2);
for iC = 1:15
    cuecombin = indCueModel{iC};
    L = length(cuecombin);
    if L == 1
        box = zeros(2,3);
        for i = 1:3            
            indexF = find(Dataset1(cuecombin,:)==i);
            box(1,i) = length(indexF);
            indexF = find(Dataset2(cuecombin,:)==i);
            box(2,i) = length(indexF);
        end
%         p = box(1,:)./sum(box,1);
%         p(isnan(p)) = 0;
%         perc = sum(box,1)/N;
%         H = -(p .* log2(p) + (1-p) .* log2(1-p));
%         H(isnan(H)) = 0;
%         InfoGain(iC) = Hc - H*perc';        
        
    elseif L == 2
        box = zeros(2,3^L);
        for i = 1:3
            for j = 1:3
                indexF = intersect(find(Dataset1(cuecombin(1),:)==i),find(Dataset1(cuecombin(2),:)==j));
                box(1,i+(j-1)*3) = length(indexF);
                indexF = intersect(find(Dataset2(cuecombin(1),:)==i),find(Dataset2(cuecombin(2),:)==j));
                box(2,i+(j-1)*3) = length(indexF);
            end
        end
%         p = box(1,:)./sum(box,1);
%         p(isnan(p)) = 0;
%         perc = sum(box,1)/N;
%         H = -(p .* log2(p) + (1-p) .* log2(1-p));
%         H(isnan(H)) = 0;
%         InfoGain(iC) = Hc - H*perc';
    elseif L == 3
        box = zeros(2,3^L);
        for i = 1:3
            for j = 1:3
                for k = 1:3
                    indexF = intersect(find(Dataset1(cuecombin(1),:)==i),find(Dataset1(cuecombin(2),:)==j));
                    indexF = intersect(indexF,find(Dataset1(cuecombin(3),:)==k));
                    box(1,i+(j-1)*3+(k-1)*9) = length(indexF);
                    indexF = intersect(find(Dataset2(cuecombin(1),:)==i),find(Dataset2(cuecombin(2),:)==j));
                    indexF = intersect(indexF,find(Dataset2(cuecombin(3),:)==k));
                    box(2,i+(j-1)*3+(k-1)*9) = length(indexF);
                end
            end
        end
    else
        box = zeros(2,3^L);
        for i = 1:3
            for j = 1:3
                for k = 1:3
                    for t = 1:3
                        indexF = intersect(find(Dataset1(cuecombin(1),:)==i),find(Dataset1(cuecombin(2),:)==j));
                        indexF = intersect(indexF,find(Dataset1(cuecombin(3),:)==k));
                        indexF = intersect(indexF,find(Dataset1(cuecombin(4),:)==t));
                        box(1,i+(j-1)*3+(k-1)*9+(t-1)*27) = length(indexF);
                        indexF = intersect(find(Dataset2(cuecombin(1),:)==i),find(Dataset2(cuecombin(2),:)==j));
                        indexF = intersect(indexF,find(Dataset2(cuecombin(3),:)==k));
                        indexF = intersect(indexF,find(Dataset2(cuecombin(4),:)==t));
                        box(1,i+(j-1)*3+(k-1)*9+(t-1)*27) = length(indexF);
                    end
                end
            end
        end
    end
    p = box(1,:)./sum(box,1);
    p(isnan(p)) = 0;
    perc = sum(box,1)/N;
    H = -(p .* log2(p) + (1-p) .* log2(1-p));
    H(isnan(H)) = 0;
    InfoGain(iC) = Hc - H*perc';
end


% 
% % This function is to calculated entropy of P(C|X)
% function H = EntropyData(Dataset)
% numCue = size(Dataset,2) - 1;
% numFeat = 3;
% numVet = numFeat.^numCue;
% Vectors = zeros(numCue,numVet);
% for i = 1:numVet
%    
% end