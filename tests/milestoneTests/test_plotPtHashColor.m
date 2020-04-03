% TESTED: can plot milestone in colors grouped by EER value

% colorMap = containers.Map(keySet,valueSet)
figure
hold on

% colorMap = java.util.HashMap;
% 
% legendMap = [];
% for myMS = ms
%     if colorMap.containsKey(myMS.EER)
%         color = colorMap.get(myMS.EER);
%         
%     else
%         color = [rand  rand  rand];
%         colorMap.put(myMS.EER,color);
%         legendMap = [legendMap  myMS.EER];
%     end
%     plot(myMS.loc(1),myMS.loc(2),'^','MarkerFaceColor',color,'MarkerEdgeColor','None');
% end
% legend(legendMap)

allEER = [];
for myMS = ms
    allEER = [allEER  myMS.EER];
end
A = unique(allEER);
count = 1;
for eer = A
    color = rand(1,3);
    xs = []; ys = [];
    for myMS = ms
        if myMS.EER == eer
%             plot(myMS.loc(1),myMS.loc(2),'^','MarkerFaceColor',color,'MarkerEdgeColor','None');
            xs = [xs   myMS.loc(1)];
            ys = [ys   myMS.loc(2)];
        end
    end
h(count) = plot(xs,ys,'^','MarkerFaceColor',color,'MarkerEdgeColor','None');
count =  count + 1;
end
str = strtrim(cellstr(strcat('EER = ',num2str(A.',3))))
legend(h(:),str{:}); 