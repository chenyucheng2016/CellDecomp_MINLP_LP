clear
close all
clc

%%
figure;

load('usborder.mat','x','y','xx','yy');
rng(3,'twister') % makes a plot with stops in Maine & Florida, and is reproducible
nStops = 24+1; % you can use any number, but the problem size scales as N^2
% stopsLon = zeros(nStops,1); % allocate x-coordinates of nStops
% stopsLat = stopsLon; % allocate y-coordinates
% n = 1;
% while (n <= nStops)
%     xp = rand*1.5;
%     yp = rand;
%     if inpolygon(xp,yp,x,y) % test if inside the border
%         stopsLon(n) = xp;
%         stopsLat(n) = yp;
%         n = n+1;
%     end
% end




stopsLon = zeros(nStops,1);
stopsLon([1]) = 1;
stopsLon([2,17,14,12]) = 2;
stopsLon([3,13,15]) = 3;
stopsLon([4,6,16]) = 4;
stopsLon([5,7,18]) = 5;
stopsLon([8,19,21]) = 6;
stopsLon([9,20]) = 7;
stopsLon([10,22]) = 8;
stopsLon([11,23]) = 9;
stopsLon([24]) = 10;
stopsLon(25) = 0;

stopsLat = zeros(nStops,1);
stopsLat([12,13]) = 4;
stopsLat([1,14,15,6,7,8,9]) = 3;
stopsLat(16) = 2.5;
stopsLat([17,18,19,20,10,11]) = 2;
stopsLat([2,3,4,5,21,22,23,24]) = 1;
stopsLat(25) = 0;

black = [12,13,14,15,16,17,18,19,20,21,22,23,24,25];    % element is index in stopsLon & stopsLat

% plot(x,y,'Color','red'); % draw the outside border
% hold on

flag = ismember([1:nStops],black);
% Add the stops to the map
plot(stopsLon(flag),stopsLat(flag),'.k','markers',60)
hold on
plot(stopsLon(~flag),stopsLat(~flag),'ok','markers',16)
hold on
% plot(1, 1, '.r', 'MarkerSize',69)

for i = 1:nStops-1
    x = stopsLon(i)-0.2;
    y = stopsLat(i)-0.2;
    st = ['(',num2str(i),')'];
    text(x,y,st);
    hold on
end
x = stopsLon(nStops)-0.3;
y = stopsLat(nStops)-0.3;
text(x,y,'Dummy');





%%
idxs = nchoosek(1:nStops,2);
idxs = [idxs;[idxs(:,2),idxs(:,1)]];
dist = zeros(size(idxs,1),1);
% dist = hypot(stopsLat(idxs(:,1)) - stopsLat(idxs(:,2)), ...
%              stopsLon(idxs(:,1)) - stopsLon(idxs(:,2)));
% not_conn = [1,3;1,6;2,5;2,6;3,5;3,6;4,6];
% for i = 1:size(idxs,1)
%     if sum(ismember(not_conn,idxs(i,:),'rows'))
%         dist(i) = 1e3;
%     end
% end

keySet =   {'1,12', '1,14', '1,17', '1,2', '12,13', '12,14', '13,14', '14,15', '15,17', '16,17', '17,18', '2,17','2,18','2,3', '13,15','6,13','15,16','3,4','6,16','6,7','6,18','16,18','4,5','7,8','7,19','7,18','18,19','5,21','8,9','8,19','8,20','19,20','21,22','9,10','9,20','10,11','10,23','22,23','11,24','11,23','23,24'};                                                 
valueSet = [2,      1.8,      1.3,    1,      0.5,    0.4,        0.5    0.3,    1.1,      1,       0.5,    2,      2,    1,     0.5,    0.6,   1,     0.5,   0.8,   1,    1.5,     1,    0.5, 0.5,     1,    1.2,  0.5,    0.5,  1.2,   0.7,   1,      0.6,    0.5,   2.5,   1.1,   0.5,     5,     0.7,     5,      2.1,    0.5];
conn = containers.Map(keySet,valueSet);

for i = 1:size(idxs,1)
    n1 = idxs(i,1);
    n2 = idxs(i,2);
    if n1<n2
        st = [num2str(n1),',',num2str(n2)];
    else
        st = [num2str(n2),',',num2str(n1)];
    end
    k = {st};
    if n1==nStops || n2==nStops
        dist(i) = 0;
        continue
    end
    if isKey(conn,k)
        dist(i) = conn(st);
        plot([stopsLon(n1),stopsLon(n2)],[stopsLat(n1),stopsLat(n2)],'k-','LineWidth',0.5);
        hold on
    else
        dist(i) = 1e5;
    end
end

lendist = length(dist);
tsp = optimproblem;
trips = optimvar('trips',lendist,1,'Type','integer','LowerBound',0,'UpperBound',1);

tsp.Objective = dist'*trips;

%% 
% % must have nStops trips
% constrips = sum(trips) == nStops;
% tsp.Constraints.constrips = constrips;

% (1) in == out
constrtrips = optimconstr(nStops,1);
for stops = 1:nStops
    whichIdxs = (idxs == stops);
    constrips(stops) = sum(trips(whichIdxs(:,1))) == sum(trips(whichIdxs(:,2)));
end
tsp.Constraints.constrips = constrips;

% (2) visit dummy once
constrtrips2 = optimconstr(1,1);
stops = nStops;
whichIdxs = (idxs == stops);
constrtrips2(1) = sum(trips(whichIdxs(:,1))) == 1;
tsp.Constraints.constrtrips2 = constrtrips2;

% (3) assign starting node
constrtrips3 = optimconstr(1,1);
start = 17;
dummy = nStops;
ind1 = (idxs==dummy);
ind2 = (idxs==start);
ind = ind1(:,1)&ind2(:,2);
constrtrips3(1) = sum(trips(ind)) == 1;
tsp.Constraints.constrtrips3 = constrtrips3;


% (4) visit black at least once
constr2trips = optimconstr(nStops,1);
for i = 1:length(black)
    stops = black(i);
    whichIdxs = (idxs == stops);
%     whichIdxs = any(whichIdxs,2); % start or end at stops
    constr2trips(i) = sum(trips(whichIdxs(:,1))) >= 1;
    constr2trips(length(black)+i) = sum(trips(whichIdxs(:,2))) >= 1;
end
tsp.Constraints.constr2trips = constr2trips;



opts = optimoptions('intlinprog','Display','off','Heuristics','advanced',...
    'LPPreprocess','basic');
tspsol = solve(tsp,opts)

% visualize
hold on
segments = find(tspsol.trips); % Get indices of lines on optimal path
lh = zeros(nStops,1); % Use to store handles to lines on plot
lh_dummy = zeros(2,1);
[lh,lh_dummy] = updateSalesmanPlot_2(lh,lh_dummy,tspsol.trips,idxs,stopsLon,stopsLat);
xlim([0,10]);
ylim([0,4]);
title('Solution with Subtours');

t = logical(tspsol.trips);idxs(t,:)
%% Subtours

[tours,is_loop] = detectSubtours_2(tspsol.trips,idxs);
numtours = length(tours); % number of subtours
fprintf('# of subtours: %d\n',numtours);

% Index of added constraints for subtours
k = 1;
while numtours > 1 % repeat until there is just one subtour
    % Add the subtour constraints
    for ii = 1:numtours
        subTourIdx = tours{ii}; % Extract the current subtour
%         The next lines find all of the variables associated with the
%         particular subtour, then add an inequality constraint to prohibit
%         that subtour and all subtours that use those stops.
        variations = nchoosek(1:length(subTourIdx),2);
        a = false(length(idxs),1);
        for jj = 1:size(variations,1)
            % whichVar: the edge in the subtour, indexed by idxs
            whichVar = (sum(idxs==subTourIdx(variations(jj,1)),2)) & ...
                       (sum(idxs==subTourIdx(variations(jj,2)),2));
            a = a | whichVar;
        end
        % a: all the edges in the subtour
        tsp.Constraints.(sprintf('subtourconstr%i',k)) = sum(trips(a)) <= length(subTourIdx)-1;
        k = k + 1;
    end
    % Try to optimize again
    [tspsol,fval,exitflag,output] = solve(tsp,opts);

    % Visualize result
    [lh,lh_dummy] = updateSalesmanPlot_2(lh,lh_dummy,tspsol.trips,idxs,stopsLon,stopsLat);

    % How many subtours this time?
    tours = detectSubtours_2(tspsol.trips,idxs);
    numtours = length(tours); % number of subtours
    fprintf('# of subtours: %d\n',numtours);
    pause(1);
end

[lh,lh_dummy] = updateSalesmanPlot_2(lh,lh_dummy,tspsol.trips,idxs,stopsLon,stopsLat);

title('Solution with Subtours Eliminated');
hold off

disp(output.absolutegap)
fval
exitflag

% pause()
% set(lh_dummy,'Visible','off'); drawnow; % Add new lines to plot

