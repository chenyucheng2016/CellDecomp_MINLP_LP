function [lh,lh_dummy] = updateSalesmanPlot_2(lh,lh_dummy,xopt,idxs,stopsLat,stopsLon)
% Plotting function for tsp_intlinprog example

%   Copyright 2014-2016 The MathWorks, Inc. 

if ( lh ~= zeros(size(lh)) ) % First time through lh is all zeros
    set(lh,'Visible','off'); % Remove previous lines from plot
end
if ( lh_dummy ~= zeros(size(lh_dummy)) ) % First time through lh is all zeros
    set(lh_dummy,'Visible','off'); % Remove previous lines from plot
end

segments = find(round(xopt)); % Indices to trips in solution

% Loop through the trips then draw them
Lat = zeros(3*(length(segments)-2),1);
Lon = zeros(3*(length(segments)-2),1);
Lat_dummy = zeros(3*2,1);
Lon_dummy = zeros(3*2,1);

ind1 = 1;
ind2 = 1;
ii = 1;
while ii <= length(segments)
    start = idxs(segments(ii),1);
    stop = idxs(segments(ii),2);
    if start==size(stopsLon,1) || stop==size(stopsLon,1)
        Lat_dummy(ind2:ind2+2) = [stopsLat(start); stopsLat(stop); NaN];
        Lon_dummy(ind2:ind2+2) = [stopsLon(start); stopsLon(stop); NaN];
        ind2 = ind2+3;
    else
        % Separate data points with NaN's to plot separate line segments
        Lat(ind1:ind1+2) = [stopsLat(start); stopsLat(stop); NaN];
        Lon(ind1:ind1+2) = [stopsLon(start); stopsLon(stop); NaN];
        ind1 = ind1+3;
    end
    ii = ii+1;
end
lh = plot(Lat,Lon,'b-','LineWidth',3);
set(lh,'Visible','on'); drawnow; % Add new lines to plot

lh_dummy = plot(Lat_dummy,Lon_dummy,'g-','LineWidth',3);
set(lh_dummy,'Visible','on'); drawnow; % Add new lines to plot

