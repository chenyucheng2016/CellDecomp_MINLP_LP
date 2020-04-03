f = figure;
axes = gca;
axes.Visible = 'off';
pos = f.Position;
w = pos(3);
h = pos(4);
center = [w/2 h/2];
viscircles(center,4,'Color','r');
viscircles(center,2.5,'Color','b');