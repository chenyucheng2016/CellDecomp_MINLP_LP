function blocked = isWallBlocked(walls,pos,tarloc)
x = pos(1);
z = pos(2);
blocked = 0;
wlen = length(walls);
for j = 1:wlen
    verts = walls{j}';
    vxmin = min(verts(1,:));
    vxmax = max(verts(1,:));
    vzmin = min(verts(2,:));
    vzmax = max(verts(2,:));
    lsegxmin = min(x,tarloc(1));
    lsegxmax = max(x,tarloc(1));
    lsegzmin = min(z,tarloc(2));
    lsegzmax = max(z,tarloc(2));
    if (vxmin > lsegxmax || vxmax < lsegxmin...
            || vzmin > lsegzmax ||vzmax < lsegzmin)
        %fprintf('vert/hor seperate idx %d\n',j);
    elseif (isSameSide(verts,[x,z],atan2(tarloc(2)-z,tarloc(1)-x)) == 0)
        %fprintf('rot seperate idx %d\n',j);
    else
        blocked = 1;
%          disp('break')
%          fprintf('The wall idx %d\n',j);
        break;
    end
end
end
function block = isSameSide(verts,pos,theta)
x = pos(1);
y = pos(2);
theta = toWholeAngle(theta);
angs = zeros(4,1);
for k = 1:size(verts,2)
    angs(k) = toWholeAngle(atan2(verts(2,k)-y,verts(1,k)-x));
end
if any(angs<pi/2) && any(angs>3*pi/2)
    s_angs = sort(angs);
    if theta > s_angs(3) || theta < s_angs(2)
        block = 1;
    else
        block = 0;
    end
else
    if length(find(angs<theta))==4 ||length(find(angs>theta))==4
        block = 0;
    else
        block = 1;
    end
end
end

function theta = toWholeAngle(thetar)
if thetar < 0
    theta = thetar + 2*pi;
else
    theta = thetar;
end
end