function visibleTargetList = findVisibleTarget(targets,pose,walls)
x = pose(1);
z = pose(2);
theta = pose(3)/57.29;
tarLen = size(targets,1);
visibleTargetList = [];
halfAngle = pi/3;
phi = convAngle(theta);
for i = 1:tarLen
    tarloc = targets(i,:);
    if (tarloc(1) == -30 && tarloc(2) == -30)
        continue
    else
        psi = atan2(z-tarloc(2),x-tarloc(1));
        rangl = [];
        if abs(abs(phi)-pi) > halfAngle
            philb = phi - halfAngle;
            phiub = phi + halfAngle;
            rangl = [philb,phiub];
        elseif phi < 0
            rangl = [-pi,phi+halfAngle,(phi-halfAngle+2*pi),pi];
        else
            rangl = [-pi,(phi+halfAngle-2*pi),phi-halfAngle,pi];
        end
        if (isInRange(psi,rangl)==1)
            blockwall = isWallBlocked(walls,[x,z],tarloc);
            if (blockwall == 0)
                visibleTargetList = [i,visibleTargetList];
            end
        end
    end
end
end



