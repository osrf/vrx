function [torque,d] = getWaterline(theta,L,n,D,W)
if nargin < 5
    % define the size and displacement of the boat
    L = 0.6;%m
    D = 0.5;%m
    W = 1;%m
    n = 2;
end

% define a grid of NxN points
N = 1000;
y_vals = linspace(-W-D,W+D,N);
z_vals = linspace(-W,D+W,N);

[y,z]=meshgrid(y_vals,z_vals);
dy = mode(diff(y_vals));  % hacky way to get dy
dz = mode(diff(z_vals));
da = dy.*dz;

% find the com;
[ycom,zcom,area] = cob(y,z,n,0,D,da,D,W);
mass = area*L*1000/4;%Kg

% define the direction of gravity
down = [0, 0, -1];
[d,ycob,zcob] = fastWaterline(y,z,n,theta,da,L,D,W,mass);
% find the torque
rotated_com = [cosd(theta) sind(theta);...
               -sind(theta) cosd(theta)]*[ycom; zcom];
torque = cross([0,ycob-rotated_com(1),zcob-rotated_com(2)],-down*mass*9.8);%Nm
torque = torque(1);
end

function [d,ycob,zcob] = fastWaterline(y,z,n,theta,da,L,D,W,mass)
aboveHullGrid = aboveHull(y,z,n,D,W,theta);
belowDeckGrid = belowDeck(y,z,n,D,W,theta);
waterlineIndex = min(find(da*cumsum(sum(aboveHullGrid&belowDeckGrid,2))*L*1000-mass>0));
d = z(waterlineIndex,1);
belowWaterGrid = belowWater(y,z,d);
index = find(aboveHullGrid & belowWaterGrid & belowDeckGrid);
area = length(index).*da;
ycob = sum(y(index).*da)./area;
zcob = sum(z(index).*da)./area;
end

function [ycob,zcob,area] = cob(y,z,n,theta,d,da,D,W)
% find the cob
aboveHullGrid = aboveHull(y,z,n,D,W,theta);
belowWaterGrid = belowWater(y,z,d);
belowDeckGrid = belowDeck(y,z,n,D,W,theta);
index = find(aboveHullGrid & belowWaterGrid & belowDeckGrid);
area = length(index).*da;
ycob = sum(y(index).*da)./area;
zcob = sum(z(index).*da)./area;
end

function z = hull(y,n,D,W,theta)
% the hull function
% transform y into boat frame by rotating by -theta
yBoatFrame = y*cos(theta);
z = D.*abs(2.*yBoatFrame/W).^n;
end

function aboveHullGrid = aboveHull(y,z,n,D,W,theta)
% the hull function
% transform y,z into boat frame by rotating by -theta
yBoatFrame = y*cosd(-theta) + z*sind(-theta);
zBoatFrame = y*sind(theta) + z*cosd(-theta);
hullBoatFrame = D.*abs(2.*yBoatFrame/W).^n;
aboveHullGrid = zBoatFrame > hullBoatFrame;
end

function belowDeckGrid = belowDeck(y,z,n,D,W,theta)
% transform z into boat frame by rotating by -theta
zBoatFrame = y*sind(theta) + z*cosd(-theta);
belowDeckGrid = zBoatFrame < D;
end

function belowWaterGrid = belowWater(y,z,d)
% the water function
belowWaterGrid = z <= d;
end
