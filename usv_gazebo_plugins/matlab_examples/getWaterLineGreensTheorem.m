function [torque,d] = getWaterLineGreensTheorem(theta, L, y_vals, z_vals)
if nargin < 4
    % define the size and displacement of the boat
    L = 0.6;%m
    D = 0.5;%m
    % this is for a boat of width 1
    W = 1;%m
    % define points to comprise the hull in the boat fram
    N = 5000;
    % define shape parameterr
    n = 2;

    y_vals_hull = linspace(-W/2,W/2,N);
    z_vals_hull = D.*abs(2.*y_vals_hull./W).^n;

    y_vals_deck = linspace(W/2,-W/2,N);
    z_vals_deck = D*ones(size(y_vals_deck));
    y_vals = [y_vals_hull y_vals_deck];
    z_vals = [z_vals_hull z_vals_deck];
end

% find the com;
[area,ycom,zcom] = line_integral(y_vals,z_vals);
mass = area*L*1000/4;%Kg

% define the direction of gravity
down = [0, 0, -1];

% rotate the boat about the origin to match the Gazebo simulator
[y_vals_rotated,z_vals_rotated]=rotate(y_vals,z_vals,theta,0,0);

[d,ycob,zcob] = fastWaterline(y_vals_rotated,z_vals_rotated,L,mass);
% find the torque
rotated_com = [cosd(theta) sind(theta);...
               -sind(theta) cosd(theta)]*[ycom; zcom];
torque = cross([0,ycob-rotated_com(1),zcob-rotated_com(2)],-down*mass*9.8);%Nm
torque = torque(1);
end

function [d,ycob,zcob] = fastWaterline(y,z,L,mass)
dz = diff(z);
area_term = y(1:end-1).*dz;
ycom_term = 0.5*y(1:end-1).^2.*dz;
zcom_term = y(1:end-1).*z(1:end-1).*dz;

area_displaced_at_eq = mass/L/1000;

M = sortrows([z(1:end-1)' y(1:end-1)' area_term' ycom_term' zcom_term'],1);
area = cumtrapz(M(:,3));
ycom = cumtrapz(M(:,4))./area_displaced_at_eq;
zcom = cumtrapz(M(:,5))./area_displaced_at_eq;
eq_index = min(find(area > area_displaced_at_eq));

d = M(eq_index,1);
ycob = ycom(eq_index);
zcob = zcom(eq_index);
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

function aboveHullGrid = aboveHull(y,z,n,D,W,theta)
% the hull function
% transform y,z into boat frame by rotating by -theta
yBoatFrame = y*cosd(-theta) + z*sind(-theta);
zBoatFrame = y*sind(theta) + z*cosd(-theta);
hullBoatFrame = D.*abs(2.*yBoatFrame/W).^n;
aboveHullGrid = zBoatFrame > hullBoatFrame;
end

% Define a rotation about a point (ycom,zcom)
function [yy,zz] = rotate(y,z,theta,ycom,zcom)
yy = ycom + cosd(theta)*(y-ycom) + sind(theta)*(z-zcom);
zz = zcom - sind(theta)*(y-ycom) + cosd(theta)*(z-zcom);
end

% Use line integrals to compute area and cob
function [area,ycom,zcom] = line_integral(y,z)
% Use Green's theorem to convert the double integrals
% to a line integral of the form integral f(x) dy. Each
% quantity has a different integrand.
dz = diff(z);
area_term = y(1:end-1).*dz;
ycom_term = 0.5*y(1:end-1).^2.*dz;
zcom_term = y(1:end-1).*z(1:end-1).*dz;
area = trapz(area_term);
ycom = trapz(ycom_term)./area;
zcom = trapz(zcom_term)./area;
end