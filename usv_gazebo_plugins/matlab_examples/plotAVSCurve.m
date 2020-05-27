function plotAVSCurve(n, useGreensTheorem)
if nargin < 2
    useGreensTheorem = true;
end
L = 0.6;%m
D = 0.5;%m
% this is for a boat of width 1
W = 1;%m
% define points to comprise the hull in the boat fram
if useGreensTheorem
    % define the boat and pass it in (saves time)
    N = 5000;
    y_vals_hull = linspace(-W/2,W/2,N);
    z_vals_hull = D.*abs(2.*y_vals_hull./W).^n;

    y_vals_deck = linspace(W/2,-W/2,N);
    z_vals_deck = D*ones(size(y_vals_deck));
    y_vals = [y_vals_hull y_vals_deck];
    z_vals = [z_vals_hull z_vals_deck];
end
thetas = linspace(0,180,180*4+1);
torques = zeros(size(thetas));
waterlines = zeros(size(thetas));
for i = 1:length(thetas)
    if useGreensTheorem
        [torques(i),waterlines(i)] = getWaterLineGreensTheorem(thetas(i),L,y_vals,z_vals);
    else
        [torques(i),waterlines(i)] = getWaterLine(thetas(i),L,n,D,W);
    end
end
figure;
plot(thetas, torques);
xlabel('Heel Angle (degrees)');
ylabel('Torque (Nm)');

figure;
plot(thetas, waterlines);
xlabel('Heel Angle (degrees)');
ylabel('Draft (m)');
end