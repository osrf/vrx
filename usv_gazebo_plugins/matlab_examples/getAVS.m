function AVS = getAVS(n,useGreensTheorem)
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
AVSGuess = 90;
options = optimset('TolX',1e-6);

if useGreensTheorem
    AVS = fzero(@(theta) getWaterLineGreensTheorem(theta,L,y_vals,z_vals),AVSGuess,options);
else
    AVS = fzero(@(theta) getWaterLine(theta,L,n,D,W),AVSGuess,options);   
end
end