function AVS = getAVS(n)
AVSGuess = 90;
options = optimset('TolX',1e-6);
AVS = fzero(@(theta) getWaterLine(n,theta),AVSGuess,options);
end