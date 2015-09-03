function [X]=ft(x,nft)
% [X]=ft(x,nft)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	ft.m (matlab function to compute forward fourier transform, using
%	proper definition of the transform [dc component is mean of data])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% HISTORY
%	curt a.l. szuberla --  mar 94

[r,c] = size(x);

% use matlab's algorithms
if nargin == 1
  X = 1/r * fft(x);
else
  X = 1/r * fft(x,nft);
end

return
