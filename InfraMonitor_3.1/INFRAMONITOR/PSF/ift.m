function [x]=ift(X,nft)
% [x]=ift(X,nft)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	ift.m (matlab function to compute inverse fourier transform, using
%	proper definition of the transform [dc component is mean of data])
%
%	curt a.l. szuberla --  mar 94
%==============================================================================

[r,c] = size(X);

if nargin == 1
  x = r * ifft(X);
else
  x = r * ifft(X,nft);
end

return