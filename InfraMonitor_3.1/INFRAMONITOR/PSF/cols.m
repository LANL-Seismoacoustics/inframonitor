function [ar]=cols(a,n)
% [ar]=cols(a,n)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       cols.m    (matlab function to form matrix composed of n cols of 
%                  vector/matrix a)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% HISTORY
%	curt a.l. szuberla --  18 apr 94
%	                       13 may 94 <generalized to add matrix input>

[r,c] = size(a);
ar = reshape(a(:) * ones(1,n),r,n*c);

return