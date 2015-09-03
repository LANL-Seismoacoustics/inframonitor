function [trS,trS2]=trace_calc(S,didx);

% function for use with psf.m  [c.a.l. szuberla, 23 may 95]
% performs trace(S).^2 and trace(S*S) using index tricks

% this one is easy since didx points to diagonal elements
trS = sum(S(:,didx).').'.^2;

% here we recognize that trace(S*S) is just sum square magnitudes of all the
% non-redundant components of S, doubling squares of the non-diagonal elements
S(:,didx) = S(:,didx)/sqrt(2);
S=S*sqrt(2);
S=S.*(S').';
trS2 = sum(S.').';

return