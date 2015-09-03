function [Xpf,P,S]=psf(X,w,p,n,window);
% [Xpf,P,S]=psf(X,width,power,nsmooth,window);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	psf.m    (matlab function to pure state filter data matrices)
%==============================================================================
%	input:	   X -- [N,d] real data matrix (times series as columns)
%	       width -- width of smoothing window (in points)
%   	                [defaults to fix(sqrt(N))]
%   	       power -- exponent of P (polarization estimate)
%   	                [defaults to 2]
%            nsmooth -- # of times to smooth spectral matrices
%                       [defaults to 3]
%                       window -- smoothing window type (string, see manual
%                       [defaults to 'triang']           for allowed types)
%
%      output:	 Xpf -- [N,d] pure state filtered data matrix (by column)
%		   P -- polarization estimate
%		   S -- spectral matrices (see code for storage format)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% HISTORY
%	curt a.l. szuberla -- 23 may 95

% DEPENDANCIES
%       psf.m -- Ssmooth.m (performs smoothing)
%             -- trace_calc.m (calculates traces using compact storage format)
%             -- ft.m & ift.m (redefined fft & ifft such that DC component
%                              is mean of series)
%             -- cols.m (forms column matrices)

% size up input (each of d time series is N points long)
[N,d] = size(X);

% set defaults if req'd (note that Ssmooth does its own defaults/error checks)
if nargin==1
  w=[];
  p=2;
  n=[];
  window=[];
elseif nargin==2
  p=2;
  n=[];
  window=[];
elseif nargin==3
  n=[];
  window=[];
  if isempty(p),p=2;end
elseif nargin==4
  if isempty(p),p=2;end
  window=[];
else
  if isempty(p),p=2;end
end

% error checks (Ssmooth will do its own for w,n,window)
if (isstr(X) | isstr(p))
  error('X,p must be numeric');
end
if p~=real(p)
  p=real(p);
  disp('warning: p=real(p)')
end
if(length(p)>1)
  error('p must be scalar');
end
if (N<4 | d<2)
  error('X must be at least [4,2] matrix');
end
if(find(X~=real(X)))
  error('X must be real data matrix');
end
% force N even, warn if padded
if (N/2 ~= fix(N/2))
  X=[X;zeros(1,d)];
  N=N+1;
  disp('warning: padded X to make N even') % why use odd-length data anyway?
end

% fourier transform data matrix
x = ft(X);
x = x(1:N/2+1,:); 			% keep only DC--Nyq.

% preallocate space for polarization estimates and spectral matrices
P=zeros(1:N/2+1,1);
S=zeros(N/2+1,d*(d+1)/2);

% tricky indexing to save time & storage space (have fun figuring this
% stuff out!); for each frequency (n/2+1 of them) we get a [d,d] spectral
% matrix, but for real input the spectral matrix is hermitian, therefore
% only has d*(d+1)/2 indep. elements.  so for each freq. we store the
% spectral matrix as a vector containing only the non-redundant information
% 
% eg. for d==3        | 1 2* 3*|
%             S'(w) = | 2 4  5*|
%                     | 3 5  6 |
% is stored as
%              S(w) = [1 2 3 4 5 6]
%
% in order to get maximum efficiency in matlab, we form some index vectors
% right away: didx gives locations of diagonal elements in S(w), while Sidx
% is used to identify which elements of S'(w) are non-redundant
dstep=d:-1:2;
didx=ones(d,1);
Sidx=[1:d zeros(1,d*(d+1)/2-d)];
for l = 2:d
  didx(l) = didx(l-1) + dstep(l-1);
  Sidx(didx(l):l*(d-(l-1)/2))=(1:d-(l-1)) + (l-1)*(d+1);
end

% form each spectral matrix via brute force, then use Sidx to put it into
% compact storage format
for ii = 1:N/2+1
  Sw=x(ii,:)'*x(ii,:);
  S(ii,:)=Sw(Sidx);
end
clear Sw 				% no longer req'd

% smooth each column of S (ie. in freq. domain) 
% with specified window n times 
S=Ssmooth(S,w,n,window);

% calculate trace(S).^2 & trace(S*S) from our storage format
[trS,trS2]=trace_calc(S,didx);

% estimate polarization at each freq. via samson method
% (reduces to fowler's for d==2)
P=(d*trS2 - trS)./((d-1)*trS);
if nargout<3
  clear S 				% if S not req'd as output
end

% array multiplication in freq. space
P=cols(P,d);
Xpf=P.^p.*x;
if nargout<2
  clear P; 				% if P not req'd as output
end
P=P(:,1); 				% only need 1 column

% inverse xfrom to get pure state filtered data matrix
Xpf=[Xpf;conj(flipud(Xpf(2:N/2,:)))]; 	% fold out conjugates again
Xpf=real(ift(Xpf)); 			% trap residual complex part

return
