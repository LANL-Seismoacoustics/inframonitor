function S=Ssmooth(S,w,n,window);
% S=Ssmooth(S,w,n,window);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	Ssmooth.m    (matlab function to smooth a matrix S by columns)
%==============================================================================
%	input:	S -- [a,b] matrix
%	        w -- width of window [defaults to fix(sqrt(a))]
%	        n -- number of times to smooth [defaults to 1]
%	   window -- smoothing window (see manual for allowed types)
%	             [defaults to 'triang']
%
%	output:	S -- smoothed verion of input
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% HISTORY 
%	curt a.l. szuberla --  23 may 95

% trap n==0 (ie. no smoothing req'd)
if nargin>2
  if n==0,return,end
end

% default assignments
[a,b] = size(S);
if nargin==1
  w=fix(sqrt(a));
  n=1;
  window='triang';
elseif nargin==2
  if isempty(w)
    w=fix(sqrt(a));
  end
  n=1;
  window='triang';
elseif nargin==3
  if isempty(w)
    w=fix(sqrt(a));
  end
  if isempty(n),n=1;end
  window='triang';
else
  if isempty(w)
    w=fix(sqrt(a));
  end
  if isempty(n),n=1;end
  if isempty(window),window='triang';end
end

% error checking
if (isstr(S) | isstr(w) | isstr(n) | ~isstr(window))
  error('S,w,n must be numeric, window must be string');
end
w=fix(w);
n=fix(n);
if (w<1 | n<1)
  error('must have w>=1,n>=0');
end
if exist(window)<2
  error(['window type ' window ' does not exist']);
end
if (length(w)>1 | length(n)>1)
  error('w,n must be scalar');
end
if (w~=real(w) | n~=real(n))
  w=real(w);n=real(n);
  disp('warning: n,w=real(n,w)')
end

% get window vector from alpha-numeric arguments & normalize to unit DC gain
win = eval([window '(' num2str(w) ')']);
win = win/sum(win);

% a for loop here is actually slightly faster than a recursive call
% via string concatination
for ii=1:n
  S=conv2(S,win,'same');
end