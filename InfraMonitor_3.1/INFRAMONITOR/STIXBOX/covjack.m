function [C,y] = covjack(x,theta,p1,p2,p3,p4,p5,p6,p7,p8,p9)
%COVJACK  Jackknife estimate of the variance of a parameter estimate.
%
%	  C = covjack(X,'T')
%	  
%	  The function omputes T(X) with one observation removed at a 
%	  time  and uses the result to compute an estimate of the vari- 
%	  ance of T(X) assuming that X is a representative sample from  
%	  the underlying distribution of X. If T is multidimensional then
%	  the covariance matrix is estimated. Note that the jackknife  
%	  method does not work for some functions T that are not smooth 
%	  enough, the median being one example.

%       Anders Holtsberg, 14-12-94
%       Copyright (c) Anders Holtsberg

arglist = [];
for i = 3:nargin
   arglist = [arglist, ',p', num2str(i-2)];
end

if min(size(x)) == 1
   x = x(:);
end

n = size(x,1);

evalstring = [theta,'(xmi',arglist,')'];
xmi = x(2:n,:);
s = eval(evalstring);
y = [s(:) zeros(length(s(:)),n-1)];
for i = 2:n
   xmi = x([1:i-1,i+1:n],:);
   yy = eval(evalstring);
   y(:,i) = yy(:);
end

C = cov(y')*(n-1)^2 / n;