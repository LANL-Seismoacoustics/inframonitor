function [b,idx,outliers] = deleteoutliers(a,alpha,rep);
% [B, IDX, OUTLIERS] = DELETEOUTLIERS(A, ALPHA, REP)
%
% For input vector A, returns a vector B with outliers (at the significance
% level alpha) removed. Also, optional output argument idx returns the
% indices in A of outlier values. Optional output argument outliers returns
% the outlying values in A.
%
% ALPHA is the significance level for determination of outliers. If not
% provided, alpha defaults to 0.05.
%
% REP is an optional argument that forces the replacement of removed
% elements with NaNs to presereve the length of a. (Thanks for the
% suggestion, Urs.)
%
% This is an iterative implementation of the Grubbs Test that tests one
% value at a time. In any given iteration, the tested value is either the
% highest value, or the lowest, and is the value that is furthest
% from the sample mean. Infinite elements are discarded if rep is 0, or
% replaced with NaNs if rep is 1 (thanks again, Urs).
%
% Appropriate application of the test requires that data can be reasonably
% approximated by a normal distribution. For reference, see:
% 1) "Procedures for Detecting Outlying Observations in Samples," by F.E.
%    Grubbs; Technometrics, 11-1:1--21; Feb., 1969, and
% 2) _Outliers in Statistical Data_, by V. Barnett and
%    T. Lewis; Wiley Series in Probability and Mathematical Statistics;
%    John Wiley & Sons; Chichester, 1994.
% A good online discussion of the test is also given in NIST's Engineering
% Statistics Handbook:
% http://www.itl.nist.gov/div898/handbook/eda/section3/eda35h.htm
%
% ex:
% [B,idx,outliers] = deleteoutliers([1.1 1.3 0.9 1.2 -6.4 1.2 0.94 4.2 1.3 1.0 6.8 1.3 1.2], 0.05)
%    returns:
%    B = 1.1000    1.3000    0.9000    1.2000    1.2000    0.9400    1.3000    1.0000    1.3000    1.2000
%    idx =  5     8    11
%    outliers = -6.4000    4.2000    6.8000
%
% ex:
% B = deleteoutliers([1.1 1.3 0.9 1.2 -6.4 1.2 0.94 4.2 1.3 1.0 6.8 1.3 1.2
% Inf 1.2 -Inf 1.1], 0.05, 1)
% returns:
% B = 1.1000  1.3000  0.9000  1.2000  NaN  1.2000  0.9400  NaN  1.3000  1.0000  NaN  1.3000  1.2000  NaN  1.2000  NaN  1.1000
% Written by Brett Shoelson, Ph.D.
% shoelson@helix.nih.gov
% 9/10/03
% Modified 9/23/03 to address suggestions by Urs Schwartz.
% Modified 10/08/03 to avoid errors caused by duplicate "maxvals."
%    (Thanks to Valeri Makarov for modification suggestion.)
% Copyright (c) 2012, Los Alamos National Security, LLC
% All rights reserved.
% 
% Copyright 2012. Los Alamos National Security, LLC. This software was produced under U.S.
% Government contract DE-AC52-06NA25396 for Los Alamos National Laboratory (LANL), which is
% operated by Los Alamos National Security, LLC for the U.S. Department of Energy. The U.S.
% Government has rights to use, reproduce, and distribute this software.  NEITHER THE
% GOVERNMENT NOR LOS ALAMOS NATIONAL SECURITY, LLC MAKES ANY WARRANTY, EXPRESS OR IMPLIED,
% OR ASSUMES ANY LIABILITY FOR THE USE OF THIS SOFTWARE.  If software is modified to produce
% derivative works, such modified software should be clearly marked, so as not to confuse it
% with the version available from LANL.
% 
% Additionally, redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
% ·         Redistributions of source code must retain the above copyright notice, this list
% 		  of conditions and the following disclaimer.
% ·         Redistributions in binary form must reproduce the above copyright notice, this
% 	      list of conditions and the following disclaimer in the documentation and/or
% 	      other materials provided with the distribution.
% ·         Neither the name of Los Alamos National Security, LLC, Los Alamos National
% 	      Laboratory, LANL, the U.S. Government, nor the names of its contributors may be
% 	      used to endorse or promote products derived from this software without specific
% 	      prior written permission.
% THIS SOFTWARE IS PROVIDED BY LOS ALAMOS NATIONAL SECURITY, LLC AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
% EVENT SHALL LOS ALAMOS NATIONAL SECURITY, LLC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
% INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
% LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
% OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
% WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

if nargin == 1
	alpha = 0.05;
	rep = 0;
elseif nargin == 2
	rep = 0;
elseif nargin == 3
	if ~ismember(rep,[0 1])
		error('Please enter a 1 or a 0 for optional argument rep.')
	end
elseif nargin > 3
	error('Requires 1,2, or 3 input arguments.');
end

if isempty(alpha)
	alpha = 0.05;
end

b = a;
b(isinf(a)) = NaN;

%Delete outliers:
outlier = 1;
while outlier
	tmp = b(~isnan(b));
	meanval = mean(tmp);
	maxval = tmp(find(abs(tmp-mean(tmp))==max(abs(tmp-mean(tmp)))));
	maxval = maxval(1);
	sdval = std(tmp);
	tn = abs((maxval-meanval)/sdval);
	critval = zcritical(alpha,length(tmp));
	outlier = tn > critval;
	if outlier
		tmp = find(a == maxval);
		b(tmp) = NaN;
	end
end
if nargout >= 2
	idx = find(isnan(b));
end
if nargout > 2
	outliers = a(idx);
end
if ~rep
	b=b(~isnan(b));
end
return

function zcrit = zcritical(alpha,n)
%ZCRIT = ZCRITICAL(ALPHA,N)
% Computes the critical z value for rejecting outliers (GRUBBS TEST)
tcrit = tinv(alpha/(2*n),n-2);
zcrit = (n-1)/sqrt(n)*(sqrt(tcrit^2/(n-2+tcrit^2)));
