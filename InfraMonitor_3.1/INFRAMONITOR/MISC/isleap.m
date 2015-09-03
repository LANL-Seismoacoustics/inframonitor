function x=isleap(Year)
%ISLEAP True for leap year.
%     ISLEAP(Year) returns 1 if Year is a leap year and 0 otherwise.
%     ISLEAP is only set for gregorian calendar, so Year >= 1583
%
% Syntax: 	ISLEAP(YEAR)
%
%     Inputs:
%           YEAR - Year of interest (default = current year).
%           You can input a vector of years.
%     Outputs:
%           Logical vector.
%
%      Example:
%
%           Calling on Matlab the function: isleap
%
%           Answer is: 0
%
%
%           Calling on Matlab the function: x=isleap([2007 2008])
%
%           Answer is:
%           x = 0 1
%
%           Created by Giuseppe Cardillo
%           giuseppe.cardillo-edta@poste.it
%           Modified after Simon Jan suggestions
% To cite this file, this would be an appropriate format:
% Cardillo G. (2007) Isleap: a simple routine to test if a year is a leap
% year.
% http://www.mathworks.com/matlabcentral/fileexchange/14172
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



%Input Error handling
switch nargin
    case 0
         c=clock; Year=c(1); clear c
    case 1
        if ~isvector(Year) || ~all(isnumeric(Year)) || ~all(isfinite(Year)) || isempty(Year)
            error('Warning: Year values must be numeric and finite')
        end
        if ~isequal(Year,round(Year))
            error('Warning: Year values must be integer')
        end
        L=Year-1583;
        if L(L<0)
            error('Warning: Every value of Year must be >1582')
        end
    otherwise
        error('stats:Isleap:TooMuchInputs','Year must be a scalar or a vector.');
end

% The Gregorian calendar has 97 leap years every 400 years:
% Every year divisible by 4 is a leap year.
% However, every year divisible by 100 is not a leap year.
% However, every year divisible by 400 is a leap year after all.
% So, 1700, 1800, 1900, 2100, and 2200 are not leap years,
% but 1600, 2000, and 2400 are leap years.
x = ~mod(Year, 4) & (mod(Year, 100) | ~mod(Year, 400));
return
