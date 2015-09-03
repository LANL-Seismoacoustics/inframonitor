function [dateV] = doy2date(doyV,yearV)
% DOY2DATE.m will convert a vector of day of year numbers and years
% and convert them to MATLAB date format.
%
% Sample Call:
%  doyV = [54;200.4315];
%  yearV = [2009;2009];
%  [dateV] = doy2date(doyV,yearV);
%
% Inputs:
%  doyV -> vector of day of year numbers (n x 1)
%  yearV -> vector of years (n x 1)
%
% Outputs:
%  dateV -> vector of MATLAB dates (n x 1)
%
% AUTHOR    : A. Booth (ashley [at] boothfamily [dot] com)
% DATE      : 22-May-2009 09:34:53
% Revision  : 1.00
% DEVELOPED : 7.4.0.287 (R2007a) Windows XP
% FILENAME  : doy2date.m
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

%Check size of incoming vectors
if (size(doyV,1)== 1) | (size(doyV,2) == 1) %Make sure only a nx1 vector
    if size(doyV,1)<size(doyV,2)
        doyV = doyV';
        colflip = 1; %take note if the rows were flipped to col
    else
        colflip = 0;
    end
else %check to see that vectors are columns:
    error('DOY vector can not be a matrix')
end

%year vector
if (size(yearV,1)== 1) | (size(yearV,2) == 1) %Make sure only a nx1 vector
    if size(yearV,1)<size(yearV,2)
        yearV = yearV';
%         colflip = 1; %take note if the rows were flipped to col
    else
%         colflip = 0;
    end
else %check to see that vectors are columns:
    error('Year vector can not be a matrix')
end

%Check to make sure sizes of the vectors are the same
if ~min(size(doyV) == size(yearV))
    error('Day of year vector and year vector must be the same size')
end


%Make year into date vector
z = zeros(length(yearV),5);
dv = horzcat(yearV,z);

%Calc matlab date
dateV = doyV + datenum(dv);

% flip output if input was flipped
if colflip
    dateV = dateV';
end


% disp('Completed doy2date.m')
% ===== EOF [doy2date.m] ======
