function [mindt,maxdt] = GetGridLimits(NCOMB,stn_combs,D_STN,cel_min,cel_max,p_err)
% GetGridLimits: Outputs vectors containing minimum and maximum delay-times
% between each pair of arrays for each grid node
%
% New functionality for InfraMonitor 1.1
%
% Stephen Arrowsmith (08/08)
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
% Redistributions of source code must retain the above copyright notice, this list
% 		  of conditions and the following disclaimer.
% Redistributions in binary form must reproduce the above copyright notice, this
% 	      list of conditions and the following disclaimer in the documentation and/or
% 	      other materials provided with the distribution.
% Neither the name of Los Alamos National Security, LLC, Los Alamos National
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

for i = 1:NCOMB

    % Extracting array ID's:
    stn1 = stn_combs(i,1);
    stn2 = stn_combs(i,2);
    
    % Computing max and min travel times to each array:
    dt1 = D_STN{stn1}./cel_max(stn1);
    dt2 = D_STN{stn2}./cel_min(stn2);
    dt1_2 = D_STN{stn1}./cel_min(stn1);
    dt2_2 = D_STN{stn2}./cel_max(stn2);
    
    % Computing max and min inter-array group velocities:
    v_g{1} = (D_STN{stn1} - D_STN{stn2})./(dt1 - dt2);
    v_g{2} = (D_STN{stn1} - D_STN{stn2})./(dt1_2 - dt2_2);
    
    % Computing max and min inter-array delay times:
    dT{1} = (D_STN{stn1} - D_STN{stn2})./v_g{1};
    dT{2} = (D_STN{stn1} - D_STN{stn2})./v_g{2};

    mindt{i} = min(dT{1},dT{2}) - p_err;
    maxdt{i} = max(dT{1},dT{2}) + p_err;

end
