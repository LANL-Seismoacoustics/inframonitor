function [detections,C] = Detect(F_K,dofnum,dofden,InfraUser)
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

% Adapting F-statistic to computed F-distribution:
fstats = F_K.fstat(1:2:numel(F_K.fstat));
c_scale = (0:0.1:100);
diff = zeros(1,numel(c_scale));
for i = 1:numel(c_scale)
    diff(i) = distfit(F_K.fstat,dofnum,dofden,c_scale(i),1);
end

%---
try
    C = c_scale(round(mean(find(diff == max(diff))))); % Scaling factor (C)
catch
    % Handling data outages by setting detections = 0:
    C = 0; detections = 0;
    return
end

% C = 1;

f_corr = (F_K.fstat)/C;

% plotdistfit(F_K.fstat,f_corr,dofnum,dofden,1-InfraUser.p)

% Converting F-statistics to p-values:
F_central_fit = pf(f_corr,dofnum,dofden);

% Computing start/end times of detections:
[spout,epout] = getdet(F_central_fit,1-InfraUser.p,1);    % F adaptive

% Assembling detection parameters:
detections = makebul(F_K.time,spout,epout,F_K.az,F_K.slofk,F_K.corr,f_corr,2);


% detections = cat(1,det2{1},det2{2});
if (numel(detections) == 0)
    clear detections
end

% Handling situations where no detections are obtained:
atest = exist('detections');
if (atest == 0)
    detections = 0;
    return
end
