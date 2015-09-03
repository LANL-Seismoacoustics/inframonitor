function [start_pt,stop_pt] = getdet(C_P,sig_level,Npts)
% getdet: Obtains the start and end points for significant detections given
% a p-value and threshold significance level.
% Usage: [start_pt,stop_pt] = getdet(C_P,sig_level,Npts)
% Where:
% start_pt and stop_pt give the start/end points of each detection
% C_P is a p-value
% sig_level is the significance threshold
% Npts is currently obsolete (but can provide a minimum length threshold)
% Stephen Arrowsmith (arrows@lanl.gov)
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

in_det = 0;     % Flag that indicates if detector is within a detection
d = 0;

% Computing start/stop points for detections:

for i = 1:numel(C_P)

    if (in_det == 0)
        if (C_P(i) >= sig_level)
            in_det = 1;
            d = d + 1;
            start_pt(d) = i;
        else
            continue
        end
    else
        if (C_P(i) >= sig_level)
            continue
        else
            in_det = 0;
            stop_pt(d) = i;
        end
    end

end

% Exiting if either (a) there are no detections, (b) there is a continuous
% detection:
check_det(1) = exist('start_pt');
check_det(2) = exist('stop_pt');
if (numel(find(check_det == 0)) == 2)       % No detections
    start_pt = 0;
    stop_pt = 0;
    return
elseif (numel(find(check_det == 0)) == 1)   % Continuous detection
    start_pt = -1;
    stop_pt = -1;
    return
end

if (numel(start_pt) > numel(stop_pt))
    stop_pt(numel(start_pt)) = numel(C_P);
end
