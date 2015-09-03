%----------------------------------------------------------------------------------------------------------
function [B,stn_combs] = TWOSTN(stns,T,ARID,D_STN,gv_min,gv_max,pick_err)
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

% Calculating all possible combinations of station pairs...

stn_combs = nchoosek(stns,2);
NCOMB = size(stn_combs,1);

% Computing combinations of detection pairs that occur within +/- dt (for each set of station pairs)

for i = 1:size(stn_combs,1);    % Note: Taking each combination of station pairs separately
    
    [mindt,maxdt] = GetGridLimits(NCOMB,stn_combs,D_STN,gv_min,gv_max,pick_err);
    
    dt(i) = max(maxdt{i});
    
    % Calculating the maximum inter-station delay time...
%     dt(i) = (6371 * acos(sin(lat_pairs(i,1)/57.2958)*sin(lat_pairs(i,2)/57.2958) + cos(lat_pairs(i,1)/57.2958)*cos(lat_pairs(i,2)/57.2958)*cos(lon_pairs(i,2)/57.2958 - lon_pairs(i,1)/57.2958)))/cel_min;
    
    i_str = num2str(i);
    tot_str = num2str(size(stn_combs,1));
    txt_str1 = 'TWOSTN: Processing combination ';
    txt_str2 = ' of ';
    txt_caption = strcat(txt_str1,i_str,txt_str2,tot_str);
    k = 0;      % NOTE: Added 01/04/2007
    for j = 1:numel(T{stn_combs(i,1)});  % Note: Taking 1st station in station pair as reference

        % Finding detections at 2nd station in station pair that occur within dt of detection...
        A = find((T{stn_combs(i,1)}(j)) - (dt(i)/86400) < T{stn_combs(i,2)} & T{stn_combs(i,2)} < (T{stn_combs(i,1)}(j)) + (dt(i)/86400));
        
        %---------------------------------------------------------
        % NOTE: Added 01/04/2007...
        if (numel(ARID{stn_combs(i,2)}(A)) == 0)
            continue
        end
        k = k + 1;
        %---------------------------------------------------------
        
        A_ARID{j} = ARID{stn_combs(i,2)}(A);
        % Rearranging cell array 'A' into numeric array 'b' of detection pairs...
        j_resized = repmat(j,size(A_ARID{j},1),1);
        j_resized = ARID{stn_combs(i,1)}(j_resized);
        A_ARID{j} = cat(2,j_resized,A_ARID{j});
        if (k == 1)
            b = A_ARID{j};
        else
            b = cat(1,b,A_ARID{j});
        end
        
    end
    
    clear A_ARID;

    isb = exist('b');      % Check to see if n-station detections exist
	if (isb == 0)
        B = 0;
        return
	end
    
    B{i} = b;   % Storing combinations of detection pairs for each station pair
        
end
