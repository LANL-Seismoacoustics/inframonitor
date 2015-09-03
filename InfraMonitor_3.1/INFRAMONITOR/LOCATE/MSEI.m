function [tag] = MSEI(Y,b_offset,gv_min,gv_max,stn_lon,stn_lat,D_STN,...
    BA_STN,LAT,LON,pick_err)
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

% Temporary (Change)
gv_min = repmat(gv_min,1,numel(Y));
gv_max = repmat(gv_max,1,numel(Y));

stns = (1:numel(stn_lon));

% (D) Setting flags to indicate which portions of the code to run...
% (Note: 1 = Run function, 0 = Don't run function)...
run_twostn = 1;     run_clust = 1;      run_consis = 1;     run_nstn = 1;
run_plotnstn = 0;

% (1) READING IN DETECTION DATA:
for i = 1:numel(Y)
    X = Y{i};
       
    if (numel(X{1}) == 0)
        tag = 0;
        return
    end
    
    % -----------------------------------
    % Added 01/10/07...
    FLAGS = X{5};
    IX = find(FLAGS == 1);
    for i2 = 1:numel(X)
        X{i2} = X{i2}(IX);
    end
    % -----------------------------------
    ARID{i} = X{4};
    A{i} = X{3};
    T{i} = datenum(cat(2,cat(1,X{1}{:}),cat(1,X{2}{:})),'yyyy-mm-ddHH:MM:SS');
end
clear X

Method = 1;     % 1=Old,2=New

if (Method == 1)
    % (2) CALLING TWOSTN:

    if (run_twostn == 1)
        t1 = clock;
        [B,stn_combs] = TWOSTN(stns,T,ARID,D_STN,gv_min,gv_max,pick_err);

        if (iscell(B)==0)       % If B is set to 0 it is not a cell array!
            tag = 0;
            return
        end

        t2 = clock;
        e_twostn = etime(t2,t1);
    %     save TWOSTN.mat B stn_combs;
    elseif (run_twostn == 0)
        load TWOSTN.mat;
    end

    % Setting parameters for the 2-station only case...
    if (numel(stns) == 2)
        ndet = B{1};
        run_clust = 0;
        run_consis = 0;
    end

    % (3) CALLING CLUST:

    if (run_clust == 1)
        t1 = clock;
        [ndet] = CLUST(B,stn_combs);

        if (ndet == 0)
            tag = 0;
            return
        end

        t2 = clock;
        e_clust = etime(t2,t1);
    %     save CLUST.mat ndet;
    elseif (run_clust == 0)
        if (numel(stns) ~= 2)
            load CLUST.mat;
        end
    end

    % (4) CALLING CONSIS_V2:

    if (run_consis == 1)
        t1 = clock;
        [ndet] = CONSIS_V2(ndet);
        t2 = clock;
        e_consis = etime(t2,t1);
    %     save CONSIS_V2.mat ndet;
    elseif (run_consis == 0)
        if (numel(stns) ~= 2)
            load CONSIS_V2.mat;
        end
    end
end

% (5) CALLING NSTN:

if (run_nstn == 1)
    
    try
        if (Method == 2)
            % *** ADDED 08/18/08 ***
            ndet = AllCombs(ARID);
            stn_combs = nchoosek(stns,2);
        end
        % **********************
        [tag] = NSTN_v2(stns,stn_combs,b_offset,gv_min,gv_max,T,A,ndet,...
            ARID,D_STN,BA_STN,LAT,LON,pick_err);
    catch
        keyboard
    end

elseif (run_nstn == 0)
    load NSTN.mat;
end

% (6) CALLING plot_NSTN:

if (run_plotnstn == 1)
    y = plot_NSTN(stn_lon,stn_lat,tag)
end
