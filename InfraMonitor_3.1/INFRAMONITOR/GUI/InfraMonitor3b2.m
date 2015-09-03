function InfraMonitor3b2(wfdiscPath,wfdisc_file,f_band,f_order,min_arrays,schemaType)
%
% InfraMonitor3b2 - Processes a wfdisc file with InfraMonitor to produce
% output detections (arrival and detect files), and locations (origin,
% IMassoc, and IMpoly files).
%
% This version is an update of InfraMonitor3b.m, which requires much less
% memory as it only reads in the waveform data from 1 array at a time.
%
% Note 1: A site file with the same prefix as the wfdisc file must be
% located in the same place as the wfdisc file
%
% Note 2: To produce locations, an appropriately named grid file must be
% located in the same path as the wfdisc file. The grid file prefix must
% have the same prefix as the wfdisc file. The suffix should be .mat.
%
% Usage:
% InfraMonitor3b(wfdiscPath,wfdisc_file,f_band,f_order,min_arrays,schemaType)
%
% Where:
% - wfdiscPath is the path to the wfdisc file
% - wfdisc_file is the name of the wfdisc file
% - f_band is the frequency band [min_f max_f]
% - f_order is the order of the filter
% - min_arrays is the minimum number of arrays required to produce an
%   association
% - schemaType is either 'css3.0' or 'nnsa' depending on the format of the
%   wfdisc and site files
%
% e.g.,
% InfraMonitor3b('.','utahOut.wfdisc',[1 5],2,3,'css3.0');
%
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

warning off all
clear global InfraConfig InfraConfigBackup assoc toassoc AssocIn
clear global arrays

global InfraConfig

% Reading user parameters:
InfraUser = open('InfraUser.mat');
InfraUser.slow = linspace(-400,400,40);

% Ensuring wfdiscPath has a trailing '/':
if (strcmp(wfdiscPath(numel(wfdiscPath)),'/') ~= 1)
    wfdiscPath = [wfdiscPath '/'];
end
InfraConfig.db.wfdiscPath = wfdiscPath;

% Reading wfdisc and site tables:
try
    InfraConfig.db.dir = wfdiscPath;
    InfraConfig.db.name = regexprep(wfdisc_file,'.wfdisc','');
    InfraConfig.db.wfdisc = readWfdisc([wfdiscPath wfdisc_file],schemaType);
    InfraConfig.db.site = readSite([wfdiscPath InfraConfig.db.name '.site'],schemaType);
    InfraConfig.db.select = 1:numel(InfraConfig.db.wfdisc{1});
    InfraConfig.db.stime = epoch(min(InfraConfig.db.wfdisc{3}(InfraConfig.db.select)));
    InfraConfig.db.etime = epoch(max(InfraConfig.db.wfdisc{7}(InfraConfig.db.select)));
catch
    disp('Could not read wfdisc and site tables. Try changing the schema type, or ensure that both tables exist');
    return
end

% Just need InfraConfig.trace.refsta:
wfdiscElements = InfraConfig.db.wfdisc{1};
siteElements = InfraConfig.db.site{1};
j = 0;
for i = 1:numel(wfdiscElements)
%     strcmpNos = strcmp(wfdiscElements,siteElements{i});
    try
        strcmpNos = strcmp(siteElements,wfdiscElements{i});
    catch
        keyboard
    end

    I = find(strcmpNos == 1);

    if (numel(I) > 0)
        j = j + 1;
        try
            InfraConfig.trace.refsta{j} = InfraConfig.db.site{9}{I};
%             InfraConfig.trace.refsta{j} = wfdiscElements{I};
        catch
            keyboard
        end
    end
end
% -------------------------------------------------------------------------

% Running detector on each array separately:
m = 0;
uniqueArrays = unique(InfraConfig.trace.refsta);
for i = 1:numel(uniqueArrays)

    I = strcmp(uniqueArrays{i},InfraConfig.trace.refsta);

    InfraConfig.trace.select = find(I == 1);

    if (i > 1)
        InfraConfig.trace.data = [];
        InfraConfig.trace.time = [];
    end

    Maketrace2(InfraConfig.trace.select);

    % Filtering data:
    for jj = 1:numel(InfraConfig.trace.select)
        kk = InfraConfig.trace.select(jj);
        try
            [b,a] = butter(f_order,[f_band(1) f_band(2)]/ceil(InfraConfig.trace.s_f{kk}/2));
            InfraConfig.trace.data{kk} = filter(b,a,HanningWindow(InfraConfig.trace.data{kk}));
        catch
            keyboard
        end
    end

    save('InfraMonitor3b2.mat');

    emptyCells = find(cellfun('isempty',InfraConfig.trace.name)==1);
    for rr = 1:numel(emptyCells)
        prevName = InfraConfig.trace.name{emptyCells(rr)-1};
        InfraConfig.trace.name{emptyCells(rr)} = [prevName '2'];
    end

    tmp = InfraConfig.trace.select;
    tmp2 = InfraConfig.trace.fname;
    tmp3 = InfraConfig.trace.refsta;
    cleanTrace
    InfraConfig.trace.select = tmp;
    InfraConfig.trace.fname = tmp2;
    InfraConfig.trace.refsta = tmp3;

    % Removing additional entries from InfraConfig.trace.select:
    for q = 1:numel(InfraConfig.trace.select)
        try
            qq = InfraConfig.trace.name{InfraConfig.trace.select(q)};
        catch
            removeNos(q) = InfraConfig.trace.select(q);
        end
    end

    try
        InfraConfig.trace.select = setdiff(InfraConfig.trace.select,removeNos);
        InfraConfig.trace.fname = InfraConfig.trace.fname(1:numel(InfraConfig.trace.name));
    end

    stime = InfraConfig.db.stime; etime = InfraConfig.db.etime;
    s_f = InfraConfig.trace.s_f{InfraConfig.trace.select(1)};

    NSeconds = (etime - stime)*86400;
    NRuns = ceil(NSeconds/InfraUser.w);
    InfraConfig.db.f_band = f_band';

    % Split data into time windows and process on each time window
    % separately:
    NWindows = ceil((numel(InfraConfig.trace.data{InfraConfig.trace.select(1)})/s_f)/InfraUser.w);

    dofnum = 0.5*(2*InfraUser.twin*(InfraConfig.db.f_band(2)...
        -InfraConfig.db.f_band(1)));
    dofden = 0.5*(2*InfraUser.twin*(InfraConfig.db.f_band(2)...
        -InfraConfig.db.f_band(1)))*(numel(InfraConfig.trace.select)-1);

    for j = 1:NWindows

        if (j == 1)
            t_start = 1;
            p_stime = stime;
            p_etime = stime + InfraUser.w/86400;
        else
            t_start = t_start + InfraUser.w*s_f;
            p_stime = p_etime;
            p_etime = p_stime + InfraUser.w/86400;
        end

        try
            F_K = RunFK(InfraUser,p_stime,min(etime,p_etime),t_start,j,NWindows);
        catch
            keyboard
        end

        if (isstruct(F_K) == 0)
            disp('Error reading data. Check wfdisc and site files carefully')
        end
        [arrivals,C] = Detect(F_K,dofnum,dofden,InfraUser);

        if (j == 1)
            InfraConfig.detect.time{i} = F_K.time;
            InfraConfig.detect.slofk{i} = F_K.slofk;
            InfraConfig.detect.az{i} = F_K.az;
            InfraConfig.detect.pow{i} = F_K.pow;    % ***
            InfraConfig.detect.fstat{i} = F_K.fstat/C;
            InfraConfig.detect.corr{i} = F_K.corr;
            InfraConfig.detect.trc{i} = InfraConfig.trace.select;
            if (size(arrivals,2) == 8)
                arrivals_all = arrivals;
            end
        else
            InfraConfig.detect.time{i} = cat(2,InfraConfig.detect.time{i},F_K.time);
            InfraConfig.detect.slofk{i} = cat(2,InfraConfig.detect.slofk{i},F_K.slofk);
            InfraConfig.detect.az{i} = cat(2,InfraConfig.detect.az{i},F_K.az);
            InfraConfig.detect.pow{i} = cat(2,InfraConfig.detect.pow{i},F_K.pow);
            InfraConfig.detect.fstat{i} = cat(2,InfraConfig.detect.fstat{i},F_K.fstat/C);
            InfraConfig.detect.corr{i} = cat(2,InfraConfig.detect.corr{i},F_K.corr);
            try
                arrivals_all = cat(1,arrivals_all,arrivals);
            catch
                if (exist('arrivals_all','var') == 0)
                    if (size(arrivals,2) == 8)
                        arrivals_all = arrivals;
                    end
                end
            end
        end

    end

    % *** ADDED 03/06/12 ***
    try
        fid = fopen([uniqueArrays{i}(1:3) '.phi'],'w');
        for j=1:numel(InfraConfig.detect.az{i})
            fprintf(fid,'%s %7.4f %7.4f %7.4f %7.4f\n',datestr(InfraConfig.detect.time{i}(j),31),InfraConfig.detect.az{i}(j),InfraConfig.detect.slofk{i}(j),InfraConfig.detect.corr{i}(j),log10(InfraConfig.detect.pow{i}(j)));
        end
        fclose(fid);
    catch
        keyboard
    end
    % **********************

    try
        InfraConfig.array.arr{i} = arrivals_all;
    catch
        % No detections obtained!
        disp('No detections obtained...moving to next array')
        continue
    end

    InfraConfig.array.trc{i} = InfraConfig.trace.select;
    InfraConfig.array.loc{i} = GetArrayLoc(InfraConfig);

    try
        m = WriteArrivals(m,i);
        fid_det = fopen([wfdiscPath InfraConfig.db.name '.detect'],'a');
        ArrayName = InfraConfig.trace.name{InfraConfig.trace.select(1)};
        for j = 1:size(arrivals_all,1)
            fprintf(fid_det,'%s %s %s %4.1f %3.2f %3.2f\n',ArrayName,datestr(arrivals_all(j,1),'yyyy-mm-dd HH:MM:SS'),...
                datestr(arrivals_all(j,2),'yyyy-mm-dd HH:MM:SS'),arrivals_all(j,3),...
                111.1949*(1./arrivals_all(j,7)),arrivals_all(j,4));
        end
    catch
        % No detections obtained!
        continue
    end

    % Tidying Up:
    InfraConfig.detect = [];
    clear arrivals arrivals_all

end

% Tidying Up:
clear C F_K NRuns NSeconds NTrace NWindows a b chans dofden dofnum etime
clear etimes f_band f_order fid i j k l m npts_e npts_s p_etime p_stime
clear s s_f sacfiles stime stimes t_start wfdisc_file
pack

% Restructuring array structure:
try
    array_arr = 0; for i = 1:numel(InfraConfig.array.arr); array_arr(i) = size(InfraConfig.array.arr{i},1); end
catch
    return
end
I = find(array_arr ~= 0);   % Arrays with detections
for i = 1:numel(I)
    arr_tmp{i} = InfraConfig.array.arr{I(i)};
    trc_tmp{i} = InfraConfig.array.trc{I(i)};
    loc_tmp{i} = InfraConfig.array.loc{I(i)};
end
InfraConfig.array.arr = arr_tmp;
InfraConfig.array.trc = trc_tmp;
InfraConfig.array.loc = loc_tmp;

% Opening grid file:
try
	grid = load([wfdiscPath InfraConfig.db.name '.mat'],'-mat');
    for i = 1:numel(I)
        D_STN{i} = grid.grid.D_STN{I(i)};
        BA_STN{i} = grid.grid.BA_STN{I(i)};
    end
    grid.grid.D_STN = D_STN;
    grid.grid.BA_STN = BA_STN;
	InfraConfig.grid = grid.grid;
	clear grid
catch
	disp('InfraMonitor2b: Cannot find grid file');
end

% Saving detection results:
save InfraMonitor2b.mat

% Performing association and location:
try
    global toassoc InfraConfigBackup

    InfraConfigBackup.array = InfraConfig.array;
    InfraConfigBackup.grid = InfraConfig.grid;

    NArrays = numel(InfraConfigBackup.array.arr);
    narrays = (min_arrays:1:NArrays);

    C = progress('init','InfraMonitor2: Performing associations');
    for i = 1:numel(narrays)

        C = progress(C,i/numel(narrays));

        DoAssociation(narrays(i),NArrays);

        if (toassoc == 0)
            break
        end

    end

    InfraConfig.array = InfraConfigBackup.array;
    InfraConfig.grid = InfraConfigBackup.grid;

    k = 0; BISLMain2(k);
catch
    disp('No grid file...cannot perform associations/locations')
end
