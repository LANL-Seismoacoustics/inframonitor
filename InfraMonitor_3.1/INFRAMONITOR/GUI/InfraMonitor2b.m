function InfraMonitor2b(wfdisc_file,arrays,f_band,f_order,min_arrays)
%
% InfraMonitor2b - Computes arrival, origin, IMassoc, and IMpoly tables
% from the command line
%
% e.g.,
% InfraMonitor2b('UTAH.wfdisc',{'BGU1,BGU2,BGU3,BGU4','EPU1,EPU2,EPU3,EPU4'
% ,'NOQ3,NOQ4,NOQ5,NOQ6'},[1 5],2);
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
global arrays

InfraUser = open('InfraUser.mat');
InfraUser.slow = linspace(-400,400,40);             % Infrasound

% Computing arrival table:

m = 0;

for i = 1:numel(arrays)

    chans = list2cell(arrays{i});

    % Reading data:
    fid = fopen(wfdisc_file,'r');
    InfraConfig.db.wfdisc = textscan(fid,...
        '%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s');
    InfraConfig.db.name = regexprep(wfdisc_file,'.wfdisc','');
    fclose(fid);

    % Extracting appropriate values from wfdisc file:
    stimes = []; etimes = [];
    l = 0;
    for j = 1:numel(InfraConfig.db.wfdisc{1})
        for k = 1:numel(chans)
            try
                if (InfraConfig.db.wfdisc{1}{j} == chans{k})
                    l = l + 1;
                    %---
                    InfraConfig.trace.name{l} = InfraConfig.db.wfdisc{1}{j};
                    %---
                    stimes = cat(1,stimes,epoch(str2double(InfraConfig.db.wfdisc{3}{j})));
                    etimes = cat(1,etimes,epoch(str2double(InfraConfig.db.wfdisc{7}{j})));

                    sacfilesdirs{l} = InfraConfig.db.wfdisc{16}{j};
                    sacfiles{l} = InfraConfig.db.wfdisc{17}{j};
                    nopoints{l} = InfraConfig.db.wfdisc{8}{j};
                    byteoffset{l} = InfraConfig.db.wfdisc{18}{j};
                    s_f = round(str2double(InfraConfig.db.wfdisc{9}{j}));
                    InfraConfig.trace.s_f{l} = s_f;
                end
            end
        end
    end

    % Computing processing start/end times:
    stime = max(stimes);
    etime = min(etimes);

    fid_procTimes = fopen('processingTimes','a');
    fprintf(fid_procTimes,'%s %s\n',datestr(stime),datestr(etime));
    fclose(fid_procTimes);

    % Reading and clipping waveforms:
    NoData = 0;
    for j = 1:numel(stimes)
        try
            s = readsac([sacfilesdirs{j} '/' sacfiles{j}],'L');
            if (numel(s) == 0)
                s = readsac([sacfilesdirs{j} '/' sacfiles{j}],'B');
            end
        catch
            disp('No data for array...moving on')
            NoData = 1;
            break
        end
        npts_s = (stime - stimes(j))*86400*s_f;

        if (npts_s == 0)
            npts_s = 1;
        end

        npts_e = (etimes(j) - etime)*86400*s_f;
        try
            x{j} = s.DATA1(npts_s:numel(s.DATA1)-npts_e);
        catch
            x_t = readcss([sacfilesdirs{j} '/' sacfiles{j}],str2double(nopoints{j}),'t4',str2double(byteoffset{j}));
            x{j} = x_t.DATA1;
        end
    end

    if (NoData == 1)
        continue
    end

    try
        % Filtering data:
        NTrace = numel(x);
        for j = 1:NTrace
            [b,a] = butter(f_order,[f_band(1) f_band(2)]/ceil(s_f/2));
            InfraConfig.trace.data{j} = filter(b,a,HanningWindow(x{j}));
            InfraConfig.trace.data{j} = InfraConfig.trace.data{j}.*7.9473e-06;  % ***
        end
    catch
        continue
    end

    clear x

    % Creating InfraConfig structure:
    fid = fopen([InfraConfig.db.name '.site']);
    i2 = 0;
    while 1
        i2 = i2 + 1;
        tline = fgetl(fid);
        if ~ischar(tline),   break,   end
        tline = regexp(tline,'\S*','match');
        if (i2 == 1)
            for j = 1:5
                InfraConfig.db.site{j} = tline(j);
            end
        else
            for j = 1:5
                InfraConfig.db.site{j} = cat(1,InfraConfig.db.site{j},tline{j});
            end
        end
    end
    fclose(fid);


    InfraConfig.db.f_band = f_band';
    InfraConfig.db.stime = stime;
    InfraConfig.trace.select = (1:numel(InfraConfig.trace.data));

    % From InfraMonitor2:
    NSeconds = (etime - stime)*86400;
    NRuns = ceil(NSeconds/InfraUser.w);

    % Processing data:
    % (Split data into time windows and process on each time window
    % separately)
    NWindows = ceil((numel(InfraConfig.trace.data{1})/s_f)/InfraUser.w);

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

        %***
        F_K = RunFK(InfraUser,p_stime,min(etime,p_etime),t_start,j,NWindows);
        %***
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

    % ******** ADDED 06/23/2009 ********
    fid = fopen([chans{1}(1:3) '.phi'],'w');
    for j=1:numel(InfraConfig.detect.az{i})
        fprintf(fid,'%s %7.4f %7.4f %7.4f %7.4f\n',datestr(InfraConfig.detect.time{i}(j),31),InfraConfig.detect.az{i}(j),InfraConfig.detect.slofk{i}(j),InfraConfig.detect.corr{i}(j),log10(InfraConfig.detect.pow{i}(j)));
    end
    fclose(fid);
    % **********************************

    try
        InfraConfig.array.arr{i} = arrivals_all;
    catch
        % No detections obtained!
        disp('No detections obtained...moving to next array')
        continue
    end

    InfraConfig.array.trc{i} = (1:numel(InfraConfig.trace.data));
    InfraConfig.array.loc{i} = GetArrayLoc(InfraConfig);

    try
        m = WriteArrivals(m,i);
        % *** Added 10/04/10 ***
        fid_det = fopen([InfraConfig.db.name '.detect'],'a');
        ArrayName = InfraConfig.trace.name{InfraConfig.trace.select(1)};
        for j = 1:size(arrivals_all,1)
            fprintf(fid_det,'%s %s %s %4.1f %3.2f %3.2f\n',ArrayName,datestr(arrivals_all(j,1),'yyyy-mm-dd HH:MM:SS'),...
                datestr(arrivals_all(j,2),'yyyy-mm-dd HH:MM:SS'),arrivals_all(j,3),...
                111.1949*(1./arrivals_all(j,7)),arrivals_all(j,4));
        end
        % **********************
    catch
        % No detections obtained!
        continue
    end

    % Tidying Up:
    InfraConfig.detect = []; InfraConfig.trace = [];
    clear arrivals arrivals_all

end
%
% Computing origin, IMassoc, and IMpoly tables:

% Tidying Up:
clear C F_K NRuns NSeconds NTrace NWindows a b chans dofden dofnum etime
clear etimes f_band f_order fid i j k l m npts_e npts_s p_etime p_stime
clear s s_f sacfiles stime stimes t_start wfdisc_file
pack

% *** Added 10/05/10 ***
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
% **********************

try
	% Opening grid file:
	grid = load([InfraConfig.db.name '.mat'],'-mat');

    % *** Added 10/05/10 ***
    for i = 1:numel(I)
        D_STN{i} = grid.grid.D_STN{I(i)};
        BA_STN{i} = grid.grid.BA_STN{I(i)};
    end
    grid.grid.D_STN = D_STN;
    grid.grid.BA_STN = BA_STN;
    % **********************

	InfraConfig.grid = grid.grid;
	clear grid
catch
	disp('InfraMonitor2b: Cannot find grid file');
end

save InfraMonitor2b.mat

%--------------------------------------------------------------------------
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
