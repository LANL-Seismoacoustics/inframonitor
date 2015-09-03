function Maketrace(NTrace)
% Maketrace - Generates the trace structure array
%
% New Functionality for InfraMonitor 2.0
%
% Stephen Arrowsmith (08/22/08)
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

global InfraConfig
InfraUser = open('InfraUser.mat');

%h = waitbar(0,'InfraMonitor2: Reading data...');
for i = 1:NTrace

    %waitbar(i/NTrace);

    j = InfraConfig.db.select(i);

    % Data start/end times:
    stime = epoch(InfraConfig.db.wfdisc{3}(j));
    etime = epoch(InfraConfig.db.wfdisc{7}(j));

    % Additional trace information:
    InfraConfig.trace.name{i} = InfraConfig.db.wfdisc{1}{j};
    InfraConfig.trace.s_f{i} = InfraConfig.db.wfdisc{9}(j);

    % Searching for filename (allowing for missed null operators in wfdisc file
    % after the 10th column):
    InfraConfig.trace.fname{i} = InfraConfig.db.wfdisc{17}{j};

    % Formatting full data path:
    if ( strncmp(InfraConfig.db.wfdisc{16}{i},'/',1) )  % Checking for relative or full path
      if (max(regexp(InfraConfig.db.wfdisc{16}{i},'/')) == numel(InfraConfig.db.wfdisc{16}{i})) % Checking for trailing /
        InfraConfig.db.dir_data = [ InfraConfig.db.wfdisc{16}{j}];
      else
        InfraConfig.db.dir_data = [ InfraConfig.db.wfdisc{16}{j} '/'];
      end
    else
      if (max(regexp(InfraConfig.db.wfdisc{16}{i},'/')) == numel(InfraConfig.db.wfdisc{16}{i}))
        InfraConfig.db.dir_data = [InfraConfig.db.dir InfraConfig.db.wfdisc{16}{j}];
      else
        InfraConfig.db.dir_data = [InfraConfig.db.dir InfraConfig.db.wfdisc{16}{j} '/'];
      end
    end

    %----------------------------------------------------------------------
%     for k = 10:numel(InfraConfig.db.wfdisc)
%         if (exist([InfraConfig.db.dir_data InfraConfig.trace.fname{i}],'file') == 0)
%             InfraConfig.trace.fname{i} = InfraConfig.db.wfdisc{k}{j};
%         else
%             break
%         end
%     end
%     if (exist([InfraConfig.db.dir_data InfraConfig.trace.fname{i}],'file') == 0)
%         errordlg('InfraMonitor2: Cannot read wfdisc file format')
%         clear global InfraConfig
%         close(h);
%         return
%     end
    %----------------------------------------------------------------------

    % Reading SAC or CSS data:
    sac_data = readsac([InfraConfig.db.dir_data InfraConfig.trace.fname{i}],'L');
    if (numel(sac_data) == 0)
        sac_data = readsac([InfraConfig.db.dir_data InfraConfig.trace.fname{i}],'B');
    end
    try
        NPts = numel(sac_data.DATA1);
    catch
        try
            sac_data = readcss([InfraConfig.db.dir_data InfraConfig.trace.fname{i}],...
                double(InfraConfig.db.wfdisc{8}(j)), ...
                InfraConfig.db.wfdisc{14}{j}, ...
                double(InfraConfig.db.wfdisc{18}(j)) );
        catch
            %close(h);
        end
    end

    % Taking time window of data:
    i1 = (InfraConfig.db.stime - stime)*86400*InfraConfig.trace.s_f{i} + 1;
    i2 = (InfraConfig.db.etime - stime)*86400*InfraConfig.trace.s_f{i} + 1;
    try
        % Extracting user-selected time window:
        InfraConfig.trace.data{i} = sac_data.DATA1(floor(i1):floor(i2));
        InfraConfig.trace.time{i} = linspace(InfraConfig.db.stime,...
            InfraConfig.db.etime,numel(InfraConfig.trace.data{i}));
    catch
        % Extracting full time series if end time exceeds length of SAC
        % file:
        try
            InfraConfig.trace.data{i} = sac_data.DATA1(floor(i1):numel(sac_data.DATA1));
        catch
            InfraConfig.trace.data{i} = cat(1,zeros(abs(i1),1),sac_data.DATA1);
        end
        InfraConfig.trace.time{i} = linspace(InfraConfig.db.stime,...
            etime,numel(InfraConfig.trace.data{i}));
    end

end
%close(h);

InfraConfig.array.loc = [];
InfraConfig.array.arr = [];
InfraConfig.array.trc = [];
