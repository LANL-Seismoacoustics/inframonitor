function cleanTrace
%
% Combines separate traces for each array, adding zeros to fill any data
% gaps
%
% Notes:
% - Requires traces to be ordered by time in the wfdisc file (this is
%   typical)
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

uniqueStations = unique(InfraConfig.trace.name);

for i = 1:numel(uniqueStations)

    % Merge each station (assuming stations are in sequential order):
    k = 0;
    for j = 1:numel(InfraConfig.trace.name)
        if (strcmp(uniqueStations{i},InfraConfig.trace.name{j}))
            k = k + 1;
            if (k == 1)
                data = InfraConfig.trace.data{j};
                time = InfraConfig.trace.time{j}';
                s_f = InfraConfig.trace.s_f{j};
            else
                maxOldTime = max(time);
                I = find(InfraConfig.trace.data{j} ~= 0);
                minNewTime = min(InfraConfig.trace.time{j}(I));
                timeLag = minNewTime*86400 - maxOldTime*86400;      % time lag needs to be filled with zeros
                numSamples = round2(timeLag,1/s_f)*s_f;             % this many samples fill the gap

                if (numSamples < 0)
                    % Overlapping traces (NOT TESTED)
                    data = cat(1,data(1:numel(data)-(abs(numSamples)+1)),InfraConfig.trace.data{j}(I(1):numel(InfraConfig.trace.data{j})));
                    time = cat(1,time(1:numel(time)-(abs(numSamples)+1)),InfraConfig.trace.time{j}(I(1):numel(InfraConfig.trace.time{j}))');
                elseif (numSamples == 0)
                    % No data gap (NOT TESTED)
                    data = cat(1,data(1:numel(data)-1),InfraConfig.trace.data{j}(I(1):numel(InfraConfig.trace.data{j})));
                    time = cat(1,time(1:numel(time)-1),InfraConfig.trace.time{j}(I(1):numel(InfraConfig.trace.time{j}))');
                else

                    % Filling data gap with zeros
                    % *** DEAN CLAUTER IDENTIFIED PROBLEM 5/2/12 ***
                    try
                        data = cat(1,data,zeros(numSamples,1),InfraConfig.trace.data{j}(I(1):numel(InfraConfig.trace.data{j})));
                    catch
                        disp('Dimensions for concatenating:')
                        disp(size(data))
                        disp(size(zeros(numSamples,1)))
                        disp(size(InfraConfig.trace.data{j}(I(1):numel(InfraConfig.trace.data{j}))))
                        keyboard
                    end

                    try
                        timesInBetween = linspace(maxOldTime + (1/s_f)/86400, minNewTime - (1/s_f)/86400, numSamples);
                    catch
                        time = InfraConfig.trace.time{j}';
                        continue
                    end
                    time = cat(1,time,timesInBetween',InfraConfig.trace.time{j}(I(1):numel(InfraConfig.trace.time{j}))');
                end

            end
        end
    end

    segment = find(InfraConfig.db.stime <= time & time <= InfraConfig.db.etime);
    trace.name{i} = uniqueStations{i};
    trace.s_f{i} = s_f;
    trace.data{i} = data(segment);
    trace.time{i} = time(segment)';

end

InfraConfig.trace = trace;
