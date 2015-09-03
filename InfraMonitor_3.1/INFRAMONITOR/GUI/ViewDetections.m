function ViewDetections()
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

% Obtaining Arrivals:
for i = 1:numel(InfraConfig.array.trc)
    if (numel(find(InfraConfig.array.trc{i} == InfraConfig.trace.select(1))) < 1)
        continue
    else
        arrivals = InfraConfig.array.arr{i}(:,1:2);
        elements = find(InfraConfig.array.arr{i}(:,5)==2);
        arrivals = arrivals(elements,:);
        break
    end
end

% Obtaining Additional Detection Parameters:
for i = 1:numel(InfraConfig.detect.trc)
    if (InfraConfig.detect.trc{i} == InfraConfig.trace.select)
        j = i;
        break
    end
end

if (exist('j') ~= 1)
    errordlg('InfraMonitor2: Ensure that corresponding traces are selected!')
    return
end

% Generating Figure Window:
scrsz = get(0,'ScreenSize');
figure('Position',[1 scrsz(4)/2 scrsz(3)/2 scrsz(4)/2])

% Plotting F-statistic:
ax(1) = subplot(5,1,1);
plot(InfraConfig.detect.time{j},InfraConfig.detect.fstat{j},'k.');
datetick('x')
ylabel('F-statistic')

% Plotting Correlation:
ax(2) = subplot(5,1,2);
plot(InfraConfig.detect.time{j},InfraConfig.detect.corr{j},'k.');
datetick('x')
ylabel('Correlation')
ylim([0 1])

% Plotting Azimuth:
ax(3) = subplot(5,1,3);
plot(InfraConfig.detect.time{j},InfraConfig.detect.az{j},'k.');
datetick('x')
ylabel('Azimuth')
ylim([0 360])

% Plotting Phase velocity:
ax(4) = subplot(5,1,4);
plot(InfraConfig.detect.time{j},InfraConfig.detect.slofk{j},'k.');
datetick('x')
ylabel('Phase velocity')
ylim([0 1])

% Plotting Waveform:
ax(5) = subplot(5,1,5);
hold on
for i = 1:numel(arrivals(:,1))
    rectangle('Position',[arrivals(i,1),-1,arrivals(i,2)-arrivals(i,1),2],...
        'EdgeColor',[0.5 0.5 0.5],'FaceColor',[0.5 0.5 0.5])
end
plot(InfraConfig.trace.time{InfraConfig.trace.select(1)},...
    InfraConfig.trace.data{InfraConfig.trace.select(1)}/...
    max(abs(InfraConfig.trace.data{InfraConfig.trace.select(1)})),'k');
datetick('x')
ylabel('Normalized Amp.')
xlabel('Time')

% Setting time axis properties:
linkaxes(ax,'x');
xlim([min(InfraConfig.trace.time{InfraConfig.trace.select(1)}) ...
    max(InfraConfig.trace.time{InfraConfig.trace.select(1)})])
