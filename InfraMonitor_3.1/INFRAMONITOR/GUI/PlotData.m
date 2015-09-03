function PlotData(handles,NTrace,iflag)
% PlotData - Plots waveform data in the main InfraMonitor GUI window
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

try
    set(handles.figure1, 'CurrentAxes', handles.axes1)
catch
    keyboard
end

try
    title(['Filtered Data: ' num2str(InfraConfig.db.f_band(1)) '-' num2str(InfraConfig.db.f_band(2)) ' Hz']);
catch
    title('Unfiltered data');
end

if (iflag == 1)
    x = get(gca,'XLim');
    y = get(gca,'YLim');
end

hold on

% Plotting arrivals:
try
    for i = 1:numel(InfraConfig.array.loc)

        y_min = min(InfraConfig.trace.y_ax(InfraConfig.array.trc{i}));
        y_max = max(InfraConfig.trace.y_ax(InfraConfig.array.trc{i}));

        for j = 1:numel(InfraConfig.array.arr{i}(:,1))
            if (numel(InfraConfig.array.arr{i}) == 1)
                continue
            elseif (InfraConfig.array.arr{i}(j,5) == 1)
                continue
            end
            t1 = InfraConfig.array.arr{i}(j,1);
            t2 = InfraConfig.array.arr{i}(j,2);

            try
                if (numel(find(InfraConfig.array.select{i} == j)) > 0)
                    rectangle('Position',[t1,y_min-1,t2-t1,(y_max+1)-(y_min-1)],...
                        'FaceColor',[0.25 0.5 0.25])
                else
                    rectangle('Position',[t1,y_min-1,t2-t1,(y_max+1)-(y_min-1)],...
                        'FaceColor',[0.25 0.25 0.25])
                end
            catch
                try
                    rectangle('Position',[t1,y_min-1,t2-t1,(y_max+1)-(y_min-1)],...
                        'FaceColor',[0.25 0.25 0.25])
                catch
                    continue
                end
            end

        end

    end
end

for i = 1:NTrace

    % Obtaining ground truth parameters:
    try
        %keyboard
        for j = 1:numel(InfraConfig.gt.t0)
            [slat,slon] = GetLoc(InfraConfig.trace.name{i});
            Dist = vdist(slat,slon,InfraConfig.gt.loc0{j}(1),InfraConfig.gt.loc0{j}(2))/1000;
            GTTime1{j}(i,1) = InfraConfig.gt.t0{j} + (Dist/InfraConfig.gt.v_g{j}(1))/86400;
            GTTime2{j}(i,1) = InfraConfig.gt.t0{j} + (Dist/InfraConfig.gt.v_g{j}(2))/86400;
            GTTime_Pn{j}(i,1) = InfraConfig.gt.t0{j} + (Dist/7.9)/86400;
            GTTime_Pg{j}(i,1) = InfraConfig.gt.t0{j} + (Dist/6.0)/86400;
            GTTime_Lg{j}(i,1) = InfraConfig.gt.t0{j} + (Dist/3.5)/86400;

            %---
            InfraConfig.trace.gt1{j}(i) = GTTime1{j}(i,1);
            InfraConfig.trace.gt2{j}(i) = GTTime2{j}(i,1);
            [d,AA] = vdist(slat,slon,InfraConfig.gt.loc0{j}(1),InfraConfig.gt.loc0{j}(2));
            InfraConfig.trace.gt_azi{j}(i) = AA;
            InfraConfig.trace.GTTime_Pn{j}(i) = GTTime_Pn{j}(i,1);
            InfraConfig.trace.GTTime_Pg{j}(i) = GTTime_Pg{j}(i,1);
            InfraConfig.trace.GTTime_Lg{j}(i) = GTTime_Lg{j}(i,1);
            %---

            GTTime1{j}(i,2) = 2*(i-1);
            GTTime2{j}(i,2) = 2*(i-1);
            GTTime_Pn{j}(i,2) = 2*(i-1);
            GTTime_Pg{j}(i,2) = 2*(i-1);
            GTTime_Lg{j}(i,2) = 2*(i-1);
        end
    end

    if (numel(find(InfraConfig.trace.select == i)) == 0)
        try
            plot(InfraConfig.trace.time{i},((InfraConfig.trace.data{i}/...
                max(abs(InfraConfig.trace.data{i}))))*InfraConfig.mag+2*(i-1),'k')
        catch
            plot(InfraConfig.trace.time{i},(InfraConfig.trace.data{i}/...
                max(abs(InfraConfig.trace.data{i})))+2*(i-1),'k')
        end
    else
        try
            plot(InfraConfig.trace.time{i},((InfraConfig.trace.data{i}/...
                max(abs(InfraConfig.trace.data{i}))))*InfraConfig.mag+2*(i-1),'r')
        catch
            plot(InfraConfig.trace.time{i},(InfraConfig.trace.data{i}/...
                max(abs(InfraConfig.trace.data{i})))+2*(i-1),'r')
        end
    end

    InfraConfig.trace.y_ax(i) = 2*(i-1);

    try
        y_lb = strvcat(y_lb,[num2str(i) ' ' InfraConfig.trace.name{i}]);
    catch
        y_lb = [num2str(i) ' ' InfraConfig.trace.name{i}];
    end

end
datetick('x')

try
    for j = 1:numel(InfraConfig.gt.t0)
        %keyboard
        plot(GTTime1{j}(:,1),GTTime1{j}(:,2),'g')
        plot(GTTime2{j}(:,1),GTTime2{j}(:,2),'g')
        plot(GTTime_Pn{j}(:,1),GTTime_Pn{j}(:,2),'r')
        plot(GTTime_Pg{j}(:,1),GTTime_Pg{j}(:,2),'b')
        plot(GTTime_Lg{j}(:,1),GTTime_Lg{j}(:,2),'m')
    end
end

set(gca,'YTick',[])
set(gca,'YTickLabel',[])
set(gca,'YTick',InfraConfig.trace.y_ax)
set(gca,'YTickLabel',y_lb)
set(gca,'Color','w')

if (iflag == 1)
    set(gca,'XLim',x)
    set(gca,'YLim',y)
%     xlim([min(InfraConfig.trace.time{1}) max(InfraConfig.trace.time{1})])
end

set(handles.axes1, 'Visible', 'on');

hold off
