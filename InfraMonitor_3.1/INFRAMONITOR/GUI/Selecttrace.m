function handles = Selecttrace(handles,keypress)
% Selecttrace - Allows plot selection functionality
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

if (keypress == 's')
    waitforbuttonpress
    point1 = get(gca,'CurrentPoint');    % button down detected
    x1 = point1(1,1); y1 = point1(1,2);
    finalRect = rbbox;                   % return figure units
    point2 = get(gca,'CurrentPoint');    % button up detected
    x2 = point2(1,1); y2 = point2(1,2);
    [a,b1] = min(abs(InfraConfig.trace.y_ax - y1));
    [a,b2] = min(abs(InfraConfig.trace.y_ax - y2));
    b = (min(b1,b2):1:max(b1,b2));
    InfraConfig.trace.select = b;
    NTrace = numel(InfraConfig.db.select);
    PlotData(handles,NTrace,1);
elseif (keypress == 'c')
    waitforbuttonpress
    point1 = get(gca,'CurrentPoint');    % button down detected
    x1 = point1(1,1); y1 = point1(1,2);
    finalRect = rbbox;                   % return figure units
    point2 = get(gca,'CurrentPoint');    % button up detected
    x2 = point2(1,1); y2 = point2(1,2);
    try
        load DataOrig.mat
        for i = 1:numel(InfraConfig.trace.time)
            [a,b] = find(x1 <= InfraConfig.trace.time{i} & InfraConfig.trace.time{i} <= x2);
            InfraConfig.trace.time{i} = InfraConfig.trace.time{i}(min(b):max(b));
            InfraConfig.trace.data{i} = InfraConfig.trace.data{i}(min(b):max(b));
            data_orig{i} = data_orig{i}(min(b):max(b));
        end
        save DataOrig.mat data_orig
    catch
        for i = 1:numel(InfraConfig.trace.time)
            [a,b] = find(x1 <= InfraConfig.trace.time{i} & InfraConfig.trace.time{i} <= x2);
            InfraConfig.trace.time{i} = InfraConfig.trace.time{i}(min(b):max(b));
            InfraConfig.trace.data{i} = InfraConfig.trace.data{i}(min(b):max(b));
        end
    end
    delete(handles.axes1);
    handles.axes1 = axes('Parent',handles.figure1,'Units','characters',...
        'Position',[8.16667 2.33333 172.667 49.1667]);
    NTrace = numel(InfraConfig.db.select);
    PlotData(handles,NTrace,0)
elseif (keypress == 'a')

    [x,y] = ginput(1);

    % Calculating a pointer to the appropriate arrivals matrix (j):
    [u,I] = min(abs(y - InfraConfig.trace.y_ax));
    for i = 1:numel(InfraConfig.array.trc)
        if (numel(find(InfraConfig.array.trc{i} == I)) > 0)
            j = i;
        end
    end

    if (exist('j','var') == 0)
        return
    end

    % Calculating a pointer to the appropriate arrival in the arrivals
    % matrix (k):
    [u,k] = min(abs(x - InfraConfig.array.arr{j}(:,1)));

    try
        if (numel(find(InfraConfig.array.select{j} == k)) == 0)
            InfraConfig.array.select{j} = union(InfraConfig.array.select{j},k);
        else
            InfraConfig.array.select{j} = setdiff(InfraConfig.array.select{j},k);
        end
    catch
        InfraConfig.array.select{j} = k;
    end

    NTrace = numel(InfraConfig.db.select);
    PlotData(handles,NTrace,1);

else
    return
end
