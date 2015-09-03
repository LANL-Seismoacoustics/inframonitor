function dcmH = customDataCursor(h, datalabels)
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

% customDataCursor allows a user to label each point in a data series with
% custom labels. When the data cursor mode is enabled (DATACURSORMODE ON),
% clicking on a data point will display the custom label, then the x and y
% values of the data point. If more than one data point of the data series
% is in the same location, the data cursor will display all of the labels
% first, then the x and y locations.
%
% Usage: customDataCursor(H,DATALABELS) applies the custom labels found in
% cell array DATALABELS to the line with handle H. DATALABELS must be a
% cell array with the same length as 'XDATA' and 'YDATA' in the line; in
% other words, there must be a label for each data point in the line.
%
% dcmH = customDataCursor(H,DATALABELS) returns the handle to the data
% cursor in dcmH.
%
% Note: CUSTOMDATACURSOR uses the 'UserData' property of a line to store
% and retrieve the labels. If that property is used by another portion of a
% user's program, this function may be broken, or this function may break
% the user's program.
%
% Example:
%   % generate some data and chart it
%   N = 20;
%   x = rand(N,1);
%   y = rand(N,1);
%   h = plot(x,y,'ko');
%   % make up some labels and apply them to the chart
%   labels = cell(N,1);
%   for ii = 1:N
%       labels{ii} = ii;
%   end
%   customDataCursor(h,labels)

% Check input arguments
if ~exist('h','var')||~exist('datalabels','var')
    error('Improper inputs to function customDataCursor')
elseif ~ishandle(h)
    error('Nonexistent handle passed to function customDataCursor')
elseif length(datalabels) ~= length(get(h,'xdata'))
    error(['Error in input to function customDataCursor: '...
        'number of labels is different than the number of data points in the line'])
end

% Put the labels in the 'userdata' property of the line
set(h,'userdata',datalabels)
% find the handle for the data cursor; set the update function
dcmH = datacursormode(gcf);
set(dcmH,'UpdateFcn',@cursorUpdateFcn)

function output_txt = cursorUpdateFcn(obj,event_obj)
% Display the position of the data cursor
% obj          Currently not used (empty)
% event_obj    Handle to event object
% output_txt   Data cursor text string (string or cell array of strings).

% position of the data point to label
pos = get(event_obj,'Position');
% read in the labels from 'UserData' of the line
labels = get(get(event_obj,'Target'),'UserData');
% read the x and y data
xvals = get(get(event_obj,'Target'),'XData');
yvals = get(get(event_obj,'Target'),'YData');
% now figure out which data point was selected
datapoint = find( (xvals==pos(1))&(yvals==pos(2)) );

% create the text to be displayed
output_txt = { labels{datapoint};...
    ['X: ',num2str(pos(1),4)];...
    ['Y: ',num2str(pos(2),4)] };

% If there is a Z-coordinate in the position, display it as well
if length(pos) > 2
    output_txt{end+1} = ['Z: ',num2str(pos(3),4)];
end
