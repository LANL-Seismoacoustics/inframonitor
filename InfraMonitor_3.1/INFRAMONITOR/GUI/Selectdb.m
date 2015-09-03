function varargout = Selectdb(varargin)
% SELECTDB M-file for Selectdb.fig
%      SELECTDB, by itself, creates a new SELECTDB or raises the existing
%      singleton*.
%
%      H = SELECTDB returns the handle to a new SELECTDB or the handle to
%      the existing singleton*.
%
%      SELECTDB('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SELECTDB.M with the given input arguments.
%
%      SELECTDB('Property','Value',...) creates a new SELECTDB or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Selectdb_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Selectdb_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES
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

% Edit the above text to modify the response to help Selectdb

% Last Modified by GUIDE v2.5 24-Aug-2008 12:35:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Selectdb_OpeningFcn, ...
                   'gui_OutputFcn',  @Selectdb_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Selectdb is made visible.
function Selectdb_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Selectdb (see VARARGIN)

global InfraConfig

% Choose default command line output for Selectdb
handles.output = hObject;

% Update list box with wfdisc file summary:
for i = 1:numel(InfraConfig.db.wfdisc{1})
    try
    line = strcat(InfraConfig.db.wfdisc{1}{i},'____',...
        InfraConfig.db.wfdisc{2}{i},'____',....
        datestr(epoch(InfraConfig.db.wfdisc{3}(i)),'mm/dd/yyyy HH:MM:SS.FFF'),'____',...
        datestr(epoch(InfraConfig.db.wfdisc{7}(i)),'mm/dd/yyyy HH:MM:SS.FFF'));
    catch
        keyboard
    end
    if (i == 1)
        wfdisc = line;
    else
        wfdisc = strvcat(wfdisc,line);
    end
end
set(handles.listbox1,'String',wfdisc)

% Calculating minimum start time:
for i = 1:numel(InfraConfig.db.wfdisc{3})
    StartTime(i) = InfraConfig.db.wfdisc{3}(i);
    EndTime(i) = InfraConfig.db.wfdisc{7}(i);
end
stime = epoch(min(StartTime)+1);
etime = epoch(max(EndTime)-1);

dur = (etime-stime)*24;

set(handles.edit1,'String',datestr(stime,'mm/dd/yyyy HH:MM:SS.FFF'))
set(handles.edit2,'String',[num2str(dur) ' h']);

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Selectdb wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Selectdb_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function listbox1_Callback(hObject, eventdata, handles)


function listbox1_CreateFcn(hObject, eventdata, handles)

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pushbutton1_Callback(hObject, eventdata, handles)

global InfraConfig
InfraConfig.db.select = get(handles.listbox1,'Value');
stime = datenum(get(handles.edit1,'String'),'mm/dd/yyyy HH:MM:SS.FFF');

etime = get(handles.edit2,'String');
etime_break = regexp(etime,' ');
etime1 = str2double(etime(1:etime_break));
etime2 = etime(etime_break+1:numel(etime));

switch etime2
    case 'h'
        etime_o = stime + etime1/24;
    case 'm'
        etime_o = stime + etime1/(24*60);
    case 's'
        etime_o = stime + etime1/(24*60*60);
    otherwise
        msgbox('Enter h (hours), m (mins), or s (secs)')
end

try
    InfraConfig.db.stime = stime;
    InfraConfig.db.etime = etime_o;
catch
    return
end

delete(handles.figure1)


function edit1_Callback(hObject, eventdata, handles)


function edit1_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit2_Callback(hObject, eventdata, handles)


function edit2_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


