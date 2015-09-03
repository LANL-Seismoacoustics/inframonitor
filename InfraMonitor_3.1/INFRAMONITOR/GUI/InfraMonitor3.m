function varargout = InfraMonitor3(varargin)
% INFRAMONITOR3 M-file for InfraMonitor3.fig
%      INFRAMONITOR3, by itself, creates a new INFRAMONITOR3 or raises the existing
%      singleton*.
%
%      H = INFRAMONITOR3 returns the handle to a new INFRAMONITOR3 or the handle to
%      the existing singleton*.
%
%      INFRAMONITOR3('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INFRAMONITOR3.M with the given input arguments.
%
%      INFRAMONITOR3('Property','Value',...) creates a new INFRAMONITOR3 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before InfraMonitor3_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to InfraMonitor3_OpeningFcn via varargin.
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

% Edit the above text to modify the response to help InfraMonitor3

% Last Modified by GUIDE v2.5 27-Mar-2012 12:41:39

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @InfraMonitor3_OpeningFcn, ...
                   'gui_OutputFcn',  @InfraMonitor3_OutputFcn, ...
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


% --- Executes just before InfraMonitor3 is made visible.
function InfraMonitor3_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to InfraMonitor3 (see VARARGIN)

clear global InfraConfig InfraConfigBackup assoc toassoc AssocIn
warning off all

% h = About;
% waitfor(h)

% Choose default command line output for InfraMonitor3
handles.output = hObject;

set(handles.figure1,'KeyPressFcn',@KeyPressFcn)

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes InfraMonitor3 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = InfraMonitor3_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function KeyPressFcn(src,evnt)
handles = guidata(src);
keypress = get(handles.figure1, 'CurrentCharacter');

handles = Selecttrace(handles,keypress);

guidata(src, handles);


function Menu_File_Callback(hObject, eventdata, handles)


function Menu_Detect_Callback(hObject, eventdata, handles)


function Menu_Locate_Callback(hObject, eventdata, handles)


function Menu_Locate_Grid_Callback(hObject, eventdata, handles)


function Menu_Locate_Run_Callback(hObject, eventdata, handles)


function Menu_Grid_Locate_Open_Callback(hObject, eventdata, handles)


function Menu_Locate_Grid_Save_Callback(hObject, eventdata, handles)
msgbox('InfraMonitor2: CSS3.0 event/polygon tables will be implemented in official release')


function Menu_Detect_Run_Callback(hObject, eventdata, handles)

% Loading global variables:
global InfraConfig

try
    if (numel(InfraConfig.trace.select) == 0)
        errordlg('InfraMonitor2: Select the traces to process')
        return
    end
catch
    errordlg('InfraMonitor2: Must read data before applying detector')
    return
end

%Modified 09/2009 by RTM---------------------------------------------------
% % Checking to see if array has already been processed:
% for i = 1:numel(InfraConfig.array.trc)
%     if (isempty(intersect(InfraConfig.array.trc{1},InfraConfig.trace.select)) ~= 1)
%         button = questdlg('Are you sure you want to overwrite existing detections?');
%         if (strcmp(button,'Yes') ~= 1)
%             return
%         end
%         Overwrite = i;
%     end
% end
%Original------------------------------------------------------------------
% Checking to see if array has already been processed:
for i = 1:numel(InfraConfig.array.trc)
    try
        if (InfraConfig.array.trc{i} == InfraConfig.trace.select)
            button = questdlg('Are you sure you want to overwrite existing detections?');
            if (strcmp(button,'Yes') ~= 1)
                return
            end
            Overwrite = i;
        end
    end
end
%--------------------------------------------------------------------------

InfraUser = open('InfraUser.mat');
InfraUser.slow = linspace(-400,400,40);             % Infrasound (default=40, need to update makeMask.m if this is changed)

try
    NSeconds = (InfraConfig.db.etimeNew - InfraConfig.db.stimeNew)*86400;
catch
    NSeconds = (InfraConfig.db.etime - InfraConfig.db.stime)*86400;
end

NRuns = ceil(NSeconds/InfraUser.w);

for i = 1:NRuns

    if (i == 1)
        try
            stime = InfraConfig.db.stimeNew;
        catch
            stime = InfraConfig.db.stime;
        end
        etime = stime + InfraUser.w/86400;
    else
        stime = etime;
        etime = stime + InfraUser.w/86400;
    end

    t_start = (stime*86400 - InfraConfig.db.stime*86400)*...
        InfraConfig.trace.s_f{InfraConfig.trace.select(1)} + 1;

	% ***
    % Added 03/19/10
    itestflag = 0;
    for itest = 1:numel(InfraConfig.trace.data)
        if (numel(InfraConfig.trace.data{itest}) < floor(t_start))
            if (numel(find(InfraConfig.trace.select == itest)) ~= 0)
                itestflag = 1;
            end
        end
    end

    if (itestflag == 1)
        continue
    end

    % ***

    try
        F_K = RunFK(InfraUser,stime,min(etime,InfraConfig.db.etimeNew),t_start,i,NRuns);
    catch
        F_K = RunFK(InfraUser,stime,min(etime,InfraConfig.db.etime),t_start,i,NRuns);
    end

    try
        dofnum = 0.5*(2*InfraUser.twin*(InfraConfig.db.f_band(2)...
            -InfraConfig.db.f_band(1)));
    catch
        errordlg('InfraMonitor2: Must filter data before running detector')
        return
    end
    dofden = 0.5*(2*InfraUser.twin*(InfraConfig.db.f_band(2)...
        -InfraConfig.db.f_band(1)))*(numel(InfraConfig.trace.select)-1);
    [arrivals,C] = Detect(F_K,dofnum,dofden,InfraUser);

    try
        j = numel(InfraConfig.array.loc) + 1;
    catch
        j = 1;
    end

    if (i == 1)
        try
            InfraConfig.detect.time{Overwrite} = F_K.time;
            InfraConfig.detect.slofk{Overwrite} = F_K.slofk;
            InfraConfig.detect.az{Overwrite} = F_K.az;
            InfraConfig.detect.fstat{Overwrite} = F_K.fstat/C;
            InfraConfig.detect.corr{Overwrite} = F_K.corr;
            InfraConfig.detect.trc{Overwrite} = InfraConfig.trace.select;
            if (size(arrivals,2) == 8)
                arrivals_all = arrivals;
            end
        catch
            InfraConfig.detect.time{j} = F_K.time;
            InfraConfig.detect.slofk{j} = F_K.slofk;
            InfraConfig.detect.az{j} = F_K.az;
            InfraConfig.detect.fstat{j} = F_K.fstat/C;
            InfraConfig.detect.corr{j} = F_K.corr;
            InfraConfig.detect.trc{j} = InfraConfig.trace.select;
            if (size(arrivals,2) == 8)
                arrivals_all = arrivals;
            end
        end
    else
        try
            InfraConfig.detect.time{Overwrite} = cat(2,InfraConfig.detect.time{Overwrite},F_K.time);
            InfraConfig.detect.slofk{Overwrite} = cat(2,InfraConfig.detect.slofk{Overwrite},F_K.slofk);
            InfraConfig.detect.az{Overwrite} = cat(2,InfraConfig.detect.az{Overwrite},F_K.az);
            InfraConfig.detect.fstat{Overwrite} = cat(2,InfraConfig.detect.fstat{Overwrite},F_K.fstat/C);
            InfraConfig.detect.corr{Overwrite} = cat(2,InfraConfig.detect.corr{Overwrite},F_K.corr);
            try
                arrivals_all = cat(1,arrivals_all,arrivals);
            catch
                if (exist('arrivals_all','var') == 0)
                    if (size(arrivals,2) == 8)
                        arrivals_all = arrivals;
                    end
                end
            end
        catch
            InfraConfig.detect.time{j} = cat(2,InfraConfig.detect.time{j},F_K.time);
            InfraConfig.detect.slofk{j} = cat(2,InfraConfig.detect.slofk{j},F_K.slofk);
            InfraConfig.detect.az{j} = cat(2,InfraConfig.detect.az{j},F_K.az);
            InfraConfig.detect.fstat{j} = cat(2,InfraConfig.detect.fstat{j},F_K.fstat/C);
            InfraConfig.detect.corr{j} = cat(2,InfraConfig.detect.corr{j},F_K.corr);
            try
                arrivals_all = cat(1,arrivals_all,arrivals);
            catch
                if (exist('arrivals_all','var') == 0)
                try
                    if (size(arrivals,2) == 8)
                        arrivals_all = arrivals;
                    end
                catch
                    keyboard
                end
                end
            end
        end
    end

end

if (exist('arrivals_all','var') == 0)
    errordlg('InfraMonitor2: No detections found')
    return
end

try
    InfraConfig.array.loc{Overwrite} = GetArrayLoc(InfraConfig);
    InfraConfig.array.arr{Overwrite} = arrivals_all;
    InfraConfig.array.trc{Overwrite} = InfraConfig.trace.select;
catch
    i = numel(InfraConfig.array.loc);
    i = i + 1;
    InfraConfig.array.loc{i} = GetArrayLoc(InfraConfig);
    InfraConfig.array.arr{i} = arrivals_all;
    InfraConfig.array.trc{i} = InfraConfig.trace.select;
end

pos = get(handles.axes1,'Position');
delete(handles.axes1);
handles.axes1 = axes('Parent',handles.figure1,'Position',pos);

% delete(handles.axes1);
% handles.axes1 = axes('Parent',handles.figure1,'Units','characters',...
%     'Position',[8.16667 2.33333 172.667 49.1667]);

% PlotData(handles,numel(InfraConfig.db.select),0);
PlotData(handles,numel(InfraConfig.trace.name),0);       % Changed 10/18/11
disp('InfraMonitor2: Detection Process Completed')
guidata(hObject, handles)


function Menu_Detect_Open_Callback(hObject, eventdata, handles)


function Menu_Detect_Save_Callback(hObject, eventdata, handles)
l = WriteArrivals(0,0);


function Menu_File_Open_Callback(hObject, eventdata, handles)


function Menu_File_Close_Callback(hObject, eventdata, handles)
clear global InfraConfig

pos = get(handles.axes1,'Position');
delete(handles.axes1);
handles.axes1 = axes('Parent',handles.figure1,'Position',pos);

% delete(handles.axes1);
% handles.axes1 = axes('Parent',handles.figure1,'Units','characters',...
%     'Position',[8.16667 2.33333 172.667 49.1667]);
set(handles.axes1,'Visible','off')
guidata(hObject, handles);


function Menu_File_Quit_Callback(hObject, eventdata, handles)
user_response = closedlg;
switch user_response
case {'No'}
case 'Yes'
clear global InfraConfig
delete(handles.figure1)
delete('DataOrig.mat')
end


function Menu_Locate_Grid_New_Callback(hObject, eventdata, handles)
NewGrid()


function Menu_Locate_Grid_Open_Callback(hObject, eventdata, handles)

global InfraConfig

[filename,pathname] = uigetfile('*.mat','Select grid file...');
if (filename == 0)
    return
end

grid = load([pathname filename]);

try
    InfraConfig.grid = grid.grid;
catch
    errordlg('InfraMonitor2: Incompatible grid file')
    return
end

function File_Save_Callback(hObject, eventdata, handles)

global InfraConfig assoc

[filename,pathname] = uiputfile('Config.mat','Save Configuration file');
if (filename == 0)
    return
end

save([pathname filename],'InfraConfig','assoc')


function File_Open_CSS3_Callback(hObject, eventdata, handles)

global InfraConfig

% Obtaining wfdisc file details:
[filename,pathname] = uigetfile('*.wfdisc','Select wfdisc file...');
if (filename == 0)
    return
end

% currentdir = [pwd '/'];
% if (strcmp(pathname,currentdir) == 0)
%     errordlg('Data is not in working directory - change working directory in Matlab')
%     return
% end

% Reading wfdisc and site files:
InfraConfig.db.dir = pathname;
InfraConfig.db.name = regexprep(filename,'.wfdisc','');
try
    InfraConfig.db.wfdisc = readWfdisc([pathname filename],'css3.0');
    InfraConfig.db.site = readSite([pathname InfraConfig.db.name '.site'],'css3.0');
catch
    errordlg('Cant read in wfdisc AND site files in css3.0 format. Check both files exist; if they do, try using nnsa format');
    return;
end


% Allowing user selection of traces and time window (Configuring
% InfraConfig.db structure):
h = Selectdb;
waitfor(h)
set(handles.stime,'String',datestr(InfraConfig.db.stime,'mm/dd/yyyy HH:MM:SS'));
set(handles.etime,'String',datestr(InfraConfig.db.etime,'mm/dd/yyyy HH:MM:SS'));

% Storing trace data as a structure (trace):
NTrace = numel(InfraConfig.db.select);
try
    Maketrace(NTrace);
catch
    if (get(handles.radioButton1,'Value') == 1)
        errordlg('Try changing the schema type to nnsa')
    else
        errordlg('Try changing the schema type to css3.0')
    end
    return
end

cleanTrace;
NTrace = numel(InfraConfig.trace.name);

% Adding refsta information if available:
try
    for i = 1:numel(InfraConfig.trace.name)
        strcmpNos = strcmp(InfraConfig.trace.name{i},InfraConfig.db.site{1});
        I = find(strcmpNos == 1);
        InfraConfig.trace.refsta{i} = InfraConfig.db.site{9}{I};
    end
end

try
    % Plotting waveform data:
    InfraConfig.trace.select = [];
    PlotData(handles,NTrace,0);
    guidata(hObject, handles);
end

function File_Open_Configuration_Callback(hObject, eventdata, handles)

global InfraConfig

[filename,pathname] = uigetfile('*.mat','Select configuration file...');
if (filename == 0)
    return
end

load([pathname filename])
PlotData(handles,numel(InfraConfig.db.select),0);
set(handles.stime,'String',datestr(InfraConfig.db.stime,'mm/dd/yyyy HH:MM:SS'));
set(handles.etime,'String',datestr(InfraConfig.db.etime,'mm/dd/yyyy HH:MM:SS'));


function Menu_Detect_View_Callback(hObject, eventdata, handles)

global InfraConfig

try

    % Obtaining Arrivals:
    for i = 1:numel(InfraConfig.array.trc)
        if (numel(find(InfraConfig.array.trc{i} == InfraConfig.trace.select(1))) < 1)
            continue
        else
            try
                arrivals = InfraConfig.array.arr{i}(:,1:8);         % 2/3/11 Changed from 2 to 8!
                elements = find(InfraConfig.array.arr{i}(:,5)==2);
                arrivals = arrivals(elements,:);
                break
            end
        end
    end

    % Obtaining Additional Detection Parameters:
    for i = 1:numel(InfraConfig.detect.trc)
        try
            if (InfraConfig.detect.trc{i} == InfraConfig.trace.select)
                j = i;
                break
            end
        end
    end

    if (exist('j') ~= 1)
        errordlg('InfraMonitor2: Ensure that corresponding traces are selected!')
        return
    end

    % Obtaining Channel Names:
    for i = 1:numel(InfraConfig.trace.select)
        try
            Chans = strcat([Chans ','],InfraConfig.trace.name{InfraConfig.trace.select(i)});
        catch
            Chans = InfraConfig.trace.name{InfraConfig.trace.select(i)};
        end
    end

    [Selection,ok] = listdlg('PromptString','Select channels to plot:',...
        'ListString',list2cell([Chans,',beam']));

    if (ok == 0)
        return
    end

    if (exist('arrivals') == 0)
        arrivals = -999;
    end

    IMPlot(arrivals,Chans,Selection,j);
catch
    errordlg('InfraMonitor2: Ensure that detector has been run and corresponding traces are selected!')
end


function Menu_File_Print_Callback(hObject, eventdata, handles)
set(gcf,'PaperPositionMode','auto')
printpreview


function Menu_Filter_Apply_Callback(hObject, eventdata, handles)
global InfraConfig

button = questdlg('Filter only selected traces?');
if (strcmp(button,'Yes') == 1)
    selected_only = 1;
else
    selected_only = 0;
end

% Obtaining filter band:
filtering = inputdlg({'InfraMonitor2: Enter filter band','Enter Order'},...
    'Filtering',1,{'1,5','4'});
try
    f_band = list2array(filtering{1});
    order = str2double(filtering{2});
catch
    return
end

% Filtering data:
try
    NTrace = numel(InfraConfig.trace.s_f);
catch
    errordlg('InfraMonitor2: Ensure that data are read in')
    return
end

data_orig = InfraConfig.trace.data;
save DataOrig.mat data_orig
clear data_orig

if (selected_only == 1)
    for i = 1:NTrace
        if (numel(f_band) > 1)
            [b,a] = butter(order,[f_band(1) f_band(2)]/ceil(InfraConfig.trace.s_f{i}/2));
        else
            [b,a] = butter(order,f_band(1)/ceil(InfraConfig.trace.s_f{i}/2),'high');
        end
        if (numel(find(InfraConfig.trace.select == i)) == 1)
            InfraConfig.trace.data{i} = filter(b,a,HanningWindow(InfraConfig.trace.data{i}));
        end
    end
else
    for i = 1:NTrace
        if (numel(f_band) > 1)
            [b,a] = butter(order,[f_band(1) f_band(2)]/ceil(InfraConfig.trace.s_f{i}/2));
        else
            [b,a] = butter(order,f_band(1)/ceil(InfraConfig.trace.s_f{i}/2),'high');
        end
        InfraConfig.trace.data{i} = filter(b,a,HanningWindow(InfraConfig.trace.data{i}));
    end
end


pos = get(handles.axes1,'Position');
delete(handles.axes1);
handles.axes1 = axes('Parent',handles.figure1,'Position',pos);

% delete(handles.axes1);
% handles.axes1 = axes('Parent',handles.figure1,'Units','characters',...
%     'Position',[8.16667 2.33333 172.667 49.1667]);
%
InfraConfig.db.f_band = f_band;

PlotData(handles,NTrace,0)
guidata(hObject, handles)


function Menu_Filter_Undo_Callback(hObject, eventdata, handles)

global InfraConfig

try
    NTrace = numel(InfraConfig.trace.s_f);
catch
    errordlg('InfraMonitor2: Ensure that data are read in')
end

for i = 1:NTrace
    if (exist('DataOrig.mat','file') ~= 0)
        load('DataOrig.mat');
        InfraConfig.trace.data = data_orig;
    end
end

pos = get(handles.axes1,'Position');
delete(handles.axes1);
handles.axes1 = axes('Parent',handles.figure1,'Position',pos);

% delete(handles.axes1);
% handles.axes1 = axes('Parent',handles.figure1,'Units','characters',...
%     'Position',[8.16667 2.33333 172.667 49.1667]);

InfraConfig.db.f_band = [];
PlotData(handles,NTrace,0)
guidata(hObject, handles)


function Menu_Tools_Filter_Callback(hObject, eventdata, handles)


function Menu_Tools_Callback(hObject, eventdata, handles)


function Menu_Tools_Zoom_Callback(hObject, eventdata, handles)


function Menu_Zoom_On_Callback(hObject, eventdata, handles)
zoom on


function Menu_Zoom_Off_Callback(hObject, eventdata, handles)
zoom off


function Menu_Tools_Unselect_Callback(hObject, eventdata, handles)
global InfraConfig

InfraConfig.array.select = [];
try
    PlotData(handles,numel(InfraConfig.db.select),1);
catch
    return
end


function Menu_Tools_Plot_Callback(hObject, eventdata, handles)


function Menu_Plot_Spectra_Callback(hObject, eventdata, handles)

global InfraConfig

try
    traces = InfraConfig.trace.select;
catch
    errordlg('InfraMonitor2: Ensure that data are read in')
    return
end

if (numel(traces) == 0)
    errordlg('InfraMonitor2: Ensure that traces of interest are selected!')
    return
end

[x,y] = myginput(2,'crosshair');

try
    traces = InfraConfig.trace.select;
    for i = 1:numel(traces)
        figure
        [u,i1] = min(abs(x(1)-InfraConfig.trace.time{traces(i)}));
        [u,i2] = min(abs(x(2)-InfraConfig.trace.time{traces(i)}));
        Hs=spectrum.mtm(4);
        psd(Hs,InfraConfig.trace.data{traces(i)}(i1:i2),'Fs',InfraConfig.trace.s_f{traces(i)})
        title(InfraConfig.trace.name{traces(i)});
    end
catch
    errordlg('InfraMonitor2: Ensure that traces of interest are selected!')
end


function Menu_Plot_Spectrogram_Callback(hObject, eventdata, handles)

global InfraConfig

xtickFlag = 0;

try
    traces = InfraConfig.trace.select;
catch
    errordlg('InfraMonitor2: Ensure that data are read in')
    return
end

if (numel(traces) == 0)
    errordlg('InfraMonitor2: Ensure that traces of interest are selected!')
    return
end

[selection,ok] = listdlg('Name','Spectrogram',...
    'ListSize',[160 100],...
    'PromptString','nfft:',...
    'SelectionMode','single',...
    'ListString',{'64','128','256','512','1024','2048'});

if (ok == 0)
    return
end

nfft_choices = [64 128 256 512 1024 2048];
nfft = nfft_choices(selection);
nnf = 4096;
[x,y] = myginput(2,'crosshair');

for i = 1:numel(traces)
    figure
    subplot(2,1,1);
    [u,i1] = min(abs(x(1)-InfraConfig.trace.time{traces(i)}));
    [u,i2] = min(abs(x(2)-InfraConfig.trace.time{traces(i)}));
    %---
    %plot(InfraConfig.trace.data{traces(i)}(i1:i2)/max(abs(InfraConfig.trace.data{traces(i)}(i1:i2))),'k')
    plot(InfraConfig.trace.data{traces(i)}(i1:i2),'k')
    %---
    axis('tight')
    set(gca,'XTick',[])
    ylabel('Norm. Amplitude')
    title(InfraConfig.trace.name{traces(i)});
    subplot(2,1,2);

%     load DataOrig.mat
%     spectrogram((data_orig{traces(i)}(i1:i2)),nfft,InfraConfig.trace.s_f{traces(i)},nfft,50); view(90,-90);
    try
        load DataOrig.mat
        myspecgram((data_orig{traces(i)}(i1:i2)),nfft,InfraConfig.trace.s_f{traces(i)},nfft,50);
    catch
        try
            myspecgram((InfraConfig.trace.data{traces(i)}(i1:i2)),nfft,InfraConfig.trace.s_f{traces(i)},nfft,50);
        catch
            xtickFlag = 1;
            try
                spectrogram((data_orig{traces(i)}(i1:i2)),nfft,nfft/2,nfft,InfraConfig.trace.s_f{traces(i)}); view(90,-90);
            catch
                spectrogram((InfraConfig.trace.data{traces(i)}(i1:i2)),nfft,nfft/2,nfft,InfraConfig.trace.s_f{traces(i)}); view(90,-90);
            end
        end
    end
    XLabels = get(gca,'XTickLabel');
    stime = InfraConfig.trace.time{traces(i)}(i1);
    for j = 1:size(XLabels,1)
        XLabelsOut(j,:) = datestr(stime + str2double(XLabels(j,:))/86400,13);
    end
    if (xtickFlag == 0)
        set(gca,'XTickLabel',XLabelsOut)
    else
        set(gca,'YTickLabel',XLabelsOut)
    end
end

function Menu_Tools_GT_Callback(hObject, eventdata, handles)
global InfraConfig

GT = inputdlg({'Event Location','Origin Time','Group velocities'},...
    'Ground Truth',1,{'lat,lon','mm/dd/yyyy HH:MM:SS','0.28,0.34'});

if (numel(GT) == 0)
    return
end

try
    % *** Modified 02/10/10 ***
    try
        NoGT = numel(InfraConfig.gt.t0)
    catch
        NoGT = 0;
    end
    NoGT = NoGT + 1;

    InfraConfig.gt.loc0{NoGT} = list2array(GT{1});
    InfraConfig.gt.t0{NoGT} = datenum(GT{2},'mm/dd/yyyy HH:MM:SS');
    InfraConfig.gt.v_g{NoGT} = list2array(GT{3});
    % ***
catch
    errordlg('InfraMonitor2: Invalid format for ground truth')
    return
end

try
    PlotData(handles,numel(InfraConfig.db.select),1);
catch
    return
end

function Menu_Tools_Prefs_Callback(hObject, eventdata, handles)
h = IMPrefs;
waitfor(h)

function Menu_Tools_About_Callback(hObject, eventdata, handles)
h = About;
waitfor(h)


function Menu_Locate_View_Callback(hObject, eventdata, handles)


function Menu_Zoom_Magnify_Callback(hObject, eventdata, handles)

global InfraConfig

NTrace = numel(InfraConfig.trace.s_f);
mag = inputdlg('Magnification Factor','1');

pos = get(handles.axes1,'Position');
delete(handles.axes1);
handles.axes1 = axes('Parent',handles.figure1,'Position',pos);

% delete(handles.axes1);
% handles.axes1 = axes('Parent',handles.figure1,'Units','characters',...
%     'Position',[8.16667 2.33333 172.667 49.1667]);

InfraConfig.mag = str2double(mag);
PlotData(handles,NTrace,0)
guidata(hObject, handles)


% --------------------------------------------------------------------
function Menu_Locate_Run_Polygon_Callback(hObject, eventdata, handles)

[selection,ok] = listdlg('Name','Locator',...
    'ListSize',[160 50],...
    'PromptString','Apply Phase Association?',...
    'SelectionMode','single',...
    'ListString',{'yes','no'});

if (ok == 0)
    return
end

global InfraConfig

try
    grid_exist = numel(InfraConfig.grid);
catch
    errordlg('InfraMonitor2: Create or open grid file')
    return
end

if (selection == 1)
    Arrivals = FirstArrivals();
    PlotData(handles,numel(InfraConfig.db.select),1);
else
    try
        Test = numel(InfraConfig.array.select);
        if (Test == 0)
            errordlg('InfraMonitor2: Arrivals must be selected without phase association')
            return
        end
    catch
        errordlg('InfraMonitor2: Arrivals must be selected without phase association')
        return
    end
    Arrivals = AllFirstArrivals();
end

Locate(Arrivals)
msgbox('InfraMonitor2: Completed location processing')


% --------------------------------------------------------------------
function Menu_Locate_Run_BISL_Callback(hObject, eventdata, handles)

% [selection,ok] = listdlg('Name','Locator',...
%     'ListSize',[160 50],...
%     'PromptString','Apply Phase Association?',...
%     'SelectionMode','single',...
%     'ListString',{'yes','no'});
%
% if (ok == 0)
%     return
% end



min_arrays = inputdlg({'Min. # Arrays'},...
    'Association Parameter',1,{'3'});

if (numel(min_arrays) == 0)
    return
end

min_arrays = str2double(min_arrays);

global InfraConfig

try
    grid_exist = numel(InfraConfig.grid);
catch
    errordlg('InfraMonitor2: Create or open grid file')
    return
end

if (InfraConfig.grid.dmin ~= InfraConfig.grid.dmin)
    errdlg('Invalid grid: Grid spacing must be constant')
    return
end

global assoc toassoc InfraConfigBackup

InfraConfigBackup.array = InfraConfig.array;
InfraConfigBackup.grid = InfraConfig.grid;

NArrays = numel(InfraConfigBackup.array.arr);
narrays = (min_arrays:1:NArrays);

C = progress('init','InfraMonitor2: Performing associations');
for i = 1:numel(narrays)
    
    disp(numel(narrays))
    disp(i)
    
    C = progress(C,i/numel(narrays));

    DoAssociation(narrays(i),NArrays);

    if (toassoc == 0)
        break
    end
    
end

% save('GotToHere.mat')

InfraConfig.array = InfraConfigBackup.array;
InfraConfig.grid = InfraConfigBackup.grid;

k = 1; BISLMain2(k);

msgbox('InfraMonitor2: Completed location processing')

% --------------------------------------------------------------------
function Menu_Locate_View_Polygon_Callback(hObject, eventdata, handles)

global InfraConfig

[filename,pathname] = uigetfile('*.IMpoly','Select polygon file...');
if (filename == 0)
    return
end

poly = load([pathname filename],'-ascii');
polyids = unique(poly(:,1));

for i = 1:numel(polyids)
    choices{i} = num2str(polyids(i));
end

[selection,ok] = listdlg('Name','Location Polygons',...
    'ListSize',[160 100],...
    'PromptString','Select Polygon(s):',...
    'SelectionMode','multiple',...
    'ListString',choices);

if (ok == 0)
    return
end

polyids = polyids(selection);

figure
worldmap(InfraConfig.grid.glat,InfraConfig.grid.glon);
hold on

for i = 1:numel(InfraConfig.array.loc)
    plotm(InfraConfig.array.loc{i}(1),InfraConfig.array.loc{i}(2),'k^',...
        'MarkerFaceColor','y')
end

for i = 1:numel(polyids)
    u = find(poly(:,1) == polyids(i));
    plotm(poly(u,3),poly(u,4),'r')
end


% --------------------------------------------------------------------
function Menu_Locate_View_BISL_Callback(hObject, eventdata, handles)

global InfraConfig assoc

% Choices1 = find(cell2mat(InfraConfig.BISL.NOEVENTS) ~= 0)';
% choices1 = num2str(find(cell2mat(InfraConfig.BISL.NOEVENTS) ~= 0)');
% [RunNo,ok] = listdlg('Name','Successful runs',...
%     'ListSize',[160 100],...
%     'PromptString','Select a run number:',...
%     'SelectionMode','multiple',...
%     'ListString',choices1);
% if (ok == 0)
%     return
% end
%
% RunNo = Choices1(RunNo);

choices = num2str((1:numel(InfraConfig.BISL.CREDIBILITIES))');
[selection,ok] = listdlg('Name','Plot credibilities',...
    'ListSize',[160 100],...
    'PromptString','Select events(s):',...
    'SelectionMode','multiple',...
    'ListString',choices);
if (ok == 0)
    return
end

[selection2,ok2] = listdlg('Name','Backazimuths',...
    'ListSize',[160 50],...
    'PromptString','Plot backazimuths?',...
    'SelectionMode','single',...
    'ListString',{'yes','no'});

if (ok2 == 0)
    return
end

for p=selection
figure
m_proj('mercator','longitudes',[InfraConfig.grid.glon(1) InfraConfig.grid.glon(2)],...
    'latitudes',[InfraConfig.grid.glat(1) InfraConfig.grid.glat(2)]);
if (exist('coast.dat','file') ~= 0)
	load coast.dat; m_line(coast(:,1),coast(:,2),'Color','k'); m_grid; hold on
else
	m_coast; m_grid; hold on
end


LON = repmat(InfraConfig.BISL.lon,numel(InfraConfig.BISL.lat),1);
LAT = repmat(InfraConfig.BISL.lat',1,numel(InfraConfig.BISL.lon));

m_contour(LON,LAT,InfraConfig.BISL.CREDIBILITIES{p}{1},'LevelList',[0.75,0.90,0.95],'LineColor','black')
[cs,h] = m_contour(LON,LAT,InfraConfig.BISL.CREDIBILITIES{p}{1},'LevelList',[0.75,0.90,0.95]);
I1 = find(cs(1,:)==0.75);
I2 = find(cs(1,:)==0.9);
I3 = find(cs(1,:)==0.95);

try
    [lon,lat] = m_xy2ll(cs(1,I1+1:I2-1),cs(2,I1+1:I2-1));                      % 0.75
    lon = 111.1949*cosd(min(lat))*(lon-min(lon));
    lat = 111.1949*(lat-min(lat));
    disp(['0.75: ' num2str(polyarea(lon,lat)) ' km^2'])

    [lon,lat] = m_xy2ll(cs(1,I2+1:I3-1),cs(2,I2+1:I3-1));                      % 0.9
    lon = 111.1949*cosd(min(lat))*(lon-min(lon));
    lat = 111.1949*(lat-min(lat));
    disp(['0.9: ' num2str(polyarea(lon,lat)) ' km^2'])

    [lon,lat] = m_xy2ll(cs(1,I3+1:numel(cs(1,:))),cs(2,I3+1:numel(cs(1,:))));  % 0.95
    lon = 111.1949*cosd(min(lat))*(lon-min(lon));
    lat = 111.1949*(lat-min(lat));
    disp(['0.95: ' num2str(polyarea(lon,lat)) ' km^2'])
catch
    disp('')
end

uu = 0;
for q = 1:numel(InfraConfig.array.loc)
    [X,Y] = m_ll2xy(InfraConfig.array.loc{q}(2),InfraConfig.array.loc{q}(1));
    if (numel(find(assoc(p).arrays == q)) == 1)
        uu = uu + 1;
        % ***
        if (selection2 == 1)
            baz = assoc(p).baz(uu);
            Lon = InfraConfig.array.loc{q}(2); Lat = InfraConfig.array.loc{q}(1);

            for vv = 1:numel(InfraConfig.array.loc)
                dists(vv) = max([vdist(InfraConfig.array.loc{vv}(1),InfraConfig.array.loc{vv}(2),InfraConfig.grid.glat(1),InfraConfig.grid.glon(1)) ...
                    vdist(InfraConfig.array.loc{vv}(1),InfraConfig.array.loc{vv}(2),InfraConfig.grid.glat(1),InfraConfig.grid.glon(2)) ...
                    vdist(InfraConfig.array.loc{vv}(1),InfraConfig.array.loc{vv}(2),InfraConfig.grid.glat(2),InfraConfig.grid.glon(1)) ...
                    vdist(InfraConfig.array.loc{vv}(1),InfraConfig.array.loc{vv}(2),InfraConfig.grid.glat(2),InfraConfig.grid.glon(2))]);
            end
            max_dist = max(dists);

            [Lon2,Lat2,a21] = m_fdist(Lon,Lat,baz,max_dist);
            [dist,Lons,Lats] = m_lldist([Lon Lon2],[Lat Lat2],10);
            [Xt,Yt] = m_ll2xy(Lons,Lats);
            plot(Xt,Yt);
        end
        % ***
        plot(X,Y,'k^','MarkerFaceColor','y')
    else
        plot(X,Y,'k^','MarkerFaceColor','r')
    end
end

title(sprintf('Event %d: 75,90,95 Percent Credibility Contours',p))

try
    [X,Y] = m_ll2xy(InfraConfig.gt.loc0{1}(2),InfraConfig.gt.loc0{1}(1));
    plot(X,Y,'rp')
end

end


% --------------------------------------------------------------------
function pure_state_Callback(hObject, eventdata, handles)
% hObject    handle to pure_state (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global InfraConfig

data = zeros(numel(InfraConfig.trace.data{1}),numel(InfraConfig.trace.data));
NTraces = numel(InfraConfig.trace.data);
for i = 1:NTraces
    data(:,i) = InfraConfig.trace.data{i};
end

[data,P,S]=psf(data);
data = data(1:numel(InfraConfig.trace.data{1}),:);

InfraConfig.trace.data = [];
for i = 1:NTraces
    InfraConfig.trace.data{i} = data(:,i);
end

delete(handles.axes1);
handles.axes1 = axes('Parent',handles.figure1,'Units','characters',...
    'Position',[8.16667 2.33333 172.667 49.1667]);

PlotData(handles,NTraces,0)
guidata(hObject, handles)


% --- Executes on button press in SelectButton.
function SelectButton_Callback(hObject, eventdata, handles)
% hObject    handle to SelectButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global InfraConfig

% Obtaining filter band:
filtering = inputdlg({'InfraMonitor2: Enter Trace #(s):'},...
    'Filtering',1,{'1-4'});

traces = regexp(filtering{1},'-','split');
if (numel(traces) == 2)
    InfraConfig.trace.select = (str2double(traces{1}):1:str2double(traces{2}));
else
    InfraConfig.trace.select = str2double(traces{1});
end

pos = get(handles.axes1,'Position');
delete(handles.axes1);
handles.axes1 = axes('Parent',handles.figure1,'Position',pos);
% handles.axes1 = axes('Parent',handles.figure1,'Units','characters',...
%     'Position',[8.16667 2.33333 172.667 49.1667]);

try
    NTrace = numel(InfraConfig.trace.s_f);
catch
    errordlg('InfraMonitor2: Ensure that data are read in')
    return
end

PlotData(handles,NTrace,0)
guidata(hObject, handles)


% --------------------------------------------------------------------
function Plot_Array_Callback(hObject, eventdata, handles)

try
    global InfraConfig
    [x,y] = GetXY(InfraConfig);
    x = x*111.1949; y = y*111.1949;
    h = figure; h = plot(x,y,'.'); xlabel('km'); ylabel('km')
    sensors = strvcat(InfraConfig.trace.name{InfraConfig.trace.select});
    for i = 1:size(sensors,1)
        labels{i} = sensors(i,:);
    end
    customDataCursor(h,labels);
catch
    errordlg('Must first select all traces in an individual array',...
        'Selection Error');
end


% --- Executes on button press in ProcessingWindow.
function ProcessingWindow_Callback(hObject, eventdata, handles)
% hObject    handle to ProcessingWindow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global InfraConfig

InfraConfig.db.stimeNew = datenum(get(handles.stime,'String'),'mm/dd/yyyy HH:MM:SS');
InfraConfig.db.etimeNew = datenum(get(handles.etime,'String'),'mm/dd/yyyy HH:MM:SS');

guidata(hObject, handles)


function stime_Callback(hObject, eventdata, handles)
% hObject    handle to stime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of stime as text
%        str2double(get(hObject,'String')) returns contents of stime as a double


% --- Executes during object creation, after setting all properties.
function stime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to stime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function etime_Callback(hObject, eventdata, handles)
% hObject    handle to etime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etime as text
%        str2double(get(hObject,'String')) returns contents of etime as a double


% --- Executes during object creation, after setting all properties.
function etime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function Predict_Callback(hObject, eventdata, handles)
% hObject    handle to Predict (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function BouncePoints_Callback(hObject, eventdata, handles)
% hObject    handle to BouncePoints (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global InfraConfig

try
    loc = InfraConfig.gt.loc0{1};
    lat = loc(1); lon = loc(2);
catch
    errordlg('You must enter ground-truth first using the Tools>GT menu option');
    return
end

[metFile,pathname] = uigetfile('*.met','Select met file...');
if (metFile == 0)
    return
end

% Obtaining met file resolution (dz):
x = load(metFile,'-ascii');
dz = diff(x(:,1))*1000;
% if (numel(unique(dz)) > 1)
%     errordlg('The met file must have a uniform sampling rate!');
%     return
% end
dz = dz(1);

thetaNum = (1:5:85);
phiNum = (0:5:355);

try
    latLims = InfraConfig.grid.glat;
    lonLims = InfraConfig.grid.glon;
catch
    errordlg('You must generate a grid file using the Locate>Grid>New option');
    return
end

k = 0;
for i = 1:numel(latLims)
    for j = 1:numel(lonLims)
        k = k + 1;
%         d(k) = deg2km(distance(lat,lon,latLims(i),lonLims(j)));
        d(k) = m_idist(lon,lat,lonLims(j),latLims(i))/1000;
    end
end

maxRange = max(d);

BouncePointMap(lat,lon,metFile,dz,maxRange,thetaNum,phiNum,latLims,lonLims);


% --------------------------------------------------------------------
function Tau_P_Callback(hObject, eventdata, handles)
% hObject    handle to Tau_P (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

InfraTaup


% --- Executes on button press in processEverything.
function processEverything_Callback(hObject, eventdata, handles)
% hObject    handle to processEverything (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global InfraConfig

cb = get(handles.Menu_Detect_Run, 'Callback');

try
    uniqueArrays = unique(InfraConfig.trace.refsta);
    for i = 1:numel(uniqueArrays)
        I = strcmp(uniqueArrays{i},InfraConfig.trace.refsta);
        InfraConfig.trace.select = find(I == 1);
        cb(hObject,eventdata)
    end
catch
    errordlg('This feature requires the refsta column in the site file');
end


% --------------------------------------------------------------------
function File_Open_NNSA_Callback(hObject, eventdata, handles)
% hObject    handle to File_Open_NNSA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global InfraConfig

% Obtaining wfdisc file details:
[filename,pathname] = uigetfile('*.wfdisc','Select wfdisc file...');
if (filename == 0)
    return
end

% currentdir = [pwd '/'];
% if (strcmp(pathname,currentdir) == 0)
%     errordlg('Data is not in working directory - change working directory in Matlab')
%     return
% end

% Reading wfdisc and site files:
InfraConfig.db.dir = pathname;
InfraConfig.db.name = regexprep(filename,'.wfdisc','');
try
    InfraConfig.db.wfdisc = readWfdisc([pathname filename],'nnsa');
    InfraConfig.db.site = readSite([pathname InfraConfig.db.name '.site'],'nnsa');
catch
    errordlg('Cant read in wfdisc AND site files in nnsa format. Check both files exist; if they do, try using css3.0 format');
    return;
end

% Allowing user selection of traces and time window (Configuring
% InfraConfig.db structure):
h = Selectdb;
waitfor(h)
set(handles.stime,'String',datestr(InfraConfig.db.stime,'mm/dd/yyyy HH:MM:SS'));
set(handles.etime,'String',datestr(InfraConfig.db.etime,'mm/dd/yyyy HH:MM:SS'));

% Storing trace data as a structure (trace):
NTrace = numel(InfraConfig.db.select);
try
    Maketrace(NTrace);
catch
    if (get(handles.radioButton1,'Value') == 1)
        errordlg('Try changing the schema type to nnsa')
    else
        errordlg('Try changing the schema type to css3.0')
    end
    return
end

cleanTrace;
NTrace = numel(InfraConfig.trace.name);

% Adding refsta information if available:
try
    for i = 1:numel(InfraConfig.trace.name)
        strcmpNos = strcmp(InfraConfig.trace.name{i},InfraConfig.db.site{1});
        I = find(strcmpNos == 1);
        InfraConfig.trace.refsta{i} = InfraConfig.db.site{9}{I};
    end
end

try
    % Plotting waveform data:
    InfraConfig.trace.select = [];
    PlotData(handles,NTrace,0);
    guidata(hObject, handles);
end


% --------------------------------------------------------------------
function Menu_Detect_Run_All_Callback(hObject, eventdata, handles)
% hObject    handle to Menu_Detect_Run_All (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global InfraConfig

cb = get(handles.Menu_Detect_Run, 'Callback');

try
    uniqueArrays = unique(InfraConfig.trace.refsta);
    for i = 1:numel(uniqueArrays)
        I = strcmp(uniqueArrays{i},InfraConfig.trace.refsta);
        InfraConfig.trace.select = find(I == 1);
        cb(hObject,eventdata)
    end
catch
    errordlg('This feature requires the refsta column in the site file');
end
