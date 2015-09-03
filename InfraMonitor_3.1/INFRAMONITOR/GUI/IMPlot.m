function varargout = IMPlot(varargin)
% IMPLOT M-file for IMPlot.fig
%      IMPLOT, by itself, creates a new IMPLOT or raises the existing
%      singleton*.
%
%      H = IMPLOT returns the handle to a new IMPLOT or the handle to
%      the existing singleton*.
%
%      IMPLOT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in IMPLOT.M with the given input arguments.
%
%      IMPLOT('Property','Value',...) creates a new IMPLOT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before IMPlot_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to IMPlot_OpeningFcn via varargin.
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

% Edit the above text to modify the response to help IMPlot

% Last Modified by GUIDE v2.5 09-Aug-2012 12:23:31

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @IMPlot_OpeningFcn, ...
                   'gui_OutputFcn',  @IMPlot_OutputFcn, ...
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


% --- Executes just before IMPlot is made visible.
function IMPlot_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to IMPlot (see VARARGIN)

% Choose default command line output for IMPlot
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes IMPlot wait for user response (see UIRESUME)
% uiwait(handles.figure1);

global InfraConfig
InfraUser = open('InfraUser.mat');

arrivals = varargin{1};
Chans = varargin{2};
Selection = varargin{3};
j = varargin{4};

NPanels = 4 + numel(Selection);

% Plotting F-statistic:
ax(1) = subplot(NPanels,1,1,'Parent',handles.plotpanel);
plot(InfraConfig.detect.time{j},InfraConfig.detect.fstat{j},'k.');
datetick('x')
ylabel('F-statistic')
try
    title([datestr(InfraConfig.db.stime,2) ' (Processing parameters: Frequency-band=' num2str(InfraConfig.db.f_band(1)) '-' num2str(InfraConfig.db.f_band(2)) 'Hz, Time window=' num2str(InfraUser.twin) ...
        's, Overlap=' num2str(InfraUser.overlap) ', p-value=' num2str(InfraUser.p) ')'])
catch
    keyboard
end
Chans = list2cell(Chans);

% Plotting Correlation:
ax(2) = subplot(NPanels,1,2,'Parent',handles.plotpanel);
plot(InfraConfig.detect.time{j},InfraConfig.detect.corr{j},'k.');
datetick('x')
ylabel('Correlation')
ylim([0 1])

% Plotting Azimuth:
ax(3) = subplot(NPanels,1,3,'Parent',handles.plotpanel);
%---
try
    gc_azi = mean(InfraConfig.trace.gt_azi{1}(InfraConfig.trace.select));
    az_err = 5;
    rectangle('Position',[InfraConfig.detect.time{j}(1),gc_azi-az_err,InfraConfig.detect.time{j}(numel(InfraConfig.detect.time{j}))-InfraConfig.detect.time{j}(1),az_err],...
        'EdgeColor',[0.5 0.5 0.5],'FaceColor',[0.5 0.5 0.5])
end
hold on
plot(InfraConfig.detect.time{j},InfraConfig.detect.az{j},'k.');
hold off
%---

datetick('x')
ylabel('Azimuth')
ylim([0 360])

% Plotting Phase velocity:
ax(4) = subplot(NPanels,1,4,'Parent',handles.plotpanel);
plot(InfraConfig.detect.time{j},InfraConfig.detect.slofk{j},'k.');
datetick('x')
ylabel('Phase velocity')
ylim([0 0.5])

% Computing beam if necessary:
if (Selection > numel(InfraConfig.trace.select))

    % Extracting backazimuth and trace velocity (converting trace velocity
    % to s/deg.):
    beamparam = inputdlg({'Backazimuth','Trace velocity'},...
    'Beam Parameters',1,{'90','0.34'});
    try
        baz = str2double(beamparam{1});
        trace_v = str2double(beamparam{2});
        trace_v = (1/trace_v)*111.1949;
    catch
        return
    end

    % Generating data array (ensuring that each trace starts/ends at the
    % same time):
    tmp = InfraConfig.trace.data(InfraConfig.trace.select);
    tmp_t = InfraConfig.trace.time(InfraConfig.trace.select);
    for i = 1:numel(tmp_t)
        t_start(i) = tmp_t{i}(1);
        t_end(i) = tmp_t{i}(numel(tmp_t{i}));
    end
    t_start_max = max(t_start); t_end_min = min(t_end);
    for i = 1:numel(tmp_t)
        [U,V] = find(t_start_max < tmp_t{i} & tmp_t{i} < t_end_min);
        try
            data(:,i) = tmp{i}(V);
        catch
            try
                TMP = tmp{i}(V);
                TMP = TMP(1:size(data,1));
                data(:,i) = TMP;
            catch
                nMissing = size(data,1) - numel(TMP);
                data(:,i) = cat(1,TMP,zeros(nMissing,1));
            end
        end
        if (i == 1)
            time = tmp_t{i}(V);
        end
    end
    %---

    % Optionally applying the pure-state filter:
    %[data,P,S]=psf(data);

    % Extracting sampling frequency:
    s_f = InfraConfig.trace.s_f(InfraConfig.trace.select);
    s_f = s_f{1};

    % Computing relative coordinates of sensor elements at array:
    [X,Y] = GetXY(InfraConfig);

    % Computing the beam:
    beam = tdelay(data,s_f,X,Y,baz,trace_v);
    handles.user.beam = sum(beam')'./numel(InfraConfig.trace.select);
    handles.user.time = time;

end

% Plotting Waveform:
for I = 1:numel(Selection)

    ax(4+I) = subplot(NPanels,1,4+I,'Parent',handles.plotpanel);
    try
        for i = 1:numel(arrivals(:,1))
%             if (abs(arrivals(i,3)-272.2) < 5)
                rectangle('Position',[arrivals(i,1),-1,arrivals(i,2)-arrivals(i,1),2],...
                    'EdgeColor',[0.5 0.5 0.5],'FaceColor',[0.5 0.5 0.5])
%             end
        end
    end

    hold on

    try
        plot(InfraConfig.trace.time{InfraConfig.trace.select(Selection(I))},...
            InfraConfig.trace.data{InfraConfig.trace.select(Selection(I))}/...
            max(abs(InfraConfig.trace.data{InfraConfig.trace.select(Selection(I))})),'k');
        InfraConfig.trace.F1 = max(abs(InfraConfig.trace.data{InfraConfig.trace.select(Selection(I))}));
    catch
        plot(handles.user.time,...
            handles.user.beam/...
            max(abs(handles.user.beam)),'k');
    end

    try
        for GtNo = 1:numel(InfraConfig.gt.t0)
            plot([InfraConfig.trace.gt1{GtNo}(InfraConfig.trace.select(I)) InfraConfig.trace.gt1{GtNo}(InfraConfig.trace.select(I))],[-1 1],'g')
            plot([InfraConfig.trace.gt2{GtNo}(InfraConfig.trace.select(I)) InfraConfig.trace.gt2{GtNo}(InfraConfig.trace.select(I))],[-1 1],'g')
            plot([InfraConfig.trace.GTTime_Pn{GtNo}(InfraConfig.trace.select(I)) InfraConfig.trace.GTTime_Pn{GtNo}(InfraConfig.trace.select(I))],[-1 1],'r')
            plot([InfraConfig.trace.GTTime_Pg{GtNo}(InfraConfig.trace.select(I)) InfraConfig.trace.GTTime_Pg{GtNo}(InfraConfig.trace.select(I))],[-1 1],'b')
            plot([InfraConfig.trace.GTTime_Lg{GtNo}(InfraConfig.trace.select(I)) InfraConfig.trace.GTTime_Lg{GtNo}(InfraConfig.trace.select(I))],[-1 1],'m')
        end
    end
    hold off
    datetick('x')

end

% Setting time axis properties:
linkaxes(ax,'x');
try
    xlim([InfraConfig.db.stimeNew InfraConfig.db.etimeNew])
catch
    xlim([min(InfraConfig.trace.time{InfraConfig.trace.select(1)}) ...
        max(InfraConfig.trace.time{InfraConfig.trace.select(1)})])
end

axis 'auto y'

handles.user.arrivals = arrivals; handles.user.Chans = Chans; handles.user.j = j;
handles.user.Selection = Selection;
handles.user.NPanels = NPanels;
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = IMPlot_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function Picker_Callback(hObject, eventdata, handles)

warning off all

global InfraConfig
InfraUser = open('InfraUser.mat');
InfraUser.slow = linspace(-400,400,80);

[x,y] = myginput(1,'crosshair');

j = handles.user.j;

[u,i] = min(abs(x-InfraConfig.detect.time{j}));

t = InfraConfig.detect.time{j}(i);

try
    disp('Time after t0')
    disp((t - InfraConfig.gt.t0{1})*86400)
catch
    disp('')
end

t_start = t - (InfraUser.twin/2)/86400;
az = InfraConfig.detect.az{j}(i);
v_p = InfraConfig.detect.slofk{j}(i);
corr = InfraConfig.detect.corr{j}(i);
fstat = InfraConfig.detect.fstat{j}(i);

trc = InfraConfig.detect.trc{j};

time = InfraConfig.trace.time{trc(1)};
s_f = InfraConfig.trace.s_f{trc(1)};

for i = 1:numel(trc)
    Ndp(i) = numel(InfraConfig.trace.data{trc(i)});
end

for i = 1:numel(trc)
    data(:,i) = InfraConfig.trace.data{trc(i)}(1:min(Ndp));
end

[U,I] = min(abs(t_start-time));

time = time(I:I+(InfraUser.twin*s_f));
data = data(I:I+(InfraUser.twin*s_f),:);
[X,Y] = GetXY(InfraConfig);

try
    gt_azi = mean(InfraConfig.trace.gt_azi{1}(InfraConfig.detect.trc{j}));
    del_az = az-gt_azi;
    d = vdist(InfraConfig.array.loc{j}(1),InfraConfig.array.loc{j}(2),InfraConfig.gt.loc0{1}(1),InfraConfig.gt.loc0{1}(2))/1000;
    v_g = d/((t - InfraConfig.gt.t0{1})*86400);
end

[FK,Fstat1] = fk(data,s_f,X,Y,InfraUser.slow,InfraConfig.db.f_band');
plotfk(FK,InfraUser.slow)
set(handles.time,'String',datestr(t,13));
set(handles.azimuth,'String',num2str(az));
set(handles.correlation,'String',num2str(corr));

disp(['Trace velocity: ' num2str(v_p)])
try
    disp(['Group velocity: ' num2str(v_g)])
    disp(['Azimuth deviation: ' num2str(del_az)])
end

function time_Callback(hObject, eventdata, handles)


function time_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function azimuth_Callback(hObject, eventdata, handles)


function azimuth_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function correlation_Callback(hObject, eventdata, handles)


function correlation_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function CloseButton_Callback(hObject, eventdata, handles)

global InfraConfig

% 10/18/11, Changed fname for name in the following two lines:
if (numel(InfraConfig.trace.name) ~= numel(InfraConfig.trace.data))
    NTraces = numel(InfraConfig.trace.name);
    InfraConfig.trace.data = InfraConfig.trace.data(1:NTraces);
    InfraConfig.trace.select = InfraConfig.trace.select(1:NTraces);
    InfraConfig.trace.time = InfraConfig.trace.time(1:NTraces);
end

delete(handles.figure1);


function pushbutton3_Callback(hObject, eventdata, handles)

zoom xon

% global InfraConfig
%
% [xax,y] = myginput(2,'crosshair');
%
% % Get maximum amplitude in time window:
% try
%     I = handles.user.Selection(1);
%     time = InfraConfig.trace.time{InfraConfig.trace.select(handles.user.Selection(I))};
%     II = find(xax(1) <= time & time <= xax(2));
%     maxInTimeWindow = max(abs(InfraConfig.trace.data{InfraConfig.trace.select(handles.user.Selection(I))}(II)));
%     totalMax = max(abs(InfraConfig.trace.data{InfraConfig.trace.select(handles.user.Selection(I))}));
% catch
%     time = handles.user.time;
%     II = find(xax(1) <= time & time <= xax(2));
%     maxInTimeWindow = max(abs(handles.user.beam(II)));
%     totalMax = max(abs(handles.user.beam));
% end
%
% xlim([xax(1) xax(2)]);
% ylim([-maxInTimeWindow/totalMax maxInTimeWindow/totalMax]);
% XTickpts = linspace(xax(1),xax(2),handles.user.NPanels);
% for i = 1:handles.user.NPanels
%     subplot(handles.user.NPanels,1,i)
%     set(gca,'XTick',XTickpts)
%     set(gca,'XTickLabel',datestr(XTickpts,13))
% end
%
% a = get(gca,'Children');
% for i = 2:numel(a)
%     Position = get(a(i),'Position');
%     Position(2) = -maxInTimeWindow/totalMax;
%     Position(4) = (maxInTimeWindow/totalMax)*2;
%     set(a(i),'Position',Position);
% end
%
%

function pushbutton4_Callback(hObject, eventdata, handles)

global InfraConfig

trc = InfraConfig.trace.select;

t_p = InfraConfig.trace.time{trc(1)};
xlim([t_p(1) t_p(numel(t_p))]);
ylim([-1 1]);
XTickpts = linspace(t_p(1),t_p(numel(t_p)),5);
for i = 1:handles.user.NPanels
    subplot(handles.user.NPanels,1,i)
    set(gca,'XTick',XTickpts)
    set(gca,'XTickLabel',datestr(XTickpts,13))
end

a = get(gca,'Children');
for i = 2:numel(a)
    try
        Position = get(a(i),'Position');
        Position(2) = -1;
        Position(4) = 2;
        set(a(i),'Position',Position);
    end
end

function Menu_File_Print_Callback(hObject, eventdata, handles)
set(gcf,'PaperPositionMode','auto')
printpreview


function Untitled_1_Callback(hObject, eventdata, handles)


function Menu_File_Close_Callback(hObject, eventdata, handles)
delete(handles.figure1)


% --------------------------------------------------------------------
function Menu_File_Save_Callback(hObject, eventdata, handles)
% hObject    handle to Menu_File_Save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[filename,pathname] = uiputfile('trace','Save ASCII files');
if (filename == 0)
    return
end

SaveASCII([pathname filename])
msgbox('InfraMonitor2: Saved ASCII files')


% --- Executes on button press in Magnification.
function Magnification_Callback(hObject, eventdata, handles)
% hObject    handle to Magnification (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global InfraConfig

XLim_o = get(gca,'XLim');
XTick_o = get(gca,'XTick');
XTickLabel_o = get(gca,'XTickLabel');

magnification = str2double(inputdlg('Enter magnification factor:','Magnification'));

InfraConfig.trace.F2 = magnification;

for I = 1:numel(handles.user.Selection)

    ax(4+I) = subplot(handles.user.NPanels,1,4+I,'Parent',handles.plotpanel);
%     plot(InfraConfig.trace.time{InfraConfig.trace.select(handles.user.Selection(I))},...
%         InfraConfig.trace.data{InfraConfig.trace.select(handles.user.Selection(I))}/...
%         max(abs(InfraConfig.trace.data{InfraConfig.trace.select(handles.user.Selection(I))}))*magnification,'k');
%     hold on
    try
        for i = 1:numel(handles.user.arrivals(:,1))
            rectangle('Position',[handles.user.arrivals(i,1),-1,handles.user.arrivals(i,2)-handles.user.arrivals(i,1),2],...
                'EdgeColor',[0.5 0.5 0.5],'FaceColor',[0.5 0.5 0.5])
        end
    end
    hold on
    try
        plot(InfraConfig.trace.time{InfraConfig.trace.select(handles.user.Selection(I))},...
            InfraConfig.trace.data{InfraConfig.trace.select(handles.user.Selection(I))}/...
            max(abs(InfraConfig.trace.data{InfraConfig.trace.select(handles.user.Selection(I))}))*magnification,'k');
    catch
        plot(handles.user.time,...
            handles.user.beam/...
            max(abs(handles.user.beam))*magnification,'k');
    end
    try
        plot([InfraConfig.trace.gt1(InfraConfig.trace.select(I)) InfraConfig.trace.gt1(InfraConfig.trace.select(I))],[-1 1],'g')
        plot([InfraConfig.trace.gt2(InfraConfig.trace.select(I)) InfraConfig.trace.gt2(InfraConfig.trace.select(I))],[-1 1],'g')
        plot([InfraConfig.trace.GTTime_Pn(InfraConfig.trace.select(I)) InfraConfig.trace.GTTime_Pn(InfraConfig.trace.select(I))],[-1 1],'r')
        plot([InfraConfig.trace.GTTime_Pg(InfraConfig.trace.select(I)) InfraConfig.trace.GTTime_Pg(InfraConfig.trace.select(I))],[-1 1],'b')
        plot([InfraConfig.trace.GTTime_Lg(InfraConfig.trace.select(I)) InfraConfig.trace.GTTime_Lg(InfraConfig.trace.select(I))],[-1 1],'m')
    end
    hold off
    datetick('x')
    ylim([-1 1])
end

set(gca,'XLim',XLim_o);
set(gca,'XTick',XTick_o);
set(gca,'XTickLabel',XTickLabel_o);


% --- Executes on button press in yzoom.
function yzoom_Callback(hObject, eventdata, handles)
% hObject    handle to yzoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

yzoom_limits = get(handles.yzoom_param,'string');
yzoom_limits = list2array(yzoom_limits);

handles.user.NPanels
subplot(handles.user.NPanels,1,3)
ylim([yzoom_limits(1) yzoom_limits(2)])


function yzoom_param_Callback(hObject, eventdata, handles)
% hObject    handle to yzoom_param (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yzoom_param as text
%        str2double(get(hObject,'String')) returns contents of yzoom_param as a double


% --- Executes during object creation, after setting all properties.
function yzoom_param_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yzoom_param (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in AmpButton.
function AmpButton_Callback(hObject, eventdata, handles)
% hObject    handle to AmpButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global InfraConfig

ConversionFactor = str2double(get(handles.ConvFact,'String'));

XLim_o = get(gca,'XLim');
XTick_o = get(gca,'XTick');
XTickLabel_o = get(gca,'XTickLabel');

try
    Traces = InfraConfig.trace.select(handles.user.Selection);

    for i = 1:numel(Traces)
        figure
        plot(InfraConfig.trace.time{Traces(i)},InfraConfig.trace.data{Traces(i)})
        set(gca,'XLim',XLim_o);
        set(gca,'XTick',XTick_o);
        set(gca,'XTickLabel',XTickLabel_o);

        [xax,y] = myginput(3,'crosshair');
        close

        try
            Amp(i) = (max(y) - min(y))*ConversionFactor;
            T(i) = (xax(3) - xax(1))*86400;
        catch
            continue
        end
    end
catch
    figure
    plot(handles.user.time,handles.user.beam)
    set(gca,'XLim',XLim_o);
    set(gca,'XTick',XTick_o);
    set(gca,'XTickLabel',XTickLabel_o);

    [xax,y] = myginput(3,'crosshair');
    close

    Amp = (max(y) - min(y))*ConversionFactor;
    T = (xax(3) - xax(1))*86400;

end

Amp = Amp(find(Amp ~= 0));
T = T(find(T~=0));

set(handles.AmpEdit,'String',num2str(mean(Amp)));
set(handles.PeriodEdit,'String',num2str(mean(T)));

function AmpEdit_Callback(hObject, eventdata, handles)
% hObject    handle to AmpEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of AmpEdit as text
%        str2double(get(hObject,'String')) returns contents of AmpEdit as a double


% --- Executes during object creation, after setting all properties.
function AmpEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AmpEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PeriodEdit_Callback(hObject, eventdata, handles)
% hObject    handle to PeriodEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PeriodEdit as text
%        str2double(get(hObject,'String')) returns contents of PeriodEdit as a double


% --- Executes during object creation, after setting all properties.
function PeriodEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PeriodEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ConvFact_Callback(hObject, eventdata, handles)
% hObject    handle to ConvFact (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ConvFact as text
%        str2double(get(hObject,'String')) returns contents of ConvFact as a double


% --- Executes during object creation, after setting all properties.
function ConvFact_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ConvFact (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in RMSButton.
function RMSButton_Callback(hObject, eventdata, handles)
% hObject    handle to RMSButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global InfraConfig
global arrays;

[xax,y] = myginput(2,'crosshair');



% *************************************************************************
% *** TEMPORARY CODE SECTION FOR GENERATING TESTING DATA FOR MAURICE
% CHARBIT ***

load DataOrig.mat
TimeData = InfraConfig.trace.time(InfraConfig.trace.select);
arrayNames = InfraConfig.trace.name(InfraConfig.trace.select);
AmpData = data_orig(InfraConfig.trace.select);
loc = GetArrayLoc2(InfraConfig);
lats = loc(:,1);
lons = loc(:,2);
I = numel(arrays);

for i = 1:numel(TimeData)
    I = I + 1;
    u = find(xax(1) < TimeData{i} & TimeData{i} < xax(2));
    x = AmpData{i}(u);
    t = TimeData{i}(u);
    arrays(I).data = x';
    arrays(I).time = t;
    arrays(I).name = arrayNames{i};
    arrays(I).lat = lats(i);
    arrays(I).lon = lons(i);
end

return
% *************************************************************************





TimeData = InfraConfig.trace.time(InfraConfig.trace.select);
AmpData = InfraConfig.trace.data(InfraConfig.trace.select);

ConversionFactor = str2double(get(handles.ConvFact,'String'));

for i = 1:numel(TimeData)

    u = find(xax(1) < TimeData{i} & TimeData{i} < xax(2));

    A = AmpData{i}(u)*ConversionFactor;

    RMS_Noise(i) = sqrt(sum(A.^2)/numel(A));

end

set(handles.AmpEdit,'String',num2str(mean(RMS_Noise)));


% --- Executes on button press in RMSSignalButton.
function RMSSignalButton_Callback(hObject, eventdata, handles)
% hObject    handle to RMSSignalButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global InfraConfig

[xax,y] = myginput(1,'crosshair');

TimeData = InfraConfig.trace.time(InfraConfig.trace.select);
AmpData = InfraConfig.trace.data(InfraConfig.trace.select);

ConversionFactor = str2double(get(handles.ConvFact,'String'));

for i = 1:numel(TimeData)

    u = find(xax-(5/86400) < TimeData{i} & TimeData{i} < xax+(5/86400));

    A = AmpData{i}(u)*ConversionFactor;

    RMS_Signal(i) = sqrt(sum(A.^2)/numel(A));

end

set(handles.AmpEdit,'String',num2str(mean(RMS_Signal)));


% --------------------------------------------------------------------
function Menu_File_Write_Callback(hObject, eventdata, handles)
% hObject    handle to Menu_File_Write (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global InfraConfig

%filename = inputdlg('Enter file name');

% open('WMU.fig')
% h = gcf;
% line = findall(h, 'Type', 'Line');
% x = get(line(1),'xdata');
% y = get(line(1),'ydata');
% figure; subplot(2,1,1); plot(x,y)

event.beam = handles.user.beam;
event.time = handles.user.time';
event.loc0 = InfraConfig.gt.loc0;
event.t0 = InfraConfig.gt.t0;

time = (event.time - event.t0{1})*86400;
data = event.beam;
figure; plot(time,event.beam)

keyboard

save('WMU.mat','time','data')

%save(filename{1},'event')


% --- Executes on button press in yZoom.
function yZoom_Callback(hObject, eventdata, handles)
% hObject    handle to yZoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

zoom yon

% zoom yon
%
% a = get(gca,'Children');
% for i = 2:numel(a)
%     y = get(get(a(i),'Parent'),'YLim')
%     Position = get(a(i),'Position');
%     Position(2) = y(1);
%     Position(4) = y(1) + (y(2)-y(1));
%     set(a(i),'Position',Position);
% end


% --- Executes on button press in averageButton.
function averageButton_Callback(hObject, eventdata, handles)
% hObject    handle to averageButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global InfraConfig

[x,y] = myginput(2,'crosshair');





j = handles.user.j;

i = find(x(1) <= InfraConfig.detect.time{j} & InfraConfig.detect.time{j} <= x(2))

t = InfraConfig.detect.time{j}(i);
az = InfraConfig.detect.az{j}(i);
v_p = InfraConfig.detect.slofk{j}(i);
corr = InfraConfig.detect.corr{j}(i);
fstat = InfraConfig.detect.fstat{j}(i);

disp(mean(az))
disp(mean(v_p))
