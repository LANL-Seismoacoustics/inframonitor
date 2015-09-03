function varargout = RunInfraMonitor3(varargin)
% RUNINFRAMONITOR3 MATLAB code for RunInfraMonitor3.fig
%      RUNINFRAMONITOR3, by itself, creates a new RUNINFRAMONITOR3 or raises the existing
%      singleton*.
%
%      H = RUNINFRAMONITOR3 returns the handle to a new RUNINFRAMONITOR3 or the handle to
%      the existing singleton*.
%
%      RUNINFRAMONITOR3('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in RUNINFRAMONITOR3.M with the given input arguments.
%
%      RUNINFRAMONITOR3('Property','Value',...) creates a new RUNINFRAMONITOR3 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before RunInfraMonitor3_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to RunInfraMonitor3_OpeningFcn via varargin.
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

% Edit the above text to modify the response to help RunInfraMonitor3

% Last Modified by GUIDE v2.5 09-May-2011 09:17:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @RunInfraMonitor3_OpeningFcn, ...
                   'gui_OutputFcn',  @RunInfraMonitor3_OutputFcn, ...
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


% --- Executes just before RunInfraMonitor3 is made visible.
function RunInfraMonitor3_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to RunInfraMonitor3 (see VARARGIN)

% Choose default command line output for RunInfraMonitor3
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes RunInfraMonitor3 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = RunInfraMonitor3_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function wfdisc_filelist_Callback(hObject, eventdata, handles)
% hObject    handle to wfdisc_filelist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of wfdisc_filelist as text
%        str2double(get(hObject,'String')) returns contents of wfdisc_filelist as a double


% --- Executes during object creation, after setting all properties.
function wfdisc_filelist_CreateFcn(hObject, eventdata, handles)
% hObject    handle to wfdisc_filelist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function elements_Callback(hObject, eventdata, handles)
% hObject    handle to elements (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of elements as text
%        str2double(get(hObject,'String')) returns contents of elements as a double


% --- Executes during object creation, after setting all properties.
function elements_CreateFcn(hObject, eventdata, handles)
% hObject    handle to elements (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function frequency_Callback(hObject, eventdata, handles)
% hObject    handle to frequency (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of frequency as text
%        str2double(get(hObject,'String')) returns contents of frequency as a double


% --- Executes during object creation, after setting all properties.
function frequency_CreateFcn(hObject, eventdata, handles)
% hObject    handle to frequency (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function numpoles_Callback(hObject, eventdata, handles)
% hObject    handle to numpoles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of numpoles as text
%        str2double(get(hObject,'String')) returns contents of numpoles as a double


% --- Executes during object creation, after setting all properties.
function numpoles_CreateFcn(hObject, eventdata, handles)
% hObject    handle to numpoles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function min_arrays_Callback(hObject, eventdata, handles)
% hObject    handle to min_arrays (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of min_arrays as text
%        str2double(get(hObject,'String')) returns contents of min_arrays as a double


% --- Executes during object creation, after setting all properties.
function min_arrays_CreateFcn(hObject, eventdata, handles)
% hObject    handle to min_arrays (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in go.
function go_Callback(hObject, eventdata, handles)
% hObject    handle to go (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% ConversionFactor = str2double(get(handles.ConvFact,'String'));

global BigRun

wfdisc_file_list = get(handles.wfdisc_filelist,'String');

elements = get(handles.elements,'String');
elements = regexp(elements,';','split');

f_band = get(handles.frequency,'String');
f_band = regexp(f_band,',','split');
f_band = str2double(f_band);

npoles = get(handles.numpoles,'String');
npoles = str2double(npoles);

min_arrays = get(handles.min_arrays,'String');
min_arrays = str2double(min_arrays);

f = fopen(wfdisc_file_list);

rootFolder = pwd;
i = 0;
wfdisc_file = fgetl(f);
while ischar(wfdisc_file)

    i = i + 1;

    % Extracting wfdisc file and directory:
    splitPt = regexp(wfdisc_file,'/'); splitPt = splitPt(numel(splitPt));
    wfdisc_fileName = wfdisc_file(splitPt+1:numel(wfdisc_file));
    wfdisc_dir = wfdisc_file(1:splitPt);
    BigRun.dir{i} = wfdisc_dir;

    % Running InfraMonitor2b in sub-directory:
    cd(wfdisc_dir);
    db_name = regexp(wfdisc_fileName,'\.','Split');
    try
        copyfile([rootFolder '/' db_name{1} '.mat'],['./' db_name{1} '.mat']);
    end
    InfraMonitor2b(wfdisc_fileName,elements,f_band,npoles,min_arrays)
    cd(rootFolder);
    wfdisc_file = fgetl(f);

end

fclose(f);

dbname = regexp(wfdisc_fileName,'\.','split');
BigRun.dbname = dbname(1);

msgbox('InfraMonitor2: Completed batch processing')

% --- Executes on button press in Close.
function Close_Callback(hObject, eventdata, handles)
% hObject    handle to Close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear global BigRun
delete(handles.figure1)
%delete('DataOrig.mat')


% --- Executes on button press in ViewDetections.
function ViewDetections_Callback(hObject, eventdata, handles)
% hObject    handle to ViewDetections (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

try
    global BigRun

    system('touch all.detect');

    for i = 1:numel(BigRun.dir)
        system(['cat ' BigRun.dir{i} BigRun.dbname{1} '.detect >> ./all.detect']);
    end

    PlotDetect('all.detect')

    system('rm all.detect');
catch
    errordlg('Ensure that you have run IM2, or retrieved an old run, first in this GUI session!');
end

% --- Executes on button press in ViewLocations.
function ViewLocations_Callback(hObject, eventdata, handles)
% hObject    handle to ViewLocations (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


try
    global BigRun

    % Creating combined IMpoly file:
    f = fopen('all.IMpoly','w');
    j = 0;
    for i = 1:numel(BigRun.dir)
        try
            x = load([BigRun.dir{i} BigRun.dbname{1} '.IMpoly'],'-ascii');
            x(:,1) = x(:,1) + j;
        catch
            continue
        end
        for k = 1:size(x,1)
            fprintf(f,'%9d %9d %11.6f %11.6f\n',x(k,1),x(k,2),x(k,3),x(k,4));
        end
        j = x(numel(x(:,1)),1);
        clear x
    end
    fclose(f);

    % Creating combined IMassoc file:
    f = fopen('all.IMassoc','w');
    j = 0;
    for i = 1:numel(BigRun.dir)
        clear assocData
        try
            f2 = fopen([BigRun.dir{i} BigRun.dbname{1} '.IMassoc']);
            line = fgetl(f2);
            k = 0;
            while ischar(line)
                k = k + 1;
                splitline = regexp(line,'\S*','match');
                assocData(k,1) = datenum([splitline{1} ' ' splitline{2}],'yyyy-mm-dd HH:MM:SS');
                assocData(k,2) = str2double(splitline{3}); assocData(k,3) = str2double(splitline{4});
                line = fgetl(f2);
            end
            fclose(f2);
        catch
            continue
        end
        try
            assocData(:,2) = assocData(:,2) + j;
            for k = 1:numel(assocData(:,2))
                fprintf(f,'%s %8d %8d\n',datestr(assocData(k,1),'yyyy-mm-dd HH:MM:SS'),assocData(k,2),assocData(k,3));
            end
            j = assocData(numel(assocData(:,2)),2);
        end
    end
    fclose(f);

    % Getting elements to plot:
    elements = get(handles.elements,'String');
    elements = regexp(elements,';','split');

    % Reading site file:
    f = fopen([BigRun.dir{1} BigRun.dbname{1} '.site']);
    line = fgetl(f);
    while ischar(line)
        i = i + 1;
        splitline = regexp(line,'\S*','match');
        element{i} = splitline{1};
        lat(i) = str2double(splitline{4});
        lon(i) = str2double(splitline{5});
        line = fgetl(f);
    end

    % Extracting relevant lat, lon points:
    for i = 1:numel(elements)
        elementsSplit = regexp(elements{i},',','Split');
        for j = 1:numel(element)
            if (strcmp(elementsSplit{1},element{j}) == 1)
                Lat(i) = lat(j); Lon(i) = lon(j);
            end
        end
    end

    % Obtaining grid boundaries:
    grid = load([BigRun.dir{1} BigRun.dbname{1} '.mat']);

    % Obtaining remaining necessary parameters:
    userInput = inputdlg({'InfraMonitor2: Enter maximum area','Enter minimum # arrays'},...
        'Locations',1,{'10000','4'});
    try
        minArea = str2double(userInput{1});
        minArrays = str2double(userInput{2});
    catch
        return
    end

    PlotPolyfile('all.IMpoly',[grid.grid.glat grid.grid.glon],Lat,Lon,minArea,minArrays)
catch
    errordlg('Ensure that you have run IM2, or retrieved an old run, first in this GUI session!');
end


% --- Executes on button press in load.
function load_Callback(hObject, eventdata, handles)
% hObject    handle to load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global BigRun

wfdisc_file_list = get(handles.wfdisc_filelist,'String');

f = fopen(wfdisc_file_list);

i = 0;
wfdisc_file = fgetl(f);
while ischar(wfdisc_file)

    i = i + 1;

    % Extracting wfdisc file and directory:
    splitPt = regexp(wfdisc_file,'/'); splitPt = splitPt(numel(splitPt));
    wfdisc_fileName = wfdisc_file(splitPt+1:numel(wfdisc_file));
    wfdisc_dir = wfdisc_file(1:splitPt);
    BigRun.dir{i} = wfdisc_dir;

    wfdisc_file = fgetl(f);

end

fclose(f);

dbname = regexp(wfdisc_fileName,'\.','split');
BigRun.dbname = dbname(1);

msgbox('InfraMonitor2: Retrieved previous processing results.')


% --- Executes on button press in makeGrid.
function makeGrid_Callback(hObject, eventdata, handles)
% hObject    handle to makeGrid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global InfraConfig

wfdisc_file_list = get(handles.wfdisc_filelist,'String');

% Getting elements to plot:
elements = get(handles.elements,'String');
elements = regexp(elements,';','split');

f = fopen(wfdisc_file_list);

wfdisc_file = fgetl(f);

splitPt = regexp(wfdisc_file,'/'); splitPt = splitPt(numel(splitPt));
wfdisc_fileName = wfdisc_file(splitPt+1:numel(wfdisc_file));
wfdisc_dir = wfdisc_file(1:splitPt);

fclose(f);

dbname = regexp(wfdisc_fileName,'\.','split');
dbname = dbname(1);

% Reading site file:
f = fopen([wfdisc_dir dbname{1} '.site']);
line = fgetl(f);
i = 0;
while ischar(line)
    i = i + 1;
    splitline = regexp(line,'\S*','match');
    element{i} = splitline{1};
    lat(i) = str2double(splitline{4});
    lon(i) = str2double(splitline{5});
    line = fgetl(f);
end

% Extracting relevant lat, lon points:
for i = 1:numel(elements)
    elementsSplit = regexp(elements{i},',','Split');
    for j = 1:numel(element)
        if (strcmp(elementsSplit{1},element{j}) == 1)
            Lat(i) = lat(j); Lon(i) = lon(j);
            InfraConfig.array.loc{i} = [Lat(i) Lon(i)];
        end
    end
end

NewGrid()
