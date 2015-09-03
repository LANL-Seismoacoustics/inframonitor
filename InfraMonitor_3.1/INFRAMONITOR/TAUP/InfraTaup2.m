function varargout = InfraTaup(varargin)
% INFRATAUP MATLAB code for InfraTaup.fig
%      INFRATAUP, by itself, creates a new INFRATAUP or raises the existing
%      singleton*.
%
%      H = INFRATAUP returns the handle to a new INFRATAUP or the handle to
%      the existing singleton*.
%
%      INFRATAUP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INFRATAUP.M with the given input arguments.
%
%      INFRATAUP('Property','Value',...) creates a new INFRATAUP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before InfraTaup_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to InfraTaup_OpeningFcn via varargin.
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

% Edit the above text to modify the response to help InfraTaup

% Last Modified by GUIDE v2.5 28-Mar-2011 13:08:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @InfraTaup_OpeningFcn, ...
                   'gui_OutputFcn',  @InfraTaup_OutputFcn, ...
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


% --- Executes just before InfraTaup is made visible.
function InfraTaup_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to InfraTaup (see VARARGIN)

% Choose default command line output for InfraTaup
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes InfraTaup wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = InfraTaup_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function sourceLocation_Callback(hObject, eventdata, handles)
% hObject    handle to sourceLocation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of sourceLocation as text
%        str2double(get(hObject,'String')) returns contents of sourceLocation as a double


% --- Executes during object creation, after setting all properties.
function sourceLocation_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sourceLocation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function metFile_Callback(hObject, eventdata, handles)
% hObject    handle to metFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of metFile as text
%        str2double(get(hObject,'String')) returns contents of metFile as a double


% --- Executes during object creation, after setting all properties.
function metFile_CreateFcn(hObject, eventdata, handles)
% hObject    handle to metFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function azimuth_Callback(hObject, eventdata, handles)
% hObject    handle to azimuth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of azimuth as text
%        str2double(get(hObject,'String')) returns contents of azimuth as a double


% --- Executes during object creation, after setting all properties.
function azimuth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to azimuth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function launchAngle_Callback(hObject, eventdata, handles)
% hObject    handle to launchAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of launchAngle as text
%        str2double(get(hObject,'String')) returns contents of launchAngle as a double


% --- Executes during object creation, after setting all properties.
function launchAngle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to launchAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function File_Callback(hObject, eventdata, handles)
% hObject    handle to File (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function File_Close_Callback(hObject, eventdata, handles)
% hObject    handle to File_Close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
delete(handles.figure1);


% --- Executes on button press in computeButton.
function computeButton_Callback(hObject, eventdata, handles)
% hObject    handle to computeButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%warning off all

set(handles.computeButton,'Enable','off');
set(handles.rotateButton,'Enable','off');
clear global ray

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Reading parameters from GUI:
metFile = get(handles.metFile,'String');
resolution = str2double(get(handles.resolution,'String'));

% phi = str2double(get(handles.azimuth,'String'));
phi = get(handles.azimuth,'String');
phi = regexp(phi,':','split');
for i = 1:numel(phi)
    phiNum(i) = str2double(phi{i});
end

theta = get(handles.launchAngle,'String');
theta = regexp(theta,':','split');
for i = 1:numel(theta)
    thetaNum(i) = str2double(theta{i});
end

maxRange = str2double(get(handles.maxRange,'String'));
perspective = get(handles.perspective,'Value');
turningRays = get(handles.turningRays,'Value');

if (get(handles.shootForward,'Value') == get(handles.shootForward,'Max'))
    reverse = 0;
else
    reverse = 1;
end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Running taup:
time1 = clock;
if (get(handles.garcesMethod,'Value') == get(handles.garcesMethod,'Max'))
    try; thetaNum = (thetaNum(1):thetaNum(2):thetaNum(3)); end
    try; phiNum = (phiNum(1):phiNum(2):phiNum(3)); end
    k = 0;
    for i = 1:numel(thetaNum)
        for j = 1:numel(phiNum)
            k = k + 1;
            update_waitbar(handles,k/(numel(thetaNum)*numel(phiNum)))
            taup_old(metFile,resolution,0,phiNum(j),thetaNum(i),maxRange,1,k,reverse,0,turningRays)
        end
    end
    update_waitbar(handles,0)
else
    try; thetaNum = (thetaNum(1):thetaNum(2):thetaNum(3)); end
    try; phiNum = (phiNum(1):phiNum(2):phiNum(3)); end
    k = 0;
    for i = 1:numel(thetaNum)
        for j = 1:numel(phiNum)
            k = k + 1;
            update_waitbar(handles,k/(numel(thetaNum)*numel(phiNum)))
            taup(metFile,resolution,0,phiNum(j),thetaNum(i),maxRange,1,k,reverse,0,turningRays)
        end
    end
    update_waitbar(handles,0)
end
disp('Processing time (s):')
disp(etime(clock,time1))

global ray
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Applying geographic transformation:
geoCoords = get(handles.geographicCoordinates,'Value');
lat = str2double(get(handles.lat,'String'));
lon = str2double(get(handles.lon,'String'));
if (geoCoords == 2)
    for i = 1:numel(ray)
        RotateXY(i)
        ray(i).x = lon + km2degsc(ray(i).x,lat);
        ray(i).y = lat + (ray(i).y)/111.1949;
    end
end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Plotting rays:

for i = 1:numel(ray)
    N(i) = numel(ray(i).v_g);
    j = find(ray(i).x <= maxRange);
    ray(i).x = ray(i).x(j); ray(i).y = ray(i).y(j); ray(i).z = ray(i).z(j);
end
ray = ray(find(N==1));

set(handles.figure1,'CurrentAxes',handles.axes1);

if (numel(ray) == 1)
    plot3(ray(1).x,ray(1).y,ray(1).z)
else
    c = colormap(jet(numel(ray)));
    for i = 1:numel(ray)
        plot3(ray(i).x,ray(i).y,ray(i).z,'Color',c(i,:));
        hold on
    end
end
if (geoCoords == 1)
    xlabel('Horizontal range (km)')
    ylabel('Transverse offset (km)')
else
    xlabel('Longitude (deg)')
    ylabel('Latitude (deg)')
end

zlabel('Elevation (km)')
zlim([0 140])
% xlim([0 maxRange])
if (perspective == 1)
    view(0,0)
elseif (perspective == 2)
    view(0,90)
end
hold off
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Plotting meteorological data:
set(handles.figure1,'CurrentAxes',handles.axes2);
global MET
plot(MET.c,MET.z,'Color','g')
hold on
plot(MET.v_eff,MET.z)
xlabel('Effective sound speed in propagation direction (m/s)')
ylabel('Elevation (km)')
legend('isotropic','+ wind')
hold off
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Plotting map:
if (geoCoords == 2)
    lats = get(handles.latLimits,'String');
    lats = regexp(lats,',','split');
    for i = 1:numel(lats)
        latLims(i) = str2double(lats{i});
    end

    lons = get(handles.lonLimits,'String');
    lons = regexp(lons,',','split');
    for i = 1:numel(lons)
        lonLims(i) = str2double(lons{i});
    end

    rays = [];
    for i = 1:numel(ray)
        BouncePoints(i);
        rays = cat(1,rays,cat(2,ray(i).bp,repmat(ray(i).v_g,size(ray(i).bp,1),1)));
    end

    figure;
    m_proj('mercator','longitudes',lonLims,'latitudes',latLims);
    m_coast; m_grid; hold on
    %worldmap(latLims,lonLims)
    %scatterm(rays(:,2),rays(:,1),5,rays(:,3))
    [X,Y] = m_ll2xy(rays(:,1),rays(:,2));
    [u,i] = find(X~=min(X) & X~=max(X));
    X = X(u); Y = Y(u); C = rays(u,3);
    [u,i] = find(Y~=min(Y) & Y~=max(Y));
    X = X(u); Y = Y(u); C = C(u);

    scatter(X,Y,5,C);
    colorbar; caxis([0.22 0.34])
    title('Bounce point locations')
end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

set(handles.computeButton,'Enable','on');
set(handles.rotateButton,'Enable','on');

function resolution_Callback(hObject, eventdata, handles)
% hObject    handle to resolution (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of resolution as text
%        str2double(get(hObject,'String')) returns contents of resolution as a double


% --- Executes during object creation, after setting all properties.
function resolution_CreateFcn(hObject, eventdata, handles)
% hObject    handle to resolution (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function maxRange_Callback(hObject, eventdata, handles)
% hObject    handle to maxRange (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of maxRange as text
%        str2double(get(hObject,'String')) returns contents of maxRange as a double


% --- Executes during object creation, after setting all properties.
function maxRange_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maxRange (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in perspective.
function perspective_Callback(hObject, eventdata, handles)
% hObject    handle to perspective (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns perspective contents as cell array
%        contents{get(hObject,'Value')} returns selected item from perspective


% --- Executes during object creation, after setting all properties.
function perspective_CreateFcn(hObject, eventdata, handles)
% hObject    handle to perspective (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function File_Print_Callback(hObject, eventdata, handles)
% hObject    handle to File_Print (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(gcf,'PaperPositionMode','auto')
printpreview


% --- Executes on selection change in turningRays.
function turningRays_Callback(hObject, eventdata, handles)
% hObject    handle to turningRays (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns turningRays contents as cell array
%        contents{get(hObject,'Value')} returns selected item from turningRays


% --- Executes during object creation, after setting all properties.
function turningRays_CreateFcn(hObject, eventdata, handles)
% hObject    handle to turningRays (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function lat_Callback(hObject, eventdata, handles)
% hObject    handle to lat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lat as text
%        str2double(get(hObject,'String')) returns contents of lat as a double


% --- Executes during object creation, after setting all properties.
function lat_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function lon_Callback(hObject, eventdata, handles)
% hObject    handle to lon (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lon as text
%        str2double(get(hObject,'String')) returns contents of lon as a double


% --- Executes during object creation, after setting all properties.
function lon_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lon (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function latLimits_Callback(hObject, eventdata, handles)
% hObject    handle to latLimits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of latLimits as text
%        str2double(get(hObject,'String')) returns contents of latLimits as a double


% --- Executes during object creation, after setting all properties.
function latLimits_CreateFcn(hObject, eventdata, handles)
% hObject    handle to latLimits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function lonLimits_Callback(hObject, eventdata, handles)
% hObject    handle to lonLimits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lonLimits as text
%        str2double(get(hObject,'String')) returns contents of lonLimits as a double


% --- Executes during object creation, after setting all properties.
function lonLimits_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lonLimits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in geographicCoordinates.
function geographicCoordinates_Callback(hObject, eventdata, handles)
% hObject    handle to geographicCoordinates (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns geographicCoordinates contents as cell array
%        contents{get(hObject,'Value')} returns selected item from geographicCoordinates


% --- Executes during object creation, after setting all properties.
function geographicCoordinates_CreateFcn(hObject, eventdata, handles)
% hObject    handle to geographicCoordinates (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in rotateButton.
function rotateButton_Callback(hObject, eventdata, handles)
% hObject    handle to rotateButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.figure1,'CurrentAxes',handles.axes1);
rotate3d
