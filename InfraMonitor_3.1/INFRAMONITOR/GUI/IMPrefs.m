function varargout = IMPrefs(varargin)
% IMPREFS M-file for IMPrefs.fig
%      IMPREFS, by itself, creates a new IMPREFS or raises the existing
%      singleton*.
%
%      H = IMPREFS returns the handle to a new IMPREFS or the handle to
%      the existing singleton*.
%
%      IMPREFS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in IMPREFS.M with the given input arguments.
%
%      IMPREFS('Property','Value',...) creates a new IMPREFS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before IMPrefs_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to IMPrefs_OpeningFcn via varargin.
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

% Edit the above text to modify the response to help IMPrefs

% Last Modified by GUIDE v2.5 20-Dec-2010 08:38:47

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @IMPrefs_OpeningFcn, ...
                   'gui_OutputFcn',  @IMPrefs_OutputFcn, ...
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


% --- Executes just before IMPrefs is made visible.
function IMPrefs_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to IMPrefs (see VARARGIN)

InfraUser = open('InfraUser.mat');

set(handles.twin,'String',num2str(InfraUser.twin));
set(handles.overlap,'String',num2str(InfraUser.overlap));
set(handles.w,'String',num2str(InfraUser.w));
set(handles.p_val,'String',num2str(InfraUser.p));
set(handles.az_dev,'String',num2str(InfraUser.az_dev));
set(handles.vmin,'String',num2str(InfraUser.vmin));
set(handles.vmax,'String',num2str(InfraUser.vmax));
set(handles.Ep,'String',num2str(InfraUser.Ep));
set(handles.sigma1,'String',num2str(InfraUser.sigma1));
set(handles.sigma2,'String',num2str(InfraUser.sigma2));
set(handles.len3,'String',num2str(InfraUser.len3));
set(handles.flat_earth,'String',num2str(InfraUser.flat_earth));

% Choose default command line output for IMPrefs
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes IMPrefs wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = IMPrefs_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function CancelButton_Callback(hObject, eventdata, handles)
delete(handles.figure1);


function SaveButton_Callback(hObject, eventdata, handles)
twin = str2double(get(handles.twin,'String'));
overlap = str2double(get(handles.overlap,'String'));
w = str2double(get(handles.w,'String'));
p = str2double(get(handles.p_val,'String'));
az_dev = str2double(get(handles.az_dev,'String'));
vmin = str2double(get(handles.vmin,'String'));
vmax = str2double(get(handles.vmax,'String'));
Ep = str2double(get(handles.Ep,'String'));
sigma1 = str2double(get(handles.sigma1,'String'));
sigma2 = str2double(get(handles.sigma2,'String'));
len3 = str2double(get(handles.len3,'String'));
flat_earth = str2double(get(handles.flat_earth,'String'));
FName = which('InfraUser.mat');
save(FName,'twin','overlap','w','p','az_dev','vmin','vmax','Ep',...
    'sigma1','sigma2','len3','flat_earth');
delete(handles.figure1);


function twin_Callback(hObject, eventdata, handles)


function twin_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function overlap_Callback(hObject, eventdata, handles)


function overlap_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function w_Callback(hObject, eventdata, handles)


function w_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function p_val_Callback(hObject, eventdata, handles)


function p_val_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
function az_dev_Callback(hObject, eventdata, handles)


function az_dev_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function vmax_Callback(hObject, eventdata, handles)


function vmax_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function vmin_Callback(hObject, eventdata, handles)


function vmin_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function Ep_Callback(hObject, eventdata, handles)


function Ep_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
function sigma1_Callback(hObject, eventdata, handles)

function sigma1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function sigma2_Callback(hObject, eventdata, handles)

function sigma2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function len3_Callback(hObject, eventdata, handles)

function len3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function flat_earth_Callback(hObject, eventdata, handles)
flat_earth = str2double(get(handles.flat_earth,'String'));
if flat_earth ~= 0, set(handles.flat_earth,'String','1'); end

function flat_earth_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function sactosac_path_Callback(hObject, eventdata, handles)
% hObject    handle to sactosac_path (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of sactosac_path as text
%        str2double(get(hObject,'String')) returns contents of sactosac_path as a double


% --- Executes during object creation, after setting all properties.
function sactosac_path_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sactosac_path (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
