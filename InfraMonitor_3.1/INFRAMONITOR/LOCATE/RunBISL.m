%--------------------------------------------------------------------------
% RunBISL.m
%
% Runs BISL on an input set of arrival measurements at different arrays
%
% INPUT PARAMETERS:
% Grid dimensions (lat_min lat_max lon_min lon_max):
gridDim = [33.0 34.5 -107.5 -105];
% Grid spacing (degrees):
gridSpac = 0.01;
% Array latitudes:
stn_lat = [34.0476 34.081500 33.91626 33.640826 33.521838 33.187871];
% Array longitudes:
stn_lon = [-106.8590 -106.606180 -105.42606 -107.095965 -105.519366 -106.949672];
% Arrival times (use dummy values if using Spherical coordinates):
assoc.atimes = [datenum(2007,8,1,1,4,41) datenum(2007,8,1,1,3,41) datenum(2007,8,1,1,6,21) datenum(2007,8,1,1,3,13) datenum(2007,8,1,1,4,45) datenum(2007,8,1,1,2,39)];
% Backazimuths:
assoc.baz = [147.9074 167.8285 239.3814 109 263 62];
%--------------------------------------------------------------------------
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
% Redistributions of source code must retain the above copyright notice, this list
% 		  of conditions and the following disclaimer.
% Redistributions in binary form must reproduce the above copyright notice, this
% 	      list of conditions and the following disclaimer in the documentation and/or
% 	      other materials provided with the distribution.
% Neither the name of Los Alamos National Security, LLC, Los Alamos National
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

InfraUser = open('InfraUser.mat');

MakeGrid('RunBISL.mat',gridDim,[gridSpac gridSpac],stn_lat,stn_lon);
assoc.arrays = (1:numel(assoc.baz));

load RunBISL.mat

% load Example.mat

NEvents = numel(assoc);

% Calculating 2D Credibility matrices:
for i = 1:NEvents
    
    %Evaluate each point in parameter space
    BISLGridLoop2(assoc(i),stn_lat(assoc(i).arrays),stn_lon(assoc(i).arrays),InfraUser);

    %Compute margian PDFs, compute credibility values
    BISLMarginalize();
    
    % Storing credibilities (which are unique for each association):
    InfraConfig.BISL.CREDIBILITIES{i} = InfraConfig.BISL.Credibilities;
    InfraConfig.BISL.Credibilities = [];
    
end

% Calculating location polygons at 95% confidence region:
for i = 1:NEvents
    
    % Calculating Polygon:;
    C = contourc(InfraConfig.BISL.lon,InfraConfig.BISL.lat,...
        InfraConfig.BISL.CREDIBILITIES{i}{1},[0.95 0.95]);
    x = C(1,2:size(C,2)); y = C(2,2:size(C,2));         % Note: 1st elements are removed
    y_t = mean(y); x_t = mean(x);
    
    % Do something here to right these out or plot them...
    figure
%     c = load('/Users/arrows/coast.dat','-ascii');
    worldmap([gridDim(1) gridDim(2)],[gridDim(3) gridDim(4)])
    hold on
%     plotm(c(:,2),c(:,1),'k')
    plotm(y,x)
    plotm(stn_lat,stn_lon,'k^','MarkerFaceColor','y','MarkerSize',8)
%     plotm(41.279,129.065,'rp','MarkerFaceColor','r','MarkerSize',8)
    
end

% Adding backazimuths to the plot:
for (i = 1:numel(stn_lat))
    [lat,lon] = track1(stn_lat(i),stn_lon(i),assoc.baz(i),km2deg(1000));
    plotm(lat,lon,'b--')
end
