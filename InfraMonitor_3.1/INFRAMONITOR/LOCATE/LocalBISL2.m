function area = LocalBISL2(assoc,stn_lat,stn_lon,grid_file,InfraUser)
%
% LocalBISL.m - Runs BISL for an input event specified by assoc, and for a
% certain network configuration specified by stn_lat, stn_lon, and
% grid_file
%
% Note: Uses the input parameters defined in IMPrefs
%
% Stephen Arrowsmith (arrows@lanl.gov)
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

clear global InfraConfig
global InfraConfig

% Loading grid file:
load(grid_file)         % Note: grid_file modified to have format InfraConfig.grid

NEvents = numel(assoc);

% Calculating 2D Credibility matrices:
for i = 1:NEvents
    
    %Evaluate each point in parameter space
    BISLGridLoop3(assoc(i),stn_lat(assoc(i).arrays),stn_lon(assoc(i).arrays),InfraUser);

    %Compute margian PDFs, compute credibility values
    BISLMarginalize();
    
    % Storing credibilities (which are unique for each association):
    InfraConfig.BISL.CREDIBILITIES{i} = InfraConfig.BISL.Credibilities;
    InfraConfig.BISL.Credibilities = [];
    
end

% Calculating location polygons at 95% confidence region:
for i = 1:NEvents
    
    if (i > 1)
        keyboard
    end
    
    % Calculating Polygon:;
    C = contourc(InfraConfig.BISL.lon,InfraConfig.BISL.lat,...
        InfraConfig.BISL.CREDIBILITIES{i}{1},[0.95 0.95]);
    
    contourStartIndices = find(C(1,:) == 0.95);
    
    % Looping over each polygon:
    for j = 1:numel(contourStartIndices)
        nPoints = C(2,contourStartIndices(j));
        x = C(1,contourStartIndices(j)+1:contourStartIndices(j)+nPoints);
        y = C(2,contourStartIndices(j)+1:contourStartIndices(j)+nPoints);
        areas(j) = areaint(y,x,earthRadius('km'));
    end
    
    try
        area = sum(areas);
    catch
        area = deg2km(InfraConfig.grid.dmin)^2;     % Hard-wiring the location area based on the minimum grid spacing
    end
    
end
