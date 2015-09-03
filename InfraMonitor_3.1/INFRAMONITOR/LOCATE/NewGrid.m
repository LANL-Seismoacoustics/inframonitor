function NewGrid()
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

grid = inputdlg({'InfraMonitor2: Enter grid limits','Enter grid spacing',...
    'Enter output file'},'New Grid',1,{'lat0,lat1,lon0,lon1','dmin,dmax',...
    'Grid.mat'});
try
    gloc = list2array(grid{1});
    InfraConfig.grid.glat = gloc(1:2); InfraConfig.grid.glon = gloc(3:4);
    dspac = list2array(grid{2});
    InfraConfig.grid.dmin = dspac(1); InfraConfig.grid.dmax = dspac(2);
    fout = grid{3};
catch
    return
end

for i = 1:numel(InfraConfig.array.loc)
    slat(i) = InfraConfig.array.loc{i}(1);
    slon(i) = InfraConfig.array.loc{i}(2);
end

% Generating initial grid (constant grid spacing):
h = waitbar(0,'InfraMonitor2: Generating initial grid');
i = 1; j = 0; dmin_acc = 0;
lat(i) = InfraConfig.grid.glat(1); lon(i) = InfraConfig.grid.glon(1);
while lat(i) < InfraConfig.grid.glat(2)
    j = j + 1;
    waitbar(dmin_acc/(InfraConfig.grid.glat(2)-InfraConfig.grid.glat(1)),h);
    while lon(i) < InfraConfig.grid.glon(2)
        i = i + 1;
        lat(i) = lat(i-1);
        lon(i) = lon(i-1) + InfraConfig.grid.dmin;
    end
    i = i + 1;
    lat(i) = lat(i-1) + InfraConfig.grid.dmin;
    lon(i) = InfraConfig.grid.glon(1);
    dmin_acc = InfraConfig.grid.dmin*j;  % For waitbar
end

if (InfraConfig.grid.dmin ~= InfraConfig.grid.dmax)

    % Generating straight-line function (mapping distance to grid spacing)
    % Finding grid edges:
    k = 0;
    for i = 1:numel(InfraConfig.grid.glat)
        for j = 1:numel(InfraConfig.grid.glon)
            k = k + 1;
            loc{k} = [InfraConfig.grid.glat(i) InfraConfig.grid.glon(j)];
        end
    end
    % Computing range to closest array for each grid edge:
    for i = 1:numel(loc)
        for j = 1:numel(slat)
            [d(j),a(j),b(j)] = dist_az(loc{i},[slat(j) slon(j)]);
        end
        D(i) = (min(d))*111.1;
        clear d
    end
    % Computing Max. range for region (in km):
    D = max(D);
    m = (InfraConfig.grid.dmax - InfraConfig.grid.dmin)/D;
    c = InfraConfig.grid.dmin;


    % Reparameterizing grid (variable grid spacing):
    i = 0; test = 0;
    waitbar(0,h,'InfraMonitor2: Reparameterizing grid')
    while (test == 0)
        i = i + 1;
        waitbar(i/numel(lat),h)   
        % Exiting loop if reached final point to process:
        if (i > numel(lat))
            break
        end    
        L1 = lat(i); L2 = lon(i);
        % Computing appropriate grid spacing for location:
        for j = 1:numel(slat)
            d(j) = dist_az([L1 L2],[slat(j) slon(j)]);
        end
        D2 = (min(d))*111.1;
        GS = (m*D2 + c);
        N = GS/InfraConfig.grid.dmin;
        % Finding and removing grid nodes within appropriate grid spacing:
        lat_tmp = lat(find(L1 <= lat & lat < L1+(N*InfraConfig.grid.dmin)));
        lon_tmp = lon(find(L1 <= lat & lat < L1+(N*InfraConfig.grid.dmin)));
        lat_tmp = lat_tmp(find(L2 <= lon_tmp & lon_tmp < L2+(N*InfraConfig.grid.dmin)));
        lon_tmp = lon_tmp(find(L2 <= lon_tmp & lon_tmp < L2+(N*InfraConfig.grid.dmin)));    
        loc = cat(2,lat',lon');
        loc_tmp = cat(2,lat_tmp',lon_tmp');
        if (numel(loc_tmp) == 0)
            continue
        end
        loc = setdiff(loc,loc_tmp,'rows');
        loc = cat(1,[L1 L2],loc);
        lat = loc(:,1)';
        lon = loc(:,2)';    
    end
end

% Computing association parameters:
waitbar(0,h,'InfraMonitor2: Computing association parameters for each grid node');
for i = 1:numel(slat)
    waitbar(i/numel(slat),h);
    k = 0;
    for j = 1:numel(lat)
        k = k + 1;
        [D(k),B(k)] = dist_az([slat(i) slon(i)],[lat(j) lon(j)]);
    end
    InfraConfig.grid.D_STN{i} = 111.1949*(D);
    InfraConfig.grid.BA_STN{i} = B;
    clear D B
end
close(h)

LAT = lat;
LON = lon;

InfraConfig.grid.LAT = lat;
InfraConfig.grid.LON = lon;

grid = InfraConfig.grid;

save(fout,'grid')
