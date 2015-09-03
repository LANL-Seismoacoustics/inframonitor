%--------------------------------------------------------------------------
% RunGABISL.m
%
% Runs Association and BISL routines given a set of arrival files
%
% INPUT PARAMETERS:
DetectFile = '08_01_2011.detect';
Arrays = 'BRD,CHN,KSG';
% Grid dimensions (lat_min lat_max lon_min lon_max):
gridDim = [33.0 46.0 121.0 145.0];
% Grid spacing (degrees):
gridSpac = 0.1;
% Array latitudes:
stn_lat = [37.966 38.59 42.5];
% Array longitudes:
stn_lon = [124.642 128.357 126];

min_arrays = 3;
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

MakeGrid('08_01_2011.mat',gridDim,[gridSpac gridSpac],stn_lat,stn_lon);
load 08_01_2011.mat
InfraConfig.grid = grid;

Arrays = list2cell(Arrays);

% Reading Arrival File:
f = fopen(DetectFile,'r');
x = textscan(f,'%s %s %s %s %s %f %f %f');
fclose(f);

t = datenum(strcat(x{2},x{3}),'yyyy-mm-ddHH:MM:SS');
a = x{6};

for i = 1:numel(Arrays)
    u = strmatch(Arrays{i},x{1});
    dims = size(x{2}(u));
    InfraConfig.array.arr{i} = cat(2,t(u),ones(dims),a(u),...
        ones(dims),ones(dims)*2,ones(dims),ones(dims),ones(dims));
end

% % THESE NEED TO BE POPULATED:
% InfraConfig.array.trc = [];
% InfraConfig.array.loc = [];

% Performing association and location:
try
    global toassoc InfraConfigBackup

    InfraConfigBackup.array = InfraConfig.array;
    InfraConfigBackup.grid = InfraConfig.grid;

    NArrays = numel(InfraConfigBackup.array.arr);
    narrays = (min_arrays:1:NArrays);

    C = progress('init','InfraMonitor2: Performing associations');
    for i = 1:numel(narrays)

        C = progress(C,i/numel(narrays));

        DoAssociation(narrays(i),NArrays);

        if (toassoc == 0)
            break
        end

    end

    InfraConfig.array = InfraConfigBackup.array;
    InfraConfig.grid = InfraConfigBackup.grid;

    k = 0; BISLMain2(k);
catch
    disp('No grid file...cannot perform associations/locations')
end
