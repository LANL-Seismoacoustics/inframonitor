function RunLocate(dbname,Arrays,ArrayLocs,a,vmin,vmax,Ep)
% RunLocate - Runs Association/Location on an input set of arrival files
% and grid file
%
% e.g.,
% RunLocate('KOREA','BRD,CHN,KSG',{[37.9657 124.6444],[38.2711 127.1210],...
%   [38.5899 128.3562]},8,0.28,0.35,10)
%
% or, to process 2 arrays only (Note: requires a different GRID file),
% RunLocate('KOREA','CHN,KSG',{[38.2711 127.1210],[38.5899 128.3562]},8,0.28,0.35,10)
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

InfraConfig.array.loc = ArrayLocs;

% Input Files:
ArrivalFile = [dbname '.arrival'];
SiteFile = [dbname '.site'];
GridFile = [dbname '.mat'];

% Association/Location Parameters:
InfraUser.az_dev = a;
InfraUser.vmin = vmin;
InfraUser.vmax = vmax;
InfraUser.Ep = Ep;

Arrays = list2cell(Arrays);
InfraConfig.db.name = dbname;

% Reading Arrival File:
f = fopen(ArrivalFile,'r');
x = textscan(f,'%s %f %f %s %f %f %f %f %s');
fclose(f);

% Creating InfraConfig.array.arr:
for i = 1:numel(x)
    x{i} = x{i}(1:4:numel(x{i}));
end
for i = 1:numel(Arrays)
    u = strmatch(Arrays{i},x{1});
    dims = size(x{2}(u));
    InfraConfig.array.arr{i} = cat(2,epoch(x{2}(u)),epoch(x{2}(u)),x{5}(u),...
        ones(dims),ones(dims)*2,x{6}(u),x{7}(u),x{8}(u));
end

% Creating InfraConfig.db:
% Extracting first 5 columns from site file:
fid = fopen(SiteFile);
i = 0;
while 1
    i = i + 1;
    tline = fgetl(fid);
    if ~ischar(tline),   break,   end
    tline = regexp(tline,'\S*','match');
    if (i == 1)
        for j = 1:5
            InfraConfig.db.site{j} = tline(j);
        end
    else
        for j = 1:5
            InfraConfig.db.site{j} = cat(1,InfraConfig.db.site{j},tline{j});
        end
    end
end
fclose(fid);

try
	% Opening grid file:
	grid = load(GridFile);
	InfraConfig.grid = grid.grid;
	clear grid
catch
	disp('RunLocate: Cannot find grid file');
end

try
	Arrivals = FirstArrivals();
	Locate(Arrivals);
catch
	disp('InfraMonitor2b: Did not perform association/location processing');
    save InfraMonitor2b.mat
end
