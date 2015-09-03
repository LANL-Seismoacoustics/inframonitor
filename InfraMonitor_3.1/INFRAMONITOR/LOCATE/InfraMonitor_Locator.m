function InfraMonitor_Locator(dbname,grid_bounds,grid_spacing,min_arrays)
%
% e.g., InfraMonitor_Locator('utah',[32.0 45.0 -125.0 -104.0],[0.1 1],4);
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

clear global InfraConfig InfraConfigBackup assoc toassoc AssocIn
clear global arrays
global InfraConfig

InfraConfig.db.name = dbname;

f_in = [InfraConfig.db.name '.detect'];
f = fopen(f_in,'r');
C = textscan(f,'%s %s %s %s %s %f %f %f');
fclose(f);
t = datenum(strcat(C{2},C{3}),'yyyy-mm-ddHH:MM:SS');
arrays = unique(C{1});
azimuth = C{6};
correlation = C{8};

site = readSite([InfraConfig.db.name '.site'],'css3.0');

% Getting array locations:
for i = 1:numel(arrays)
    for j = 1:numel(site{1})
        if (strcmp(arrays{i},site{1}{j}) == 1)
            loc{i} = [site{4}(j) site{5}(j)];
            break
        end
    end
end

% Creating InfraConfig.array:
InfraConfig.array.loc = loc;

for i = 1:numel(arrays)
    
    k = 0;
    for j = 1:numel(C{1})
        if (strcmp(arrays{i},C{1}{j}) == 1)
            k = k + 1;
            I(k) = j;
        end
    end
    
    InfraConfig.array.arr{i} = cat(2,t(I),zeros(numel(t(I)),1),azimuth(I));
    I=[];
end

arrayLocations = cat(1,InfraConfig.array.loc{:});

MakeGrid([InfraConfig.db.name '.mat'],grid_bounds,grid_spacing,arrayLocations(:,1),arrayLocations(:,2));


% Opening grid file:
grid = load([InfraConfig.db.name '.mat'],'-mat');
%     D_STN = grid.grid.D_STN;
%     BA_STN = grid.grid.BA_STN;
%     grid.grid.D_STN = D_STN;
%     grid.grid.BA_STN = BA_STN;
InfraConfig.grid = grid.grid;
clear grid

% Saving detection results:
% save InfraMonitor2b.mat

% Performing association and location:
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

