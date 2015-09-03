function Locate(Y)
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
global arrays

InfraUser = open('InfraUser.mat');

for i = 1:numel(InfraConfig.array.loc)
    stn_lat(i) = InfraConfig.array.loc{i}(1);
    stn_lon(i) = InfraConfig.array.loc{i}(2);
end

Y = FlipY(Y);

Assoc = MSEI(Y,InfraUser.az_dev,InfraUser.vmin,InfraUser.vmax,...
    stn_lon,stn_lat,InfraConfig.grid.D_STN,InfraConfig.grid.BA_STN,...
    InfraConfig.grid.LAT,InfraConfig.grid.LON,InfraUser.Ep);

if (Assoc == 0)
    msgbox('InfraMonitor2: No Events Found!')
    return
end

ARIDS = Assoc(:,3:size(Assoc,2));
unique_ARIDS = unique(ARIDS,'rows');
NEvents = size(unique_ARIDS,1);

fid = fopen([InfraConfig.db.name '.IMpoly'],'w');
fid2 = fopen([InfraConfig.db.name '.origin'],'w');
fid3 = fopen([InfraConfig.db.name '.IMassoc'],'w');
for i = 1:NEvents
    
    % Calculating Polygon:
    u = ismember(ARIDS,unique_ARIDS(i,:),'rows');
    LAT = Assoc(find(u==1),1);
    LON = Assoc(find(u==1),2);
    [x,y] = Grid2Polygon(LAT,LON);
    
    [y_t,x_t] = meanm(LAT,LON);
    
    EvArids = unique_ARIDS(i,:);
    
    %***
    % Obtaining Arrival Times:
    ArrAll = [];
    for l = 1:numel(InfraConfig.array.arr)
        ArrAll = cat(1,ArrAll,InfraConfig.array.arr{l}(InfraConfig.array.select{l},:));
        t(l) = ArrAll(EvArids(l),1);
    end
    %***
    
    %---
    % Loading schema parameters:
    Schema_File = which('Schema.mat');
    Schema = load(Schema_File);
    %---
    % Writing Event Parameters to IMpoly File:
    % (Note: IM2 schema)
    for j = 1:numel(x)
        fprintf(fid,'%9d %9d %11.6f %11.6f\n',i,j,y(j),x(j));
    end
    %---
    % Writing to origin file:
    dists = vdist(y_t,x_t,stn_lat',stn_lon')/1000;
    vmean = (InfraUser.vmax + InfraUser.vmin)/2;
    tt = dists/vmean;
    o_time = iepoch(mean(t' - tt/86400));       % Mean origin time
    fprintf(fid2,Schema.origin,y_t,x_t,o_time,i,i);
    %---
    % Writing to IMassoc file:
    % (Note: IM2 schema)
    for j = 1:numel(unique_ARIDS(i,:))
        
        fprintf(fid3,'%17.5f %8d\n',iepoch(t(j)),i);
        
    end
    %---
    
end
fclose(fid);
fclose(fid2);
fclose(fid3);

disp(['InfraMonitor2: Found ' num2str(i) ' events'])

