function varargout = BISLMain(Y,varargin)
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

%-------------------------------------------------------
global InfraConfig
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
    disp('InfraMonitor2: No Events Found!')
    varargout{1} = varargin{1};
    return
end

ARIDS = Assoc(:,3:size(Assoc,2));
unique_ARIDS = unique(ARIDS,'rows');
%-------------------------------------------------------


%Evaluate each point in parameter space
BISLGridLoop(Y,unique_ARIDS,stn_lat,stn_lon);

%Compute margian PDFs, compute credibility values
BISLMarginalize();

%-------------------------------------------------------
% Added 12/21/2009:
NEvents = size(unique_ARIDS,1);

fid = fopen([InfraConfig.db.name '.IMpoly'],'a');
fid2 = fopen([InfraConfig.db.name '.origin'],'a');
fid3 = fopen([InfraConfig.db.name '.IMassoc'],'a');
for i = 1:NEvents
    
    % Calculating Polygon:
    EvArids = unique_ARIDS(i,:);
    C = contourc(InfraConfig.BISL.lon,InfraConfig.BISL.lat,...
        InfraConfig.BISL.Credibilities{i},[0.95 0.95]);
    x = C(1,2:size(C,2)); y = C(2,2:size(C,2));         % Note: 1st elements are removed
    %[y_t,x_t] = meanm(y,x);
    y_t = mean(y); x_t = mean(x);
    
    % Obtaining Arrival Times:
    ArrAll = [];
    for l = 1:numel(InfraConfig.array.arr)
        ArrAll = cat(1,ArrAll,InfraConfig.array.arr{l}(InfraConfig.array.select{l},:));
        t(l) = ArrAll(EvArids(l),1);
        b(l) = ArrAll(EvArids(l),3);
    end
    
    % ***
    InfraConfig.BISL.baz{i} = b;
    InfraConfig.BISL.meanloc{i} = [x_t y_t];
    % ***
    
    % Loading schema parameters:
    Schema_File = which('Schema.mat');
    Schema = load(Schema_File);
    
    % Writing Event Parameters to IMpoly File:
    % (Note: IM2 schema)
    for j = 1:numel(x)
        try
            fprintf(fid,'%9d %9d %11.6f %11.6f\n',varargin{1}+i,varargin{1}+j,y(j),x(j));
        catch
            fprintf(fid,'%9d %9d %11.6f %11.6f\n',i,j,y(j),x(j));
        end
    end
    
    % Writing to origin file:
    try
        dists = vdist(repmat(y_t,numel(stn_lat),1),repmat(x_t,numel(stn_lon),1),stn_lat',stn_lon')/1000;
    catch
        return
    end
    vmean = (InfraUser.vmax + InfraUser.vmin)/2;
    tt = dists/vmean;
    o_time = iepoch(mean(t' - tt/86400));       % Mean origin time
    try
        fprintf(fid2,Schema.origin,y_t,x_t,o_time,varargin{1}+i,varargin{1}+i);
    catch
        fprintf(fid2,Schema.origin,y_t,x_t,o_time,i,i);
    end
    % Writing to IMassoc file:
    % (Note: IM2 schema)
    for j = 1:numel(unique_ARIDS(i,:))
        
        try
            fprintf(fid3,'%s %8d\n',datestr(t(j),'yyyy-mm-dd HH:MM:SS'),varargin{1}+i);
        catch
            fprintf(fid3,'%s %8d\n',datestr(t(j),'yyyy-mm-dd HH:MM:SS'),i);
        end
        
    end
    
end
fclose(fid);
fclose(fid2);
fclose(fid3);

try; varargout{1} = varargin{1}+i; end
%-------------------------------------------------------

disp(['InfraMonitor2: Found ' num2str(NEvents) ' events'])
