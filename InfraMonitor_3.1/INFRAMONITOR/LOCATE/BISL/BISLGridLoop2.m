function BISLGridLoop2(assoc,stn_lat,stn_lon,InfraUser)
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

C = progress('init','InfraMonitor2: Evaluating each point in the parameter space');

%Unit conversion factors
kmperdeg = 110.57;
secperday = 24*60*60;

%Load station and grid information
global InfraConfig
% InfraUser = open('InfraUser.mat');

NEvents = numel(assoc);
NArrays = numel(assoc.arrays);
len = cell(NEvents,1);
PDF4 = cell(NEvents,1);

sigma1 = InfraUser.sigma1;
sigma2 = InfraUser.sigma2;
len3 = InfraUser.len3;
flat_earth = InfraUser.flat_earth;

%LOOP OVER EVENTS
for p = 1:NEvents

    %Load arrival time and back azimuth observations for given event
%     trc_list = stn_list(p,:);
    for q = 1:NArrays
        theta(q) = assoc.baz(q);
        t(q) = assoc.atimes(q);
%         trc_off = find(Y{q}{4} == trc_list(q));
%         theta(q) = Y{q}{3}(trc_off);
%         t(q) =  datenum(strcat(Y{q}{1}(trc_off),Y{q}{2}(trc_off)),'yyyy-mm-ddHH:MM:SS');
    end
    
    %Construct spatial vectors
    lon_min = InfraConfig.grid.glon(1); lat_min = InfraConfig.grid.glat(1);
    lon_max = InfraConfig.grid.glon(2); lat_max = InfraConfig.grid.glat(2);
    lon_step = InfraConfig.grid.dmin;   lat_step = InfraConfig.grid.dmin;
    lon = lon_min:lon_step:lon_max;
    lat = lat_min:lat_step:lat_max;
    lon_off = min(min(lon_max-stn_lon),min(stn_lon-lon_min));
    lat_off = min(min(lat_max-stn_lat),min(stn_lat-lat_min));
    dist_max= sqrt((lon_max-lon_min-lon_off)^2+(lat_max-lat_min-lat_off)^2);
    N(1) = numel(lat); N(2) = numel(lon);
    N(3) = len3;  N(4) = len3;
    len{p}(1) = lat(N(1))-lat(1); len{p}(2) = lon(N(2))-lon(1);

    %Construct origin time vector
    t0_min = min(t) - (dist_max*kmperdeg/InfraUser.vmin)/secperday;
    t0_max = min(t);
    t0_step = (t0_max-t0_min)/N(3);
    t0 = t0_min:t0_step:t0_max;
    len{p}(3) = t0(N(3)) - t0(1);
    %***
%     t0 = [datenum(2011,1,1,0,0,0) datenum(2011,1,1,0,0,1)];
%     len{p}(3) = 1/86400;
    %***
    
    %Construct group velocity vector
    vstep=(InfraUser.vmax-InfraUser.vmin)/N(4); 
    v=InfraUser.vmin:vstep:InfraUser.vmax;
    len{p}(4) = v(N(4))-v(1);
    %***
%     v = [0.28 0.281];
%     len{p}(4) = 001;
    %***

    Z{p} = zeros(N(1),N(2),N(3),N(4));

    for i=1:N(1)
    for j=1:N(2)
    for k=1:N(3)
    for l=1:N(4)

    n1 = 0;
    n2 = 0;
    sum_exp1 = 0;
    sum_exp2 = 0;

    for q=1:NArrays

    %Compare predicted and observed back azimuths for given station
    n1 = n1 + 1;
    if (flat_earth == 1)
        theta_clc = -(180/pi)*atan2(lat(i)-stn_lat(q),lon(j)-stn_lon(q))+90;
    else
        theta_clc = azimuth(stn_lat(q),stn_lon(q),lat(i),lon(j));
    end
    if theta_clc < 0, theta_clc = theta_clc + 360; end
    theta_obs = theta(q);
    gamma = theta_clc - theta_obs;
    if gamma > 180, gamma = 360 - gamma; end
    if gamma < -180, gamma = 360 + gamma; end
    sum_exp1 = sum_exp1 + gamma^2;

    %Compute predicted and observed arrival times for given station
    n2 = n2 + 1;
    if (flat_earth == 1)
        dist =  sqrt((stn_lat(q)-lat(i))^2+(stn_lon(q)-lon(j))^2)*kmperdeg;
    else
        dist = deg2km(distance(stn_lat(q),stn_lon(q),lat(i),lon(j)));
    end
    eps = (t(q) - t0(k))*secperday - dist/v(l);
    sum_exp2 = sum_exp2 + eps^2;
    
    end

    %Store combined likelihood value
    if (flat_earth == 1)
        PDF4{p}(i,j,k,l) = 1/(2*pi*sigma1^2)^(n1/2)*exp(-0.5*sum_exp1/sigma1^2)*...
                           1/(2*pi*sigma2^2)^(n2/2)*exp(-0.5*sum_exp2/sigma2^2);
    else
        PDF4{p}(i,j,k,l) = 1/(2*pi*sigma1^2)^(n1/2)*exp(-0.5*sum_exp1/sigma1^2);
    end
    
% *** The following two lines are for using the backazimuths or arrival
% times separately ***
%    PDF4{p}(i,j,k,l) = 1/(2*pi*sigma1^2)^(n1/2)*exp(-0.5*sum_exp1/sigma1^2);
%    PDF4{p}(i,j,k,l) = 1/(2*pi*sigma2^2)^(n2/2)*exp(-0.5*sum_exp2/sigma2^2);
    
    end
    end
    end
    C = progress(C,(p-1+i/N(1))/NEvents);
    end
    if (ndims(PDF4{p})>2)
        PDF4{p} = PDF4{p}/sumquad(PDF4{p},len{p});
    else
        PDF4{p} = PDF4{p}./(sum(sum(PDF4{p}))*len{p}(1)*len{p}(2)*size(PDF4{p},1)^-1*size(PDF4{p},2)^-1);
    end

end

InfraConfig.BISL.lat = lat;
InfraConfig.BISL.lon = lon;
InfraConfig.BISL.len = len;
InfraConfig.BISL.PDF4 = PDF4;
