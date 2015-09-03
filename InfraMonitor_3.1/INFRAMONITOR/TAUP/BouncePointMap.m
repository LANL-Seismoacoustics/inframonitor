function BouncePointMap(lat,lon,metFile,resolution,maxRange,thetaNum,phiNum,latLims,lonLims)
%
% Stephen Arrowsmith (arrows@lanl.gov)
%
% Input Parameters:
% lat = 38.2473; lon = -112.3398;
% metFile = '010311.met';
% resolution = 125;
% maxRange = 1500;
% thetaNum = (1:1:89);
% phiNum = (0:1:359);
% latLims = [36 46]; lonLims = [-119 -108];
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

clear global ray

h = waitbar(0,'Please wait...');
k = 0;
for i = 1:numel(thetaNum)
    for j = 1:numel(phiNum)
        k = k + 1;
        waitbar(k/(numel(thetaNum)*numel(phiNum)),h)
        taup(metFile,resolution,0,phiNum(j),thetaNum(i),maxRange,1,k,0,1,1)
    end
end
close(h);

global ray

for i = 1:numel(ray)
    RotateXY(i)
    ray(i).x = lon + km2degsc(ray(i).x,lat);
    ray(i).y = lat + (ray(i).y)/111.1949;
end

rays = [];
for i = 1:numel(ray)
    BouncePoints(i);
    rays = cat(1,rays,cat(2,ray(i).bp,repmat(ray(i).v_g,size(ray(i).bp,1),1)));
end

I = find(imag(rays(:,1)) == 0);
rays = rays(I,:);

figure;
m_proj('mercator','longitudes',lonLims','latitudes',latLims');
m_coast; m_grid; hold on

[X,Y] = m_ll2xy(rays(:,1),rays(:,2));

[u,i] = find(X~=min(X) & X~=max(X));
X = X(u); Y = Y(u); C = rays(u,3);
[u,i] = find(Y~=min(Y) & Y~=max(Y));
X = X(u); Y = Y(u); C = C(u);

scatter(X,Y,5,C);
colorbar; caxis([0.22 0.34])

[X_0,Y_0] = m_ll2xy(lat,lon);
plot(X_0,Y_0,'rp');

% worldmap(latLims,lonLims)
% scatterm(rays(:,2),rays(:,1),5,rays(:,3))
% title('Bounce point locations')
% colorbar; caxis([0.22 0.34])
