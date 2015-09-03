function fullwave(metfile,dz,h,range,fullray,evloc,lonlim,latlim,dx,dy)
%
% Computes and plots all bounce points for a given source and geographic
% area
%
% Where:
% metfile, dz, h, range (max_range), and fullray are defined for taup.m
% evloc is the event location
% lonlim are the longitude limits (for plotting)
% latlim are the latitude limits (for plotting)
% dx and dy are the pixel sizes in the image plot
%
% e.g.,
% fullwave('2005012512.met',250,0,500,1,[32.0 -106.0],[-120 -90],[24 40],0.5,0.5)
%
% Stephen Arrowsmith (sarrowsmith@gmail.com)
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

global ray

ray = runtaup(metfile,(0:1:359),(1:1:90),range,dz,h,0,fullray);

%rays = zeros(numel(ray),5);
k = 0; kk = 0;
for i = 0:1:359
    for j = 1:1:90
        k = k + 1;
        try
            for l = 1:numel(ray(k).x)
                kk = kk + 1;
                rays(kk,1) = i;                                              % Azimuth
                rays(kk,2) = j;
                rays(kk,3) = evloc(2) + km2degsc(ray(k).x(l),evloc(1));     % Longitude (bounce point)
                rays(kk,4) = evloc(1) + (ray(k).y(l))/111.1949;                % Latitude (bounce point)
                rays(kk,5) = ray(k).v_g;                                 % Group velocity
            end
        catch
            rays(kk,3) = 0;
            rays(kk,4) = 0;
            rays(kk,5) = 0;
        end
    end
end

[u,v] = find (rays(:,5) > 0.28);
rays = rays(u,:);

figure;
m_proj('mercator','longitudes',[30 50]','latitudes',[-124 -104]');
m_coast; m_grid; hold on
[X,Y] = m_ll2xy(rays(:,3),rays(:,4));
[u,i] = find(X~=min(X) & X~=max(X));
X = X(u); Y = Y(u); C = rays(u,5);
[u,i] = find(Y~=min(Y) & Y~=max(Y));
X = X(u); Y = Y(u); C = C(u);

scatter(X,Y,5,C);
% worldmap([30 50],[-124 -104])
% scatterm(real(rays(:,4)),real(rays(:,3)),5,rays(:,5))

save('rays.mat','rays');

llx = (lonlim(1):dx:lonlim(2));
lly = (latlim(1):dy:latlim(2));
for i = 1:numel(llx)
    for j = 1:numel(lly)
        u = find(llx(i)-dx <= rays(:,3) & rays(:,3) <= llx(i)+dx);
        v = find(lly(j)-dy <= rays(u,4) & rays(u,4) <= lly(j)+dy);
        if (numel(v) == 0)
            CC(i,j) = NaN;
        else
            CC(i,j) = mean(rays(u(v),5));
        end
    end
end

% Plotting map:

figure

c = colormap(jet(100));
% C = linspace(min(real(rays(:,5))),max(real(rays(:,5))),100)
C = linspace(0.28,0.35,100);

m_proj('mercator','longitudes',lonlim,'latitudes',latlim);
m_coast; m_grid; hold on

[X,Y]=m_ll2xy(llx,lly);
imagescnan(X,Y,CC'); axis xy
caxis([0.28 0.35])

m_coast; m_grid;

% [X,Y]=m_ll2xy(evloc(2),evloc(1));
% plot(X,Y,'marker','p','markersize',10,'color','r');

% [X,Y]=m_ll2xy(rays(:,3),rays(:,4));
% for i = 1:numel(X)
%     [U,V] = min(abs(C-rays(i,5)));
%     plot(X(i),Y(i),'marker','o','markersize',5,'color',c(V,:));
% end

% cbar = colorbar;
% label = strvcat(num2str(C(1),2),num2str(C(20),2),num2str(C(40),2),...
%     num2str(C(60),2),num2str(C(80),2),num2str(C(100),2))
% set(cbar,'YTick',[0 0.2 0.4 0.6 0.8 1])
% set(cbar,'YTickLabel',label)

while 1
    user = input('Pick bounce point (y/n)? ','s');
    if (user == 'y')
        [x,y] = ginput(1);
        [lon,lat] = m_xy2ll(x,y);
%         d = distance(lat,lon,rays(:,4),rays(:,3));
        d = m_idist(lon,lat,rays(:,3),rays(:,4));
        [min_d,i] = min(d);
        disp(['Azimuth: ' num2str(rays(i,1))])
        disp(['Launch angle: ' num2str(rays(i,2))])
    else
        break
    end
end
